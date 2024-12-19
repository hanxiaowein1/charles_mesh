#include "mesh_factory.h"
#include "charles_mesh.h"
#include "unit_test.h"
#include <cstdlib>
#include <boost/functional/hash.hpp>
#include <unordered_set>
#include <unordered_map>

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

std::pair<std::vector<charles_mesh::Point3D>, std::vector<std::vector<int>>> get_tetrahedron()
{
    std::vector<charles_mesh::Point3D> vertices{
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    std::vector<std::vector<int>> polygons{
        {0, 2, 1},
        {0, 1, 3},
        {0, 3, 2},
        {1, 2, 3}
    };

    return std::make_pair(vertices, polygons);
}

// see image/square_pyramid_1.jpg
std::pair<std::vector<charles_mesh::Point3D>, std::vector<std::vector<int>>> get_square_pyramid()
{
    std::vector<charles_mesh::Point3D> vertices{
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0},
        {1, 1, 0},
        {0.5f, 0.5f, 1}
    };

    std::vector<std::vector<int>> polygons{
        {0, 3, 1},
        {0, 2, 3},
        {1, 3, 4},
        {3, 2, 4},
        {2, 0, 4},
        {0, 1, 4}
    };

    return std::make_pair(vertices, polygons);
}

// see image/square_pyramid_2.jpg
std::pair<std::vector<charles_mesh::Point3D>, std::vector<std::vector<int>>> get_square_pyramid_2()
{
    std::vector<charles_mesh::Point3D> vertices{
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0},
        {1, 1, 0},
        {0.5f, 0.5f, 1}
    };

    std::vector<std::vector<int>> polygons{
        {0, 2, 1},
        {2, 3, 1},
        {1, 3, 4},
        {3, 2, 4},
        {2, 0, 4},
        {0, 1, 4}
    };

    return std::make_pair(vertices, polygons);
}

namespace charles_mesh
{

TEST(GlobalTest, construct_mesh_with_vertex_polygon)
{
    auto [vertices, polygons] = get_tetrahedron();

    std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));
    ObjMeshIO obj_mesh_io = ObjMeshIO();
    obj_mesh_io.save_mesh("./", "tetrahedron", mesh);
}

TEST(GlobalTest, edge_flip)
{
    auto [vertices, polygons] = get_tetrahedron();

    std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));
    mesh->edge_flip(mesh->e_head);
    ObjMeshIO obj_mesh_io = ObjMeshIO();
    obj_mesh_io.save_mesh("./", "edge_flip", mesh);
}

TEST(GlobalTest, face_normal)
{
    auto [vertices, polygons] = get_tetrahedron();
    std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));
    auto face = mesh->faces[0];
    auto normal = face->normal();
    Vector3D ep_normal{0, 0, -1};
    auto is_same = (normal==ep_normal);
    ASSERT_EQ(is_same, true);
}

TEST(GlobalTest, face_plane)
{
    auto [vertices, polygons] = get_tetrahedron();
    std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));
    auto face = mesh->faces[3];
    auto equation = face->plane();
    ASSERT_EQ(1, equation[0]);
    ASSERT_EQ(1, equation[1]);
    ASSERT_EQ(1, equation[2]);
    ASSERT_EQ(-1, equation[3]);

}

TEST(GlobalTest, face_point_inside)
{
    auto [vertices, polygons] = get_tetrahedron();
    std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));
    auto face = mesh->faces[0];
    Point3D point{ 0.1f, 0.1f, 0 };
    ASSERT_EQ(true, face->point_inside(point));
}

TEST(GlobalTest, face_line_segment_intersect)
{
    auto [vertices, polygons] = get_tetrahedron();
    std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));
    auto face = mesh->faces[3];
    
    {
        Point3D point1{ 0, 0, 0 };
        Point3D point2{ 1, 1, 1 };
        Point3D intersect_p;
        bool is_intersect = face->intersect(point1, point2, intersect_p);
        Point3D expect_p{ 0.33f, 0.33f, 0.33f };
        EXPECT_LT(std::abs(expect_p[0] - intersect_p[0]), 0.01f);
        EXPECT_LT(std::abs(expect_p[1] - intersect_p[1]), 0.01f);
        EXPECT_LT(std::abs(expect_p[2] - intersect_p[2]), 0.01f);
        ASSERT_EQ(true, is_intersect);
    }

    {
        Point3D point1{ 0, 0, 2 };
        Point3D point2{ 1, 1, 3 };
        Point3D intersect_p;
        bool is_intersect = face->intersect(point1, point2, intersect_p);
        ASSERT_EQ(false, is_intersect);
    }
}

TEST(GlobalTest, face_face_intersect)
{
    // test has intersection
    {
        std::vector<Point3D> vertices1{
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
        };

        std::vector<std::vector<int>> polygons1{
            { 0, 1, 2 }
        };

        std::shared_ptr<Mesh<Point3D>> mesh1(new Mesh<Point3D>(vertices1, polygons1));
        auto face1 = mesh1->faces[0];

        std::vector<Point3D> vertices2{
            {0.5f, 0, 0},
            {0, 0.5f, 0},
            {1.0f, 1.0f, 1.0f}
        };

        std::vector<std::vector<int>> polygons2{
            { 0, 1, 2 }
        };

        std::shared_ptr<Mesh<Point3D>> mesh2(new Mesh<Point3D>(vertices2, polygons2));
        auto face2 = mesh2->faces[0];

        auto is_intersect = face1->intersect(face2);
        ASSERT_EQ(is_intersect, true);
    }

    // test if has intersection if they are in same plane
    {
        std::vector<Point3D> vertices1{
            {0, 0, 0},
            {1, 0, 0},
            {0, 1, 0}
        };
        std::vector<std::vector<int>> polygons1{
            { 0, 1, 2 }
        };
        std::shared_ptr<Mesh<Point3D>> mesh1(new Mesh<Point3D>(vertices1, polygons1));
        auto face1 = mesh1->faces[0];


        std::vector<Point3D> vertices2{
            {0, 0, 0},
            {1, 0, 0},
            {1, 1, 0}
        };
        std::vector<std::vector<int>> polygons2{
            { 0, 1, 2 }
        };

        std::shared_ptr<Mesh<Point3D>> mesh2(new Mesh<Point3D>(vertices2, polygons2));
        auto face2 = mesh2->faces[0];

        auto is_intersect = face1->intersect(face2);
        // NOTE: by test, face in same plane has no intersection(strange, but is good for edge flip intersection detection)
        ASSERT_EQ(is_intersect, true);
    }

    // test no intersection
    {
        std::vector<Point3D> vertices1{
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
        };

        std::vector<std::vector<int>> polygons1{
            { 0, 1, 2 }
        };

        std::shared_ptr<Mesh<Point3D>> mesh1(new Mesh<Point3D>(vertices1, polygons1));
        auto face1 = mesh1->faces[0];

        std::vector<Point3D> vertices2{
            {1.0f, 1.0f, 1.0f},
            {2.0f, 1.5f, 2.0f},
            {1.0f, 1.5f, 2.0f}
        };

        std::vector<std::vector<int>> polygons2{
            { 0, 1, 2 }
        };

        std::shared_ptr<Mesh<Point3D>> mesh2(new Mesh<Point3D>(vertices2, polygons2));
        auto face2 = mesh2->faces[0];

        auto is_intersect = face1->intersect(face2);
        ASSERT_EQ(is_intersect, false);
    }
}

TEST(GlobalTest, mesh_intersect_with_exclude)
{
    auto [vertices, polygons] = get_tetrahedron();
    std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));
    
    std::vector<Point3D> vertices1{
        {0.1f, 0.1f, 0.1f},
        {5, 6, 7},
        {6, 7, 8},
    };

    std::vector<std::vector<int>> polygons1{
        {0, 1, 2}
    };

    std::shared_ptr<Mesh<Point3D>> mesh1(new Mesh<Point3D>(vertices1, polygons1));
    auto face1 = mesh1->faces[0];

    ASSERT_EQ(true, mesh->intersect(face1));

    ASSERT_EQ(false, mesh->intersect(face1, { mesh->faces[mesh->faces.size() - 1] }));
}

TEST(GlobalTest, mesh_intersect)
{
    // test intersect
    {
        auto [vertices, polygons] = get_tetrahedron();
        std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));

        std::vector<Point3D> vertices1{
            {0.5f, 0, 0},
            {0, 0.5f, 0},
            {1.0f, 1.0f, 1.0f}
        };

        std::vector<std::vector<int>> polygons1{
            { 0, 1, 2 }
        };

        std::shared_ptr<Mesh<Point3D>> mesh1(new Mesh<Point3D>(vertices1, polygons1));
        auto face1 = mesh1->faces[0];

        ASSERT_EQ(true, mesh->intersect(face1));
    }
    // test not intersect
    {
        auto [vertices, polygons] = get_tetrahedron();
        std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));

        std::vector<Point3D> vertices1{
            {1.0f, 1.0f, 1.0f},
            {2.0f, 1.5f, 2.0f},
            {1.0f, 1.5f, 2.0f}
        };

        std::vector<std::vector<int>> polygons1{
            { 0, 1, 2 }
        };

        std::shared_ptr<Mesh<Point3D>> mesh1(new Mesh<Point3D>(vertices1, polygons1));
        auto face1 = mesh1->faces[0];

        ASSERT_EQ(false, mesh->intersect(face1));
    }
}

TEST(GlobalTest, mesh_edge_flip_with_intersection_detect)
{
    // has intersection, and flip failed
    {
        auto [vertices, polygons] = get_tetrahedron();
        std::vector<Point3D> to_flip_vertices{
            {0, -0.1f, -0.1f},
            {0, -0.1f, 0.9f},
            {0.9f, -1.0f, 0.1f},
            {0, 0.9f, -0.1f}
        };

        std::vector<std::vector<int>> to_flip_polygons{
            {0, 1, 2},
            {0, 2, 3}
        };

        vertices.insert(vertices.end(), to_flip_vertices.begin(), to_flip_vertices.end());
        for (auto& to_flip_polygon : to_flip_polygons)
        {
            for (auto& vertex : to_flip_polygon)
            {
                vertex = vertex + 4;
            }
        }
        polygons.insert(polygons.end(), to_flip_polygons.begin(), to_flip_polygons.end());

        std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));

        //ObjMeshIO obj_mesh_io;
        //obj_mesh_io.save_mesh("./", "test_mesh.obj", mesh);

        // // the 12th edge is what we want to flip
        auto half_edge = mesh->half_edges[12];
        bool flipped = mesh->edge_flip_with_intersection_detect(half_edge);
        ASSERT_EQ(false, flipped);
    }

    // has no intersection, flipped succeed
    {
        auto [vertices, polygons] = get_tetrahedron();

        std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));
        bool flipped = mesh->edge_flip_with_intersection_detect(mesh->half_edges[0]);
        ASSERT_EQ(false, flipped);
    }
}

TEST(GlobalTest, mesh_save_obj)
{
    std::string bunny_obj_src_path = "D:\\PHD\\Projects\\DevelopApp\\DevelopApp\\model\\Bunny.obj";
    ObjMeshIO obj_mesh_io;
    auto mesh = obj_mesh_io.load_mesh(bunny_obj_src_path);
    // obj_mesh_io.save_mesh("./", "bunny.obj", mesh);
    mesh->save_obj("./", "bunny_mesh_save_obj");
}

TEST(GlobalTest, edge_collapse)
{
     //std::string bunny_obj_src_path = "D:\\PHD\\Projects\\DevelopApp\\DevelopApp\\model\\Bunny.obj";
    std::string small_bunny = "C:\\Users\\hanxi\\Downloads\\simplify_Bunny3.obj";
    ObjMeshIO obj_mesh_io;
    auto mesh = obj_mesh_io.load_mesh(small_bunny);
    //obj_mesh_io.save_mesh("./", "small_bunny", mesh);
    //mesh->edge_collapse();
    mesh->edge_collapse();
    mesh->save_obj("./", "small_bunny");
}

TEST(GlobalTest, edge_collapse_intersection_detect)
{
    // not intersect
    {
        auto [vertices, polygons] = get_square_pyramid();
        std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));

        bool is_intersect = mesh->edge_collapse_intersection_detect(mesh->half_edges[11]);
        ASSERT_EQ(false, is_intersect);
    }

    // is intersect
    {
        auto [vertices, polygons] = get_square_pyramid_2();
        std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));

        bool is_intersect = mesh->edge_collapse_intersection_detect(mesh->half_edges[11]);
        ASSERT_EQ(true, is_intersect);
    }
}

TEST(GlobalTest, mesh_duplicate_face_detect)
{
    std::string small_bunny = "C:\\Users\\hanxi\\Downloads\\simplify_Bunny3.obj";
    ObjMeshIO obj_mesh_io;
    auto mesh = obj_mesh_io.load_mesh(small_bunny);
    std::unordered_set<std::unordered_set<int>, boost::hash<std::unordered_set<int>>> face_cache;
    std::unordered_map<std::shared_ptr<Vertex<Point3D>>, int> vertex_index_map;
    for(int i = 0; i < mesh->vertices.size(); i++)
    {
        vertex_index_map.emplace(mesh->vertices[i], i);
    }
    for (const auto& face : mesh->faces)
    {
        std::unordered_set<int> vertex_indices;
        auto head = face->half_edge;
        auto iter = head;
        do
        {
            auto vertex_index = vertex_index_map.at(iter->vertex);
            vertex_indices.emplace(vertex_index);
            iter = iter->next;
        } while (iter != head);
        if(face_cache.contains(vertex_indices))
        {
            throw std::exception("duplicate face detected");
        }
        else
        {
            face_cache.emplace(vertex_indices);
        }
    }
}

TEST(GlobalTest, mesh_is_manifold)
{
    // test is manifold
    {
        auto [vertices, polygons] = get_tetrahedron();
        std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));
        ASSERT_EQ(mesh->is_manifold(), true);
    }
    // test is not manifold
    {
        std::vector<charles_mesh::Point3D> vertices{
            {0, 0, 0},
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
        };

        std::vector<std::vector<int>> polygons{
            {0, 2, 1},
            {0, 1, 3},
            {0, 3, 2},
        };
        std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));
        ASSERT_EQ(mesh->is_manifold(), false);
    }
    // test if obj mesh file is manifold
    {
        std::string small_bunny = "C:\\Users\\hanxi\\Downloads\\simplify_Bunny3.obj";
        ObjMeshIO obj_mesh_io;
        auto mesh = obj_mesh_io.load_mesh(small_bunny);
        ASSERT_EQ(true, mesh->is_manifold());
    }
}

TEST(GlobalTest, mesh_copy_faces)
{
    // copy tetrahedron
    {
        auto [vertices, polygons] = get_tetrahedron();
        std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));
        auto [new_faces, new_half_edges, new_vertices, copied_half_edge] = mesh->copy_faces(mesh->e_head);
        // test copied size correctness
        ASSERT_EQ(new_faces.size(), 4);
        ASSERT_EQ(new_half_edges.size(), 12);
        ASSERT_EQ(new_vertices.size(), 4);
        // test half edge copied correctness
        ASSERT_EQ(mesh->e_head->vertex->position, copied_half_edge->vertex->position);
        ASSERT_EQ(mesh->e_head->opposite->vertex->position, copied_half_edge->opposite->vertex->position);
        ASSERT_EQ(mesh->e_head->prev->vertex->position, copied_half_edge->prev->vertex->position);
        ASSERT_NE(copied_half_edge, mesh->e_head);
        // test connectness
        ASSERT_EQ(mesh->e_head->vertex->half_edge->vertex->position, copied_half_edge->vertex->half_edge->vertex->position);
        ASSERT_EQ(mesh->e_head->vertex->half_edge->vertex->position, copied_half_edge->vertex->position);
        ObjMeshIO obj_mesh_io;
        mesh->faces = new_faces;
        mesh->half_edges = new_half_edges;
        mesh->vertices = new_vertices;
        mesh->e_head = copied_half_edge;

        obj_mesh_io.save_mesh("./", "copy_tetrahedra", mesh);
    }
}

TEST(GlobalTest, mesh_get_sorround_faces)
{
    auto [vertices, polygons] = get_tetrahedron();
    std::shared_ptr<Mesh<Point3D>> mesh(new Mesh<Point3D>(vertices, polygons));
    auto sorround_faces = mesh->get_sorround_faces(mesh->e_head);
    for (auto face : mesh->faces)
    {
        auto it = std::find(sorround_faces.begin(), sorround_faces.end(), face);
        ASSERT_NE(it, sorround_faces.end());
    }
}

void printProgress(double percentage) {
    int val = (int)(percentage * 100);
    int lpad = (int)(percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

};