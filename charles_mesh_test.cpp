#include "mesh_factory.h"
#include "charles_mesh.h"
#include "unit_test.h"

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

namespace charles_mesh
{

TEST(GlobalTest, construct_mesh_with_vertex_polygon)
{
    auto [vertices, polygons] = get_tetrahedron();

    std::shared_ptr<Mesh> mesh(new Mesh(vertices, polygons));
    ObjMeshIO obj_mesh_io = ObjMeshIO();
    obj_mesh_io.save_mesh("./", "tetrahedron", mesh);
}

TEST(GlobalTest, edge_flip)
{
    auto [vertices, polygons] = get_tetrahedron();

    std::shared_ptr<Mesh> mesh(new Mesh(vertices, polygons));
    mesh->edge_flip(mesh->e_head);
    ObjMeshIO obj_mesh_io = ObjMeshIO();
    obj_mesh_io.save_mesh("./", "edge_flip", mesh);
}

TEST(GlobalTest, face_normal)
{
    auto [vertices, polygons] = get_tetrahedron();
    std::shared_ptr<Mesh> mesh(new Mesh(vertices, polygons));
    auto face = mesh->faces[0];
    auto normal = face->normal();
    Vector3D ep_normal{0, 0, -1};
    auto is_same = (normal==ep_normal);
    ASSERT_EQ(is_same, true);
}

TEST(GlobalTest, face_plane)
{
    auto [vertices, polygons] = get_tetrahedron();
    std::shared_ptr<Mesh> mesh(new Mesh(vertices, polygons));
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
    std::shared_ptr<Mesh> mesh(new Mesh(vertices, polygons));
    auto face = mesh->faces[0];
    Point3D point{ 0.1f, 0.1f, 0 };
    ASSERT_EQ(true, face->point_inside(point));
}

TEST(GlobalTest, face_line_segment_intersect)
{
    auto [vertices, polygons] = get_tetrahedron();
    std::shared_ptr<Mesh> mesh(new Mesh(vertices, polygons));
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

        std::shared_ptr<Mesh> mesh1(new Mesh(vertices1, polygons1));
        auto face1 = mesh1->faces[0];

        std::vector<Point3D> vertices2{
            {0.5f, 0, 0},
            {0, 0.5f, 0},
            {1.0f, 1.0f, 1.0f}
        };

        std::vector<std::vector<int>> polygons2{
            { 0, 1, 2 }
        };

        std::shared_ptr<Mesh> mesh2(new Mesh(vertices2, polygons2));
        auto face2 = mesh2->faces[0];

        auto is_intersect = face1->intersect(face2);
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

        std::shared_ptr<Mesh> mesh1(new Mesh(vertices1, polygons1));
        auto face1 = mesh1->faces[0];

        std::vector<Point3D> vertices2{
            {1.0f, 1.0f, 1.0f},
            {2.0f, 1.5f, 2.0f},
            {1.0f, 1.5f, 2.0f}
        };

        std::vector<std::vector<int>> polygons2{
            { 0, 1, 2 }
        };

        std::shared_ptr<Mesh> mesh2(new Mesh(vertices2, polygons2));
        auto face2 = mesh2->faces[0];

        auto is_intersect = face1->intersect(face2);
        ASSERT_EQ(is_intersect, false);
    }
}

TEST(GlobalTest, mesh_intersect)
{
    // test intersect
    {
        auto [vertices, polygons] = get_tetrahedron();
        std::shared_ptr<Mesh> mesh(new Mesh(vertices, polygons));

        std::vector<Point3D> vertices1{
            {0.5f, 0, 0},
            {0, 0.5f, 0},
            {1.0f, 1.0f, 1.0f}
        };

        std::vector<std::vector<int>> polygons1{
            { 0, 1, 2 }
        };

        std::shared_ptr<Mesh> mesh1(new Mesh(vertices1, polygons1));
        auto face1 = mesh1->faces[0];

        ASSERT_EQ(true, mesh->intersect(face1));
    }
    // test not intersect
    {
        auto [vertices, polygons] = get_tetrahedron();
        std::shared_ptr<Mesh> mesh(new Mesh(vertices, polygons));

        std::vector<Point3D> vertices1{
            {1.0f, 1.0f, 1.0f},
            {2.0f, 1.5f, 2.0f},
            {1.0f, 1.5f, 2.0f}
        };

        std::vector<std::vector<int>> polygons1{
            { 0, 1, 2 }
        };

        std::shared_ptr<Mesh> mesh1(new Mesh(vertices1, polygons1));
        auto face1 = mesh1->faces[0];

        ASSERT_EQ(false, mesh->intersect(face1));
    }
}

};