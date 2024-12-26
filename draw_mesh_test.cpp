#include "unit_test.h"
#include "basic_type.h"

#include "triangles_shower.h"
#include "charles_mesh.h"
#include "mesh_factory.h"

extern std::pair<std::vector<charles_mesh::Point3D>, std::vector<std::vector<int>>> get_tetrahedron();

namespace charles_mesh
{

TEST(GlobalTest, show_mesh_test)
{
    // replace V with Point3D
    auto [vertices, triangles] = get_tetrahedron();
    std::vector<Point3D> normals{
        {0, 0, -1},
        {0, -1, 0},
        {-1, 0, 0},
        {0.577350f, 0.577350f, 0.577350f}
    };
    DisplayInfo<Point3D, std::vector<int>> display_info;
    display_info.vertices = vertices;
    display_info.normals = normals;
    display_info.triangles = triangles;
    display_info.r = 0.5f;
    display_info.g = 0.5f;
    display_info.b = 0.5f;

    std::vector<DisplayInfo<Point3D, std::vector<int>>> mul_display_info{ display_info };
    show_triangles_with_model_viewer(mul_display_info);
}


TEST(GlobalTest, draw_small_bunny)
{
    std::string small_bunny = "C:\\Users\\hanxi\\Downloads\\simplify_Bunny3.obj";
    ObjMeshIO obj_mesh_io;
    auto mesh = obj_mesh_io.load_mesh(small_bunny);
    DisplayInfo<Point3D, std::vector<int>> display_info;
    std::unordered_map<decltype(mesh->v_head), int> index_cache;
    int count = 0;
    for(const auto& vertex: mesh->vertices)
    {
        // file << "v " << vertex->position.x << " " << vertex->position.y << " " << vertex->position.z << std::endl;
        display_info.vertices.emplace_back(vertex->position);
        index_cache.emplace(vertex, count);
        count++;
    }

    for (const auto& face : mesh->faces)
    {
        auto iter = face->half_edge;
        auto head = iter;
        std::vector<int> vec_face;
        do
        {
            int index = index_cache.at(iter->vertex);
            vec_face.emplace_back(index);
            iter = iter->next;
        } while (iter != head);
        display_info.triangles.emplace_back(vec_face);
        display_info.normals.emplace_back(face->normal());
    }
    display_info.r = 0.0f;
    display_info.g = 1.0f;
    display_info.b = 0.0f;

    std::vector<DisplayInfo<Point3D, std::vector<int>>> mul_display_info{ display_info };
    show_triangles_with_model_viewer(mul_display_info);
}

};

