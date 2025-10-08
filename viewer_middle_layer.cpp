#include "viewer_middle_layer.h"
#include "triangles_shower.h"
#include "charles_mesh.h"
#include "mesh_factory.h"

namespace charles_mesh
{

void mesh_viewer(std::string mesh_file)
{
    ObjMeshIO obj_mesh_io;
    auto mesh = obj_mesh_io.load_mesh(mesh_file);
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

}

int main()
{
    // choose mesh to view
    choose_file_to_process(std::function<void(std::string)>(
        [](std::string mesh_path)
        {
            charles_mesh::mesh_viewer(mesh_path);
        }
    ));
    return 0;
}
