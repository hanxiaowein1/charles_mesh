#include "viewer_middle_layer.h"


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