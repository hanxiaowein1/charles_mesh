#include <string>
#include <filesystem>

#include "charles_mesh.h"

namespace
{

void init_mesh(const std::string& mesh_path)
{
    std::filesystem::path fp_mesh_path{mesh_path};
    auto file_extension = fp_mesh_path.extension().string();
}

};
