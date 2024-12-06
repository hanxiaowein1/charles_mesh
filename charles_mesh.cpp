#include <string>
#include <filesystem>
#include <limits>

#include "charles_mesh.h"
#include "unit_test.h"
#include "charles_bvh.h"

namespace charles_mesh
{

void init_mesh(const std::string& mesh_path)
{
    std::filesystem::path fp_mesh_path{mesh_path};
    auto file_extension = fp_mesh_path.extension().string();
}

};
