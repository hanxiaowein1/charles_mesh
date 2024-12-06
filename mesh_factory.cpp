#include "mesh_factory.h"
#include "charles_mesh.h"
#include "unit_test.h"



namespace charles_mesh
{

TEST(GlobalTest, bunny_read_write_test)
{
    std::string bunny_obj_src_path = "D:\\PHD\\Projects\\DevelopApp\\DevelopApp\\model\\Bunny.obj";
    ObjMeshIO obj_mesh_io;
    auto mesh = obj_mesh_io.load_mesh(bunny_obj_src_path);
    obj_mesh_io.save_mesh("./", "bunny.obj", mesh);
}

};

