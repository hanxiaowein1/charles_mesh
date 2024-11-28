#include "mesh_factory.h"
#include "charles_mesh.h"
#include "unit_test.h"

namespace charles_mesh
{

TEST(GlobalTest, construct_mesh_with_vertex_polygon)
{
    std::vector<Point3D> vertices{
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

    std::shared_ptr<Mesh> mesh(new Mesh(vertices, polygons));
    ObjMeshIO obj_mesh_io = ObjMeshIO();
    obj_mesh_io.save_mesh("./", "tetrahedron", mesh);
}

TEST(GlobalTest, edge_flip)
{
    std::vector<Point3D> vertices{
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

    std::shared_ptr<Mesh> mesh(new Mesh(vertices, polygons));
    mesh->edge_flip(mesh->e_head);
    ObjMeshIO obj_mesh_io = ObjMeshIO();
    obj_mesh_io.save_mesh("./", "edge_flip", mesh);
}

};