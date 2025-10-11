#ifndef __CHARLES_MESH_VIEWER_MIDDLE_LAYER_H__
#define __CHARLES_MESH_VIEWER_MIDDLE_LAYER_H__

#include "hello_qt/choose_file_process_window.h"
#include "basic_type.h"
#include "charles_mesh.h"
namespace charles_mesh
{

void mesh_viewer(std::string mesh_file);
void mesh_viewer(std::shared_ptr<Mesh<Point3D>> mesh);

}

#endif