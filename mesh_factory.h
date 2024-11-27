#ifndef __CHARLES_MESH_LIBRARY_FACTORY_H__
#define __CHARLES_MESH_LIBRARY_FACTORY_H__

#include <string>
#include <memory>
#include "charles_mesh.h"

namespace charles_mesh
{

class MeshIO
{
protected:
    std::string mesh_extension;
public:
    virtual std::shared_ptr<Mesh> load_mesh(const std::string& mesh_path) = 0;
    /**
     * @brief save mesh to ${mesh_dir}bunny${this->mesh_extension}
     * 
     * @param mesh_dir: example: /home/charles/
     * @param mesh_name: example: bunny
     * @return std::shared_ptr<Mesh> 
     */
    virtual void save_mesh(const std::string& mesh_dir, const std::string& mesh_name, std::shared_ptr<Mesh> mesh) = 0;
};

class ObjMeshIO : public MeshIO
{
public:
    ObjMeshIO();
    virtual std::shared_ptr<Mesh> load_mesh(const std::string& mesh_path);
    virtual void save_mesh(const std::string& mesh_dir, const std::string& mesh_name, std::shared_ptr<Mesh> mesh);
};

};

#endif