#ifndef __CHARLES_MESH_LIBRARY_FACTORY_H__
#define __CHARLES_MESH_LIBRARY_FACTORY_H__

#include <iostream>
#include <string>
#include <fstream>
#include <format>
#include <exception>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <iomanip>
#include <numeric>

#include "charles_mesh.h"

namespace charles_mesh
{

template<typename VData>
class MeshIO
{
protected:
    std::string mesh_extension;
public:
    virtual std::shared_ptr<Mesh<VData>> load_mesh(const std::string& mesh_path) = 0;
    /**
     * @brief save mesh to ${mesh_dir}bunny${this->mesh_extension}
     * 
     * @param mesh_dir: example: /home/charles/
     * @param mesh_name: example: bunny
     * @return std::shared_ptr<Mesh> 
     */
    virtual void save_mesh(const std::string& mesh_dir, const std::string& mesh_name, std::shared_ptr<Mesh<VData>> mesh) = 0;
};

template<typename VData = Point3D>
class ObjMeshIO : public MeshIO<VData>
{
public:
    ObjMeshIO();
    virtual std::shared_ptr<Mesh<VData>> load_mesh(const std::string& mesh_path);
    virtual void save_mesh(const std::string& mesh_dir, const std::string& mesh_name, std::shared_ptr<Mesh<VData>> mesh);
};

template<typename VData>
ObjMeshIO<VData>::ObjMeshIO()
{
    this->mesh_extension = ".obj";
}

/**
 * @brief save mesh to obj file, but cannot save none manifold mesh(will occur empty pointer error(such as opposite is empty, which is impossible in manifold mesh))
 * 
 * @tparam VData: vertex data, such Point3D
 * @param mesh_dir 
 * @param mesh_name 
 * @param mesh 
 */
template<typename VData>
void ObjMeshIO<VData>::save_mesh(const std::string& mesh_dir, const std::string& mesh_name, std::shared_ptr<Mesh<VData>> mesh)
{
    std::vector<std::vector<unsigned int>> faces;

    unsigned int vertex_count = 0;
    std::unordered_map<std::shared_ptr<Vertex<VData>>, unsigned int> vertex_index_cache;
    std::unordered_map<unsigned int, std::shared_ptr<Vertex<VData>>> vertex_index_map;
    std::unordered_set<std::shared_ptr<HalfEdge<VData>>> edge_cache;
    std::queue<std::shared_ptr<HalfEdge<VData>>> edge_queue;
    std::unordered_set<std::shared_ptr<HalfEdge<VData>>> edge_queue_current;
    edge_queue.emplace(mesh->f_head->half_edge);
    edge_queue_current.emplace(mesh->f_head->half_edge);
    int count = 0;
    while(!edge_queue.empty())
    {
        count++;
        if (count % 100 == 0)
        {
            int c = count * 2;
        }
        std::vector<unsigned int> face;
        auto e_start = edge_queue.front();
        auto e_iter = e_start;
        edge_queue.pop();
        // still need to check if it is iterated, because it can be iterated by other edge from same face.
        if(edge_cache.contains(e_iter))
        {
            continue;
        }
        edge_queue_current.erase(e_iter);
        do
        {
            auto vertex = e_iter->vertex;
            auto found_vertex = vertex_index_cache.find(vertex);
            if(found_vertex != vertex_index_cache.end())
            {
                face.emplace_back(found_vertex->second);
            }
            else
            {
                // std::cout << "new vertex emplace: " << vertex_count << std::endl;
                vertex_index_cache.emplace(std::make_pair(vertex, vertex_count));
                vertex_index_map.emplace(std::make_pair(vertex_count, vertex));
                face.emplace_back(vertex_count);
                vertex_count++;
            }
            
            // log this iterated half edge
            edge_cache.emplace(e_iter);

            // get the opposite half edge of current handling edge to queue if the opposite edge is not handled
            if(!edge_cache.contains(e_iter->opposite))
            {
                // if queue hasn't already element, then emplace
                if (!edge_queue_current.contains(e_iter->opposite))
                {
                    edge_queue.emplace(e_iter->opposite);
                    edge_queue_current.emplace(e_iter->opposite);
                }
            }

            e_iter = e_iter->next;
        }
        while(e_iter != e_start);
        // a face was iterated over, save it
        faces.emplace_back(face);
    }

    // write it to obj file
    std::string mesh_path = mesh_dir + mesh_name;
	std::ofstream file(mesh_dir + mesh_name + this->mesh_extension);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << mesh_path << std::endl;
        return;
    }
    file << std::fixed << std::setprecision(std::numeric_limits<double>::digits10);
    for(unsigned int i = 0; i < vertex_count; i++)
    {
        auto vertex = vertex_index_map.at(i);
        file << "v " << vertex->position.x << " " << vertex->position.y << " " << vertex->position.z << std::endl;
    }
    for(const auto& face: faces)
    {
        file << "f";
        for(const auto& vertex: face)
        {
            file << " " << vertex + 1;
        }
        file << std::endl;
    }
	file.close();
}

template<typename VData>
std::shared_ptr<Mesh<VData>> ObjMeshIO<VData>::load_mesh(const std::string& mesh_path)
{
    // read obj mesh file
    std::shared_ptr<Mesh<VData>> mesh(new Mesh<VData>());
    std::ifstream mesh_stream(mesh_path);
    if(!mesh_stream)
    {
        std::string err_msg = std::format("Could not open mesh file: {}", mesh_path);
        std::cerr << err_msg << std::endl;
        throw std::exception(err_msg.c_str());
    }

    std::string line;
    bool first_vertex = true, first_edge = true, first_face = true;
    while(std::getline(mesh_stream, line))
    {
        if (line.substr(0, 2) == "v ")
        {
            // Parse vertex coordinates
            std::shared_ptr<Vertex<VData>> v(new Vertex<VData>());
            if(first_vertex)
            {
                mesh->v_head = v;
                first_vertex = false;
            }
            sscanf(line.c_str(), "v %lf %lf %lf", &v->position.x, &v->position.y, &v->position.z);
            mesh->vertices.emplace_back(v);
        }
        else if (line.substr(0, 2) == "f ")
        {
            // Parse face indices (simplified, assuming only triangles for now)
            int v1, v2, v3;
            sscanf(line.c_str(), "f %d %d %d", &v1, &v2, &v3);
            // Create a face
            std::shared_ptr<Face<VData>> face(new Face<VData>());
            if(first_face)
            {
                mesh->f_head = face;
                first_face = false;
            }
            mesh->faces.emplace_back(face);

            // Create half-edges for the face (more work needed to handle twins and next pointers properly)
            std::shared_ptr<HalfEdge<VData>> he1(new HalfEdge<VData>());
            if(first_edge)
            {
                mesh->e_head = he1;
                first_edge = false;
            }
            std::shared_ptr<HalfEdge<VData>> he2(new HalfEdge<VData>());
            std::shared_ptr<HalfEdge<VData>> he3(new HalfEdge<VData>());

            he1->vertex = mesh->vertices[v1 - 1];
            mesh->vertices[v1 - 1]->half_edge = he1;
            he2->vertex = mesh->vertices[v2 - 1];
            mesh->vertices[v2 - 1]->half_edge = he2;
            he3->vertex = mesh->vertices[v3 - 1];
            mesh->vertices[v3 - 1]->half_edge = he3;

            face->half_edge = he1;

            he1->next = he2;
            he2->next = he3;
            he3->next = he1;

            he1->prev = he3;
            he2->prev = he1;
            he3->prev = he2;

            he1->face = face;
            he2->face = face;
            he3->face = face;

            mesh->half_edges.emplace_back(he1);
            mesh->half_edges.emplace_back(he2);
            mesh->half_edges.emplace_back(he3);
        }
    }

    // setup opposite edge
    for (auto he : mesh->half_edges)
    {
        // Find the corresponding half-edge that goes in the opposite direction
        for (auto other_he : mesh->half_edges)
        {
            if(he->vertex == other_he->prev->vertex && he->prev->vertex == other_he->vertex)
            {
                he->opposite = other_he;
                other_he->opposite = he;
            }
        }
    }

    return mesh;
}

};

#endif