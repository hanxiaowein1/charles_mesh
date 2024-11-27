#include <string>
#include <filesystem>

#include "charles_mesh.h"
#include "unit_test.h"

namespace charles_mesh
{

void init_mesh(const std::string& mesh_path)
{
    std::filesystem::path fp_mesh_path{mesh_path};
    auto file_extension = fp_mesh_path.extension().string();
}

Mesh::Mesh(const std::vector<Point3D>& vertices, const std::vector<std::vector<int>>& polygons)
{
    bool first_vertex = true, first_edge = true, first_face = true;
    for(const auto& vertex: vertices)
    {
        std::shared_ptr<Vertex> v(new Vertex());
        if(first_vertex)
        {
            this->v_head = v;
            first_vertex = false;
        }
        v->position.x = vertex.x;
        v->position.y = vertex.y;
        v->position.z = vertex.z;
        this->vertices.emplace_back(v);
    }

    for(const auto& polygon: polygons)
    {
        std::shared_ptr<Face> face(new Face());
        if(first_face)
        {
            this->f_head = face;
            first_face = false;
        }
        this->faces.emplace_back(face);

        std::vector<std::shared_ptr<HalfEdge>> temp_half_edges;
        for(const auto& edge: polygon)
        {
            std::shared_ptr<HalfEdge> he(new HalfEdge());
            if(first_edge)
            {
                first_edge = false;
                this->e_head = he;
                face->half_edge = he;
            }
            he->vertex = this->vertices[edge];
            he->face = face;
            temp_half_edges.emplace_back(he);
        }
        for(int he_index = 0; he_index < temp_half_edges.size(); he_index++)
        {
            auto he_first = temp_half_edges[he_index % (temp_half_edges.size())];
            auto he_second = temp_half_edges[(he_index + 1) % (temp_half_edges.size())];
            he_first->next = he_second;
            he_second->prev = he_first;
        }
        this->half_edges.insert(this->half_edges.end(), temp_half_edges.begin(), temp_half_edges.end());
    }

    // setup opposite edge
    for (auto he : this->half_edges)
    {
        // Find the corresponding half-edge that goes in the opposite direction
        for (auto other_he : this->half_edges)
        {
            if(he->vertex == other_he->prev->vertex && he->prev->vertex == other_he->vertex)
            {
                he->opposite = other_he;
                other_he->opposite = he;
            }
        }
    }
}

};
