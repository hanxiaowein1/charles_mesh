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

void Mesh::init(const std::vector<Point3D>& vertices, const std::vector<std::vector<int>>& polygons)
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
            this->vertices[edge]->half_edge = he;
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

Mesh::Mesh(const std::vector<Point3D>& vertices, const std::vector<std::vector<int>>& polygons)
{
    this->init(vertices, polygons);
}

/**
 * @brief flip edge, but take attention, this is based triangle, could not used on other polygon
 *     v1
 *    /  ^
 * e1/    \e3
 *  v      \
 * v2  -e2-> v3
 *  \ <-e4- ^
 * e5\     / e6
 *    v   /
 *     v4
 * @param he: halfedge
 */
void Mesh::edge_flip(std::shared_ptr<HalfEdge> he)
{
    auto e1 = he->prev;
    auto e2 = he;
    auto e3 = he->next;
    auto e4 = e2->opposite;
    auto e5 = e4->next;
    auto e6 = e5->next;

    auto v1 = e3->vertex;
    auto v2 = e1->vertex;
    auto v3 = e2->vertex;
    auto v4 = e5->vertex;

    auto f1 = e2->face;
    auto f2 = e4->face;

    // change half edge connection and its directed vertex and its face
    e2->next = e1;
    e2->prev = e5;
    e2->vertex = v1;

    e5->next = e2;
    e5->prev = e1;
    e5->face = f1;

    e1->prev = e2;
    e1->next = e5;
    e1->face = f1;


    e4->next = e6;
    e4->prev = e3;
    e4->vertex = v4;

    e3->next = e4;
    e3->prev = e6;
    e3->face = f2;

    e6->next = e3;
    e6->prev = e4;
    e6->face = f2;

    // change the half edge of vertex(seems no need to change)

    // change the half edge of face(still no need to change)
}

Mesh::Mesh(const std::string& mesh_file_path)
{
    // TODO: construct mesh by mesh file, such obj/off or other file type
}

bool Mesh::intersect(const std::vector<int>& polygon)
{
    // TODO: init bvh tree for mesh, then check if it intersect with polygon
    return true;
}


};
