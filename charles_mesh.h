#ifndef __CHARLES_MESH_LIBRARY_H__
#define __CHARLES_MESH_LIBRARY_H__

#include <vector>
#include <memory>

namespace charles_mesh
{

class Point3D
{
public:
    double x, y, z;
};

class HalfEdge;

class Vertex
{
public:
    Point3D position;
    std::shared_ptr<HalfEdge> half_edge;
};

class Face
{
public:
    std::shared_ptr<HalfEdge> half_edge;
};

class HalfEdge
{
public:
    std::shared_ptr<Vertex>   vertex;
    std::shared_ptr<Face>     face;
    std::shared_ptr<HalfEdge> next;
    std::shared_ptr<HalfEdge> prev;
    std::shared_ptr<HalfEdge> opposite;
};

class Mesh
{
public:
    std::vector<std::shared_ptr<Vertex>>   vertices;
    std::vector<std::shared_ptr<HalfEdge>> half_edges;
    std::vector<std::shared_ptr<Face>>     faces;
public:
    std::shared_ptr<Vertex>   v_head;
    std::shared_ptr<HalfEdge> e_head;
    std::shared_ptr<Face>     f_head;
public:
    Mesh() = default;
    Mesh(const std::string& mesh_file_path) = delete;
    Mesh(const std::vector<Point3D>& vertices, const std::vector<std::vector<int>>& polygons);
};

};


#endif