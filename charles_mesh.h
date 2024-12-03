#ifndef __CHARLES_MESH_LIBRARY_H__
#define __CHARLES_MESH_LIBRARY_H__

#include <vector>
#include <memory>
#include "charles_bvh.h"
#include "mesh_type.h"

namespace charles_mesh
{

class HalfEdge;

class Vertex
{
public:
    Point3D position;
    std::shared_ptr<HalfEdge> half_edge;
};

class Face : public Object
{
private:
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::min();
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::min();
    bool is_valid = false;
public:
    std::shared_ptr<HalfEdge> half_edge;
    virtual double get_min_x();
    virtual double get_max_x();
    virtual double get_min_y();
    virtual double get_max_y();
    virtual double get_min_z();
    virtual double get_max_z();
    void update_bounding();
    virtual bool intersect(std::shared_ptr<Object> object);
    virtual bool point_inside(const Point3D& point);
    bool intersect(const Point3D& point1, const Point3D& point2, Point3D& intersect_p);
    Vector3D normal();
    // [a, b, c, d] of ax + by + cz + d = 0
    std::vector<double> plane();
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
    Mesh(const std::string& mesh_file_path);
    Mesh(const std::vector<Point3D>& vertices, const std::vector<std::vector<int>>& polygons);
public:
    void edge_flip(std::shared_ptr<HalfEdge> he);
    void init(const std::vector<Point3D>& vertices, const std::vector<std::vector<int>>& polygons);
    // bool intersect(const std::vector<int>& polygon);
    bool intersect(std::shared_ptr<Face> polygon);
};

};


#endif