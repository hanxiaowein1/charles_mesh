#ifndef __CHARLES_MESH_LIBRARY_H__
#define __CHARLES_MESH_LIBRARY_H__

#include <vector>
#include <memory>
#include "charles_bvh.h"
#include "mesh_type.h"

namespace charles_mesh
{


template <typename VData>
class HalfEdge;

template<typename VData = Point3D>
class Vertex
{
public:
    VData position;
    std::shared_ptr<HalfEdge<VData>> half_edge;
    std::shared_ptr<Vertex<VData>> deep_copy();
};

template <typename VData = Point3D>
class Face : public Object<VData>
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
    std::shared_ptr<HalfEdge<VData>> half_edge;
    virtual double get_min_x();
    virtual double get_max_x();
    virtual double get_min_y();
    virtual double get_max_y();
    virtual double get_min_z();
    virtual double get_max_z();
    void update_bounding();
    virtual bool intersect(std::shared_ptr<Object<VData>> object);
    virtual bool point_inside(const VData& point);
    bool intersect(const VData& point1, const VData& point2, VData& intersect_p);
    VData normal();
    // [a, b, c, d] of ax + by + cz + d = 0
    std::vector<double> plane();
    std::shared_ptr<Face> deep_copy();
};


template<typename VData = Point3D>
class HalfEdge
{
public:
    std::shared_ptr<Vertex<VData>>   vertex;
    std::shared_ptr<Face<VData>>     face;
    std::shared_ptr<HalfEdge<VData>> next;
    std::shared_ptr<HalfEdge<VData>> prev;
    std::shared_ptr<HalfEdge<VData>> opposite;
};

template<typename VData = Point3D>
class Mesh
{
public:
    std::vector<std::shared_ptr<Vertex<VData>>>   vertices;
    std::vector<std::shared_ptr<HalfEdge<VData>>> half_edges;
    std::vector<std::shared_ptr<Face<VData>>>     faces;
public:
    std::shared_ptr<Vertex<VData>>   v_head;
    std::shared_ptr<HalfEdge<VData>> e_head;
    std::shared_ptr<Face<VData>>     f_head;
public:
    Mesh() = default;
    Mesh(const std::string& mesh_file_path);
    Mesh(const std::vector<VData>& vertices, const std::vector<std::vector<int>>& polygons);
public:
    void edge_flip(std::shared_ptr<HalfEdge<VData>> he);
    bool edge_flip_with_intersection_detect(std::shared_ptr<HalfEdge<VData>> he);
    void init(const std::vector<VData>& vertices, const std::vector<std::vector<int>>& polygons);
    // bool intersect(const std::vector<int>& polygon);
    bool intersect(std::shared_ptr<Face<VData>> polygon);
};


template<typename VData>
std::shared_ptr<Vertex<VData>> Vertex<VData>::deep_copy()
{
    // TODO: seems no need to implement?
    return nullptr;
}

template<typename VData>
std::shared_ptr<Face<VData>> Face<VData>::deep_copy()
{
    // we can reuse mesh init function, and get faces[0]
    std::vector<VData> vertices;
    std::vector<std::vector<int>> polygons;
    std::vector<int> polygon;
    auto iter = this->half_edge;
    auto head = this->half_edge;
    int i = 0;
    do
    {
        auto pos = iter->vertex->position;
        vertices.emplace_back(pos);
        polygon.emplace_back(i);
        i++;
        iter = iter->next;
    } while (iter != head);

    polygons.emplace_back(polygon);
    Mesh<VData> mesh(vertices, polygons);
    auto new_face = mesh.faces[0];
    return new_face;
}

template <typename VData>
VData Face<VData>::normal()
{
    // use first three point to calculate normal
    VData face_normal;
    auto iter = this->half_edge;
    std::vector<VData> points;
    for(int i = 0; i < 3; i++)
    {
        points.emplace_back(iter->vertex->position);
        iter = iter->next;
    }
    VData vec1 = points[0] - points[1];
    VData vec2 = points[1] - points[2];
    return vec1.cross(vec2);
}

template <typename VData>
std::vector<double> Face<VData>::plane()
{
    auto normal = this->normal();
    std::vector<double> equation(4);
    VData point = this->half_edge->vertex->position;
    equation[0] = normal[0];
    equation[1] = normal[1];
    equation[2] = normal[2];
    equation[3] = -(normal[0] * point.x + normal[1] * point.y + normal[2] * point.z);
    return equation;
}

template <typename VData>
bool Face<VData>::point_inside(const VData& point)
{
    // check if point is the left/right of all edges
    auto iter = this->half_edge;
    auto head = iter;
    VData first_dir;
    do
    {
        const VData& v0 = iter->prev->vertex->position;
        const VData& v1 = iter->vertex->position;
        VData edge = v1 - v0;
        VData inner_edge = point - v0;
        if(iter == head)
        {
            // initialize first_dir and then do nothing
            first_dir = edge.cross(inner_edge);
        }
        else
        {
            VData current_dir = edge.cross(inner_edge);
            // check if dir is same, if not same, then point is not inside
            if(!first_dir.acute_angle(current_dir))
            {
                return false;
            }
        }
        iter = iter->next;
    } while (iter != head);
    return true;
}

template <typename VData>
bool Face<VData>::intersect(const VData& point1, const VData& point2, VData& intersect_p)
{
    auto equation = this->plane();
    double a = equation[0], b = equation[1], c = equation[2], d = equation[3];
    double x0 = point1.x, y0 = point1.y, z0 = point1.z;
    double x1 = point2.x, y1 = point2.y, z1 = point2.z;
    double denominator = a * (x1 - x0) + b * (y1 - y0) + c * (z1 - z0);
    if (denominator == 0) {
        return false;  // Line is parallel to the plane
    }

    double t = -(a * x0 + b * y0 + c * z0 + d) / denominator;
    intersect_p.x = x0 + t * (x1 - x0);
    intersect_p.y = y0 + t * (y1 - y0);
    intersect_p.z = z0 + t * (z1 - z0);

    // check if intersect_p between point1 and point2

    if(!intersect_p.between(point1, point2))
    {
        return false;
    }

    // should alse check if point inside polygon
    if(!this->point_inside(intersect_p))
    {
        return false;
    }

    return true;
}

/**
 * @brief detect if there are intersection between this and object
 * NOTE: line segment of face has no intersection between face(by unit test)
 * @param object 
 * @return true 
 * @return false 
 */
template <typename VData>
bool Face<VData>::intersect(std::shared_ptr<Object<VData>> object)
{
    auto face = std::dynamic_pointer_cast<Face<VData>>(object);
    // test this and face intersect
    // if has an edge with intersection of other face, then it's intersect, otherwise not intersect
    VData intersect_p;
    {
        auto iter = this->half_edge;
        auto head = iter;
        do
        {
            const auto& point1 = iter->vertex->position;
            const auto& point2 = iter->next->vertex->position;
            if(face->intersect(point1, point2, intersect_p))
            {
                return true;
            }
            iter = iter->next;
        } while (iter != head);
    }

    {
        auto iter2 = face->half_edge;
        auto head2 = iter2;
        do
        {
            auto point1 = iter2->vertex->position;
            auto point2 = iter2->next->vertex->position;
            if(this->intersect(point1, point2, intersect_p))
            {
                return true;
            }
            iter2 = iter2->next;
        } while (iter2 != head2);
    }

    return false;
}

template <typename VData>
double Face<VData>::get_min_x()
{
    if(!this->is_valid)
    {
        this->update_bounding();
    }
    return this->min_x;
}

template <typename VData>
double Face<VData>::get_max_x()
{
    if(!this->is_valid)
    {
        this->update_bounding();
    }
    return this->max_x;
}

template <typename VData>
double Face<VData>::get_min_y()
{
    if(!this->is_valid)
    {
        this->update_bounding();
    }
    return this->min_y;
}

template <typename VData>
double Face<VData>::get_max_y()
{
    if(!this->is_valid)
    {
        this->update_bounding();
    }
    return this->max_y;
}

template <typename VData>
double Face<VData>::get_min_z()
{
    if(!this->is_valid)
    {
        this->update_bounding();
    }
    return this->min_z;
}

template <typename VData>
double Face<VData>::get_max_z()
{
    if(!this->is_valid)
    {
        this->update_bounding();
    }
    return this->max_z;
}

template <typename VData>
void Face<VData>::update_bounding()
{
    auto head = this->half_edge;
    auto iter = this->half_edge;
    do
    {
        auto vertex = iter->vertex;
        if(vertex->position.x < this->min_x)
        {
            this->min_x = vertex->position.x;
        }
        if(vertex->position.x > this->max_x)
        {
            this->max_x = vertex->position.x;
        }

        if(vertex->position.y < this->min_y)
        {
            this->min_y = vertex->position.y;
        }
        if(vertex->position.y > this->max_y)
        {
            this->max_y = vertex->position.y;
        }

        if(vertex->position.z < this->min_z)
        {
            this->min_z = vertex->position.z;
        }
        if(vertex->position.z > this->max_z)
        {
            this->max_z = vertex->position.z;
        }
        iter = iter->next;
    } while (iter != head);
    this->is_valid = true;
}

template <typename VData>
void Mesh<VData>::init(const std::vector<VData>& vertices, const std::vector<std::vector<int>>& polygons)
{
    bool first_vertex = true, first_edge = true, first_face = true;
    for(const auto& vertex: vertices)
    {
        std::shared_ptr<Vertex<VData>> v(new Vertex<VData>());
        if(first_vertex)
        {
            this->v_head = v;
            first_vertex = false;
        }
        v->position = vertex;
        this->vertices.emplace_back(v);
    }

    for(const auto& polygon: polygons)
    {
        std::shared_ptr<Face<VData>> face(new Face<VData>());
        if(first_face)
        {
            this->f_head = face;
            first_face = false;
        }
        this->faces.emplace_back(face);
        bool face_first_edge = true;
        std::vector<std::shared_ptr<HalfEdge<VData>>> temp_half_edges;
        for(const auto& edge: polygon)
        {
            std::shared_ptr<HalfEdge<VData>> he(new HalfEdge<VData>());
            if(first_edge)
            {
                first_edge = false;
                this->e_head = he;
            }
            if(face_first_edge)
            {
                face_first_edge = false;
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

template <typename VData>
Mesh<VData>::Mesh(const std::vector<VData>& vertices, const std::vector<std::vector<int>>& polygons)
{
    this->init(vertices, polygons);
}

template <typename VData>
bool Mesh<VData>::edge_flip_with_intersection_detect(std::shared_ptr<HalfEdge<VData>> he)
{
    // edge flip could only be used in triangle, so we can simplfy it by fixed number
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

    // construct new two faces that is flipped by origin, which is (v1->v2->v4->v1) and (v1->v4->v3->v1)
    std::vector<VData> new_points{
        v1->position, v2->position, v3->position, v4->position
    };

    std::vector<std::vector<int>> new_polygons{
        {0, 1, 3},
        {0, 3, 2}
    };

    std::shared_ptr<Mesh<VData>> new_mesh(new Mesh<VData>(new_points, new_polygons));
    if(this->intersect(new_mesh->faces[0]) || this->intersect(new_mesh->faces[1]))
    {
        return false;
    }

    // do edge flip
    this->edge_flip(he);
    return true;
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
template <typename VData>
void Mesh<VData>::edge_flip(std::shared_ptr<HalfEdge<VData>> he)
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

template <typename VData>
Mesh<VData>::Mesh(const std::string& mesh_file_path)
{
    // TODO: construct mesh by mesh file, such obj/off or other file type
}

/**
 * @brief if mesh is intersect with polygon
 * 
 * @param polygon 
 * @return true 
 * @return false 
 */
// bool Mesh::intersect(const std::vector<int>& polygon)
template <typename VData>
bool Mesh<VData>::intersect(std::shared_ptr<Face<VData>> polygon)
{
    std::vector<std::shared_ptr<Object<VData>>> objects;
    for(auto face: this->faces)
    {
        objects.emplace_back(std::dynamic_pointer_cast<Object<VData>>(face));
    }
    std::shared_ptr<BVHNode<Object<VData>>> bvh_tree = build_bvh(objects, 0, objects.size());
    std::shared_ptr<Object<VData>> object = std::dynamic_pointer_cast<Object<VData>>(polygon);
    if(bvh_intersect(bvh_tree, object))
    {
        return true;
    }
    return false;
}


};


#endif