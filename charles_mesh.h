#ifndef __CHARLES_MESH_LIBRARY_H__
#define __CHARLES_MESH_LIBRARY_H__

#include <vector>
#include <memory>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <algorithm>
#include <limits>
#include <boost/functional/hash.hpp>
#include "charles_bvh.h"
#include "basic_type.h"
#include "quadric_error_metrics.h"
#include "charles_sort.h"
#include "triangles_shower.h"
#include "print_tools.h"

template<typename DataType>
void erase_from_vector(std::vector<DataType>& vec, std::unordered_set<DataType>& elems)
{
    auto new_end = std::remove_if(vec.begin(), vec.end(), 
        [&elems](DataType value) {
            return elems.find(value) != elems.end();
        });
    vec.erase(new_end, vec.end());
}


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
    // NOTE: current not support R with void type
    template<typename R, typename... Args>
    std::vector<R> handle_in_edge(std::function<R(std::shared_ptr<HalfEdge<VData>>, Args...)> callback, Args... args);
    std::tuple<std::unordered_set<std::shared_ptr<Vertex<VData>>>, std::unordered_set<std::shared_ptr<Vertex<VData>>>> get_connected_vertices();
    // NOTE: only can be used in triangle mesh
    std::unordered_set<std::shared_ptr<HalfEdge<VData>>> get_opposite_half_edges();
    std::unordered_set<std::pair<VData, VData>> get_opposite_connected_vertices();
    void reset();
};

template<typename VData>
template<typename R, typename... Args>
std::vector<R> Vertex<VData>::handle_in_edge(std::function<R(std::shared_ptr<HalfEdge<VData>>, Args...)> callback, Args... args)
{
    std::vector<R> ret;
    auto start_in_edge = this->half_edge;
    auto in_edge_iter = start_in_edge;
    do
    {
        R res = callback(in_edge_iter, args...);
        ret.emplace_back(res);
        in_edge_iter = in_edge_iter->next->opposite;
    } while (in_edge_iter != start_in_edge);
    return ret;
}

template<typename VData>
std::tuple<std::unordered_set<std::shared_ptr<Vertex<VData>>>, std::unordered_set<std::shared_ptr<Vertex<VData>>>> Vertex<VData>::get_connected_vertices()
{
    std::unordered_set<std::shared_ptr<Vertex<VData>>> upstream_vertices;
    std::unordered_set<std::shared_ptr<Vertex<VData>>> downstream_vertices;
    auto lambda = [&](std::shared_ptr<HalfEdge<VData>> in_edge) -> bool {
        upstream_vertices.emplace(in_edge->prev->vertex);
        downstream_vertices.emplace(in_edge->opposite->vertex);
        return true;
    };
    std::function<bool(std::shared_ptr<HalfEdge<VData>>)> lambda_func = lambda;
    this->handle_in_edge(lambda_func);
    return std::make_tuple(upstream_vertices, downstream_vertices);
}

template <typename VData>
std::unordered_set<std::shared_ptr<HalfEdge<VData>>> Vertex<VData>::get_opposite_half_edges()
{
    std::unordered_set<std::shared_ptr<HalfEdge<VData>>> opposite_half_edges;
    auto lambda = [&](std::shared_ptr<HalfEdge<VData>> in_edge) -> bool {
        opposite_half_edges.emplace(in_edge->next->next);
        return true;
    };
    std::function<bool(std::shared_ptr<HalfEdge<VData>>)> lambda_func = lambda;
    this->handle_in_edge(lambda_func);
    return opposite_half_edges;
}

template <typename VData>
std::unordered_set<std::pair<VData, VData>> Vertex<VData>::get_opposite_connected_vertices()
{
    std::unordered_set<std::pair<VData, VData>> opposite_connected_vertices;
    auto lambda = [&](std::shared_ptr<HalfEdge<VData>> in_edge) -> bool {
        auto start_pos = in_edge->next->vertex->position;
        auto end_pos = in_edge->prev->vertex->position;
        opposite_connected_vertices.emplace(std::make_pair(start_pos, end_pos));
        return true;
    };
    std::function<bool(std::shared_ptr<HalfEdge<VData>>)> func = lambda;
    this->handle_in_edge(func);
    return opposite_connected_vertices;
}

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
    std::vector<double> m_plane;
    VData m_normal;
    bool normal_initialized = false;
    VData calculate_normal();
    std::vector<double> calculate_plane();
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
    void update_normal();
    // [a, b, c, d] of ax + by + cz + d = 0
    std::vector<double> plane();
    void update_plane();
    std::shared_ptr<Face> deep_copy();
    void reset();
};

template<typename VData = Point3D>
class HalfEdge
{
private:
    // TODO: metrics is computed by vertex, so half edge and it's opposite has save metrics, need to avoid duplicate computation
    VData m_quadric_error_metrics_point;
    double m_quadric_error_metrics;
    bool quadric_error_metrics_initialized = false;
public:
    std::shared_ptr<Vertex<VData>>   vertex;
    std::shared_ptr<Face<VData>>     face;
    std::shared_ptr<HalfEdge<VData>> next;
    std::shared_ptr<HalfEdge<VData>> prev;
    std::shared_ptr<HalfEdge<VData>> opposite;
    std::tuple<VData, double> quadric_error_metrics();
    std::tuple<VData, double> calculate_quadric_error_metrics();
    void update_quadric_error_metrics();
    void update_quadric_error_metrics(double metrics);
    void reset();
};

template<typename VData = Point3D>
class EdgeMetricsComparator
{
public:
    bool operator()(const std::shared_ptr<HalfEdge<VData>>& a, const std::shared_ptr<HalfEdge<VData>>& b)
    {
        if(a == nullptr && b == nullptr)
        {
            return true;
        }
        if(a == nullptr)
        {
            return false;
        }
        if(b == nullptr)
        {
            return true;
        }
        auto [a_point, a_metrics] = a->quadric_error_metrics();
        auto [b_point, b_metrics] = b->quadric_error_metrics();
        return a_metrics < b_metrics;
    }
};

template<typename VData = Point3D>
class Mesh
{
public:
    std::vector<std::shared_ptr<Vertex<VData>>>   vertices;
    std::vector<std::shared_ptr<HalfEdge<VData>>> half_edges;
    std::vector<std::shared_ptr<Face<VData>>>     faces;
    bool debug_viewer = false;
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
    bool intersect(std::shared_ptr<Face<VData>> polygon, std::unordered_set<std::shared_ptr<Face<VData>>> exclude_polygons = std::unordered_set<std::shared_ptr<Face<VData>>>());
    bool intersect(std::vector<std::shared_ptr<Face<VData>>>& polygons, std::unordered_set<std::shared_ptr<Face<VData>>> exclude_polygons = std::unordered_set<std::shared_ptr<Face<VData>>>());
    // can save non manifold mesh
    void save_obj(const std::string& mesh_dir, const std::string& mesh_name);
    void save_obj(const std::string& mesh_path);
    // surface simplification using quadric error metrics
    void edge_collapse(int collapse_times = 10);
    void edge_collapse(std::shared_ptr<HalfEdge<VData>> half_edge);
    bool edge_collapse_intersection_detect(std::shared_ptr<HalfEdge<VData>> half_edge);
    bool is_manifold();
    bool is_manifold(std::shared_ptr<HalfEdge<VData>> half_edge, std::vector<std::shared_ptr<Face<VData>>> faces);
    std::tuple<
        std::vector<std::shared_ptr<Face<VData>>>,
        std::vector<std::shared_ptr<HalfEdge<VData>>>,
        std::vector<std::shared_ptr<Vertex<VData>>>,
        std::shared_ptr<HalfEdge<VData>>
    > copy_faces(std::shared_ptr<HalfEdge<VData>> half_edge);
    std::unordered_set<std::shared_ptr<Face<VData>>> get_sorround_faces(std::shared_ptr<HalfEdge<VData>> half_edge);
    std::unordered_set<std::pair<VData, VData>> get_boundary_connected_vertices(std::shared_ptr<HalfEdge<VData>> half_edge);
};

template <typename VData>
std::vector<std::shared_ptr<Vertex<VData>>> get_vertices(std::vector<std::shared_ptr<Face<VData>>>& polygons)
{
    std::unordered_set<std::shared_ptr<Vertex<VData>>> vertices;
    std::vector<std::shared_ptr<Vertex<VData>>> ret;
    for (auto& polygon : polygons)
    {
        auto he_header = polygon->half_edge;
        auto he_iter = he_header;
        do
        {
            vertices.emplace(he_iter->vertex);
            he_iter = he_iter->next;
        } while (he_iter != he_header);
    }
    for (auto& vertex : vertices)
    {
        ret.emplace_back(vertex);
    }
    return ret;
}


template<typename VData>
std::shared_ptr<Vertex<VData>> Vertex<VData>::deep_copy()
{
    // TODO: seems no need to implement?
    return nullptr;
}

template<typename VData>
void Vertex<VData>::reset()
{
    this->half_edge = nullptr;
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

template<typename VData>
void Face<VData>::reset()
{
    this->half_edge = nullptr;
}

template <typename VData>
VData Face<VData>::calculate_normal()
{
    // use first three point to calculate normal
    VData face_normal;
    auto iter = this->half_edge;
    std::vector<VData> points;
    for (int i = 0; i < 3; i++)
    {
        points.emplace_back(iter->vertex->position);
        iter = iter->next;
    }
    VData vec1 = points[0] - points[1];
    VData vec2 = points[1] - points[2];
    return vec1.cross(vec2);
}

template <typename VData>
std::vector<double> Face<VData>::calculate_plane()
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
VData Face<VData>::normal()
{
    if (!this->normal_initialized)
    {
        this->update_normal();
    }
    return this->m_normal;
}

template <typename VData>
void Face<VData>::update_normal()
{
    this->normal_initialized = true;
    this->m_normal = this->calculate_normal();
}

template <typename VData>
void Face<VData>::update_plane()
{
    this->m_plane = this->calculate_plane();
}

template <typename VData>
std::vector<double> Face<VData>::plane()
{
    if (this->m_plane.size() == 0)
    {
        this->m_plane = this->calculate_plane();
    }
    return this->m_plane;
}

/**
 * @brief requirements: point must in the plane of face
 * limitation: not support concave polygon, only can be used in convex polygon
 * 
 * @tparam VData 
 * @param point 
 * @return true 
 * @return false 
 */
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
            // check if dir is same, if not same, then point is not inside(if dir is zero, then point lying on boundary, we let it not inside)
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
    // Line is parallel to the plane
    if (denominator == 0)
    {
        // check if line on plane
        if(a * point1.x + b * point1.y + c * point1.z + d == 0)
        {
            // if one the plane, check if one of the point inside face, if inside, then has intersection
            if(this->point_inside(point1))
            {
                return true;
            }
            if(this->point_inside(point2))
            {
                return true;
            }
            // there are still another situation, if these two points on boundary of different edges, and by judgement they are all outside of face, but the points between these two points are inner face, so it has intersection
            if(this->point_inside(center(point1, point2)))
            {
                return true;
            }
            return false;
        }
        return false;
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
    // TODO: deviation problem, if point is very close to boundary(very very close, should not let inside)
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
std::tuple<VData, double> HalfEdge<VData>::quadric_error_metrics()
{
    if (!this->quadric_error_metrics_initialized)
    {
        if (this->opposite != nullptr)
        {
            if (this->opposite->quadric_error_metrics_initialized)
            {
                std::tie(this->m_quadric_error_metrics_point, this->m_quadric_error_metrics) = this->opposite->quadric_error_metrics();
            }
            else
            {
                this->update_quadric_error_metrics();
            }
        }
        else
        {
            this->update_quadric_error_metrics();
        }
    }
    return { this->m_quadric_error_metrics_point, this->m_quadric_error_metrics };
}

template <typename VData>
std::tuple<VData, double> HalfEdge<VData>::calculate_quadric_error_metrics()
{
    const auto& vertex1 = this->prev->vertex;
    const auto& vertex2 = this->vertex;
    auto lambda_func = [&](std::shared_ptr<HalfEdge<VData>> in_edge) -> std::vector<double> {
        // get face
        auto face = in_edge->face;
        // get plane
        auto plane = face->plane();
        return plane;
    };
    std::function<std::vector<double>(std::shared_ptr<HalfEdge<VData>>)> func = lambda_func;
    std::vector<std::vector<double>> planes1 = vertex1->handle_in_edge(func);
    //std::vector<std::vector<double>> planes1 = handle_in_edge(vertex1, func);
    //std::vector<std::vector<double>> planes2 = handle_in_edge(vertex2, func);
    std::vector<std::vector<double>> planes2 = vertex2->handle_in_edge(func);
    auto planes = planes1;
    planes.insert(planes.end(), planes2.begin(), planes2.end());

    // calculate quadric error metrics
    {
        std::vector<std::tuple<double, double, double, double>> inner_planes;
        for (auto plane : planes)
        {
            inner_planes.emplace_back(std::make_tuple(plane[0], plane[1], plane[2], plane[3]));
        }
        auto [x_value, y_value, z_value, metrics] = ::quadric_error_metrics(inner_planes);
        VData point;
        point[0] = x_value;
        point[1] = y_value;
        point[2] = z_value;
        return { point, metrics };
    }
}

template <typename VData>
void HalfEdge<VData>::update_quadric_error_metrics()
{
    std::tie(this->m_quadric_error_metrics_point, this->m_quadric_error_metrics) = this->calculate_quadric_error_metrics();
    this->quadric_error_metrics_initialized = true;
}

template <typename VData>
void HalfEdge<VData>::update_quadric_error_metrics(double metrics)
{
    this->m_quadric_error_metrics = metrics;
}

template <typename VData>
void HalfEdge<VData>::reset()
{
    this->vertex = nullptr;
    this->face = nullptr;
    this->next = nullptr;
    this->prev = nullptr;
    this->opposite = nullptr;
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
    std::unordered_set<std::shared_ptr<Face<VData>>> excluded_faces{f1, f2};
    if (this->intersect(new_mesh->faces, excluded_faces))
    {
        return false;
    }
    //if(this->intersect(new_mesh->faces[0]) || this->intersect(new_mesh->faces[1]))
    //{
    //    return false;
    //}

    // do edge flip
    this->edge_flip(he);
    return true;
}


/**
 * @brief flip edge, but take attention, this is based triangle, could not used on other polygon
 *     v1
 *    /  ^
 * e1/ f1 \e3
 *  v      \
 * v2  -e2-> v3
 *  \ <-e4- ^
 * e5\  f2 / e6
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

    // update face normal and plane
    f1->update_normal();
    f1->update_bounding();
    f1->update_plane();

    f2->update_normal();
    f2->update_bounding();
    f2->update_plane();
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
bool Mesh<VData>::intersect(std::shared_ptr<Face<VData>> polygon, std::unordered_set<std::shared_ptr<Face<VData>>> exclude_polygons)
{
    std::vector<std::shared_ptr<Object<VData>>> objects;
    for(auto face: this->faces)
    {
        if (exclude_polygons.contains(face))
        {
            continue;
        }
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

template <typename VData>
bool Mesh<VData>::intersect(std::vector<std::shared_ptr<Face<VData>>>& polygons, std::unordered_set<std::shared_ptr<Face<VData>>> exclude_polygons)
{
    std::vector<std::shared_ptr<Object<VData>>> objects;
    for(auto face: this->faces)
    {
        if (exclude_polygons.contains(face)) 
        {
            continue;
        }
        objects.emplace_back(std::dynamic_pointer_cast<Object<VData>>(face));
    }
    if (objects.size() == 0)
    {
        return false;
    }
    std::shared_ptr<BVHNode<Object<VData>>> bvh_tree = build_bvh(objects, 0, objects.size());
    for(auto polygon: polygons)
    {
        std::shared_ptr<Object<VData>> object = std::dynamic_pointer_cast<Object<VData>>(polygon);
        if(bvh_intersect(bvh_tree, object))
        {
            return true;
        }
    }
    return false;
}

template<typename VData>
void Mesh<VData>::save_obj(const std::string& mesh_path)
{
    std::ofstream file(mesh_path);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << mesh_path << std::endl;
        return;
    }
    file << std::fixed << std::setprecision(std::numeric_limits<double>::digits10);
    // save index cache to better search
    std::unordered_map<decltype(this->v_head), int> index_cache;
    int count = 0;
    for(const auto& vertex: this->vertices)
    {
        file << "v " << vertex->position.x << " " << vertex->position.y << " " << vertex->position.z << std::endl;
        index_cache.emplace(vertex, count);
        count++;
    }

    for (const auto& face : faces)
    {
        file << "f";
        auto iter = face->half_edge;
        auto head = iter;
        do
        {
            int index = index_cache.at(iter->vertex);
            file << " " << index + 1;
            iter = iter->next;
        } while (iter != head);
        file << std::endl;
    }

    file.close();
}

template<typename VData>
void Mesh<VData>::save_obj(const std::string& mesh_dir, const std::string& mesh_name)
{
    // just use member vertices and faces to save
    std::string mesh_path = std::format("{}{}.obj", mesh_dir, mesh_name);
    this->save_obj(mesh_path);
}

/**
 * @brief 

 * @tparam VData 
 */
template <typename VData>
void Mesh<VData>::edge_collapse(int collapse_times)
{
    // calculate all metrics of half edge
    std::vector<std::shared_ptr<HalfEdge<VData>>> sorted_half_edges;
    sorted_half_edges.insert(sorted_half_edges.end(), this->half_edges.begin(), this->half_edges.end());
    std::sort(sorted_half_edges.begin(), sorted_half_edges.end(), EdgeMetricsComparator<VData>());
    std::unordered_set<std::shared_ptr<HalfEdge<VData>>> visited_edges;
    std::cout << "after sort" << std::endl;
    for (int i = 0; i < collapse_times; i++)
    {
        // printProgress(double(i) / double(collapse_times - 1));
        std::cout << i << "/" << collapse_times - 1 << std::endl;
        // get half edge of max metrics
        std::shared_ptr<HalfEdge<VData>> half_edge;
        for (int j = 0; j < this->half_edges.size(); j++)
        {
            auto temp_half_edge = sorted_half_edges[j];
            auto [position, metrics] = temp_half_edge->quadric_error_metrics();
            if (metrics == std::numeric_limits<double>::max())
            {
                break;
            }
            if (!this->edge_collapse_intersection_detect(temp_half_edge))
            {
                half_edge = temp_half_edge;
                break;
            }
        }
        if (!half_edge)
        {
            std::cout << "all half edge collapse will result in non manifold or self intersection" << std::endl;
            break;
        }
        //auto half_edge = sorted_half_edges[0];
        auto e1 = half_edge->prev;
        auto e1_op = e1->opposite;
        auto e2 = half_edge;
        auto e3 = half_edge->next;
        auto e3_op = e3->opposite;
        auto e4 = e2->opposite;
        auto e5 = e4->next;
        auto e5_op = e5->opposite;
        auto e6 = e5->next;
        auto e6_op = e6->opposite;

        if (visited_edges.contains(e1) || visited_edges.contains(e2) || visited_edges.contains(e3) || visited_edges.contains(e4) || visited_edges.contains(e5) || visited_edges.contains(e6))
        {
            throw std::exception("why give me visited edges?");
        }

        auto v1 = e3->vertex;
        auto v2 = e1->vertex;
        auto v3 = e2->vertex;
        auto v4 = e5->vertex;

        auto f1 = e2->face;
        auto f2 = e4->face;

        // update v2 new position
        auto [position, metrics] = e2->quadric_error_metrics();
        v2->position = position;

        // update vertex half edge(v1, v2, v4)
        v1->half_edge = e1_op;
        v2->half_edge = e5_op;
        v4->half_edge = e6_op;

        // change edge that point to v3 to v2 (there exists a trap, see edge_collapse_with_duplicate.jpg under image directory, if vx->v3 and v2->vx, then after collapse, [vx, v2(origin v3), v4] and [v4, v2, vx] are same face but with different orientation, which is duplicate and should erase one of them(situation like this means it will already caused non manifoldness accur, so we can delete any one of them, here we delete v3, should also take care that before delete this face and edge, you should let opposite half edge's opposite to nullptr first))
        auto lambda_func1 = [&](decltype(v2->half_edge) in_edge, decltype(v2) vertex_point_to) -> bool {
            if (in_edge == e2 || in_edge == e6)
            {
                return false;
            }
            in_edge->vertex = vertex_point_to;
            return true;
        };
        std::function<bool(decltype(v2->half_edge), decltype(v2))> change_point_vertex_func = lambda_func1;
        // NOTE: WHY this is not work?
         //handle_in_edge(v3, lambda_func1, v2);
        v3->handle_in_edge<bool, decltype(v2)>(lambda_func1, v2);
        //handle_in_edge<VData, bool, decltype(v2)>(v3, lambda_func1, v2);

        // change half edge of retained
        e1_op->opposite = e3_op;
        e3_op->opposite = e1_op;
        e5_op->opposite = e6_op;
        e6_op->opposite = e5_op;

        std::unordered_set<std::shared_ptr<HalfEdge<VData>>> half_edges_to_delete;
        std::unordered_set<std::shared_ptr<Face<VData>>> faces_to_delete;
        std::unordered_set<std::shared_ptr<Vertex<VData>>> vertice_to_delete;

        // reset all obseleted edge vertex and face shared_ptr to nullptr(to avoid memory cannot be freed because of loop reference)
        e1->reset();
        e2->reset();
        e3->reset();
        e4->reset();
        e5->reset();
        e6->reset();
        half_edges_to_delete.insert({e1, e2, e3, e4, e5, e6});

        v3->reset();
        vertice_to_delete.emplace(v3);

        f1->reset();
        faces_to_delete.emplace(f1);
        f2->reset();
        faces_to_delete.emplace(f2);

        // delete at once
        {
            // delete half edges
            erase_from_vector(this->half_edges, half_edges_to_delete);
            erase_from_vector(sorted_half_edges, half_edges_to_delete);
        }
        {
            // delete faces
            erase_from_vector(this->faces, faces_to_delete);
        }
        {
            // delete vertice
            erase_from_vector(this->vertices, vertice_to_delete);
        }

        // update the properties of face and half edge
        auto lambda_func2 = [&](decltype(v2->half_edge) in_edge) -> bool {
            in_edge->face->update_bounding();
            in_edge->face->update_normal();
            in_edge->face->update_plane();
            auto edge_iter = in_edge;
            do
            {
                edge_iter->update_quadric_error_metrics();
                sort_sorted_vec_with_one_changed(sorted_half_edges, edge_iter, EdgeMetricsComparator<VData>());
                edge_iter = edge_iter->next;
            } while (edge_iter != in_edge);
            return true;
        };
        std::function<bool(decltype(v2->half_edge))> update_properties_func = lambda_func2;
        v2->handle_in_edge(update_properties_func);

        visited_edges.emplace(e1);
        visited_edges.emplace(e2);
        visited_edges.emplace(e3);
        visited_edges.emplace(e4);
        visited_edges.emplace(e5);
        visited_edges.emplace(e6);
    }
}

/**
 * @brief e2 is for collapse, can be considered as next sub processes:
 * (1) all edge connected to v3 changed to v2(except edge for deletion, e2, e6)
 * (2) change half edge connection
 * (3) delete all useless edge(e1~e6) and vertex(v3) (may be no need to, shared_ptr will auto release)
 * (4) update e2 position
 *     v1
 *    /  ^
 * e1/ f1 \e3
 *  v      \
 * v2  -e2-> v3
 *  \ <-e4- ^
 * e5\  f2 / e6
 *    v   /
 *     v4
 * @tparam VData 
 * @param half_edge: edge to be collapsed
 */
template <typename VData>
void Mesh<VData>::edge_collapse(std::shared_ptr<HalfEdge<VData>> half_edge)
{

    //handle_in_edge(v2, update_properties_func);

}

/**
 * @brief copy faces around half edge, and test if the new collapsed faces will intersect the remain old faces
 * 
 * @tparam VData Vertex datatype
 * @param half_edge: half edge to collapse
 * @return true: find intersection
 * @return false: don't find an intersection
 */
template <typename VData>
bool Mesh<VData>::edge_collapse_intersection_detect(std::shared_ptr<HalfEdge<VData>> half_edge)
{
    auto [new_faces, new_half_edges, new_vertices, new_half_edge_to_collapse] = this->copy_faces(half_edge);

    // new half edge collapse
    auto e1 = new_half_edge_to_collapse->prev;
    auto e1_op = e1->opposite;
    auto e2 = new_half_edge_to_collapse;
    auto e3 = new_half_edge_to_collapse->next;
    auto e3_op = e3->opposite;
    auto e4 = e2->opposite;
    auto e5 = e4->next;
    auto e5_op = e5->opposite;
    auto e6 = e5->next;
    auto e6_op = e6->opposite;

    auto v1 = e3->vertex;
    auto v2 = e1->vertex;
    auto v3 = e2->vertex;
    auto v4 = e5->vertex;

    auto f1 = e2->face;
    auto f2 = e4->face;

    std::unordered_set<std::shared_ptr<Face<VData>>> excluded_faces = this->get_sorround_faces(half_edge);

    // update v2 new position
    auto [position, metrics] = e2->quadric_error_metrics();
    v2->position = position;

    // update vertex half edge(v1, v2, v4)
    v1->half_edge = e1_op;
    v2->half_edge = e5_op;
    v4->half_edge = e6_op;

    // change edge that point to v3 to v2 (there exists a trap, see edge_collapse_with_duplicate.jpg under image directory, if vx->v3 and v2->vx, then after collapse, [vx, v2(origin v3), v4] and [v4, v2, vx] are same face but with different orientation, which is duplicate and should erase one of them(situation like this means it will already caused non manifoldness accur, so we can delete any one of them, here we delete v3, should also take care that before delete this face and edge, you should let opposite half edge's opposite to nullptr first))
    auto lambda_func1 = [&](decltype(v2->half_edge) in_edge, decltype(v2) vertex_point_to) -> bool {
        if (in_edge == e2 || in_edge == e6)
        {
            return false;
        }
        in_edge->vertex = vertex_point_to;
        return true;
    };
    std::function<bool(decltype(v2->half_edge), decltype(v2))> change_point_vertex_func = lambda_func1;
    // NOTE: WHY this is not work?
     //handle_in_edge(v3, lambda_func1, v2);
    v3->handle_in_edge<bool, decltype(v2)>(lambda_func1, v2);
    //handle_in_edge<VData, bool, decltype(v2)>(v3, lambda_func1, v2);

    // change half edge of retained
    e1_op->opposite = e3_op;
    e3_op->opposite = e1_op;
    e5_op->opposite = e6_op;
    e6_op->opposite = e5_op;

    new_faces.erase(std::remove(new_faces.begin(), new_faces.end(), f1), new_faces.end());
    new_faces.erase(std::remove(new_faces.begin(), new_faces.end(), f2), new_faces.end());
    // if debug viewer is on
    if (this->debug_viewer)
    {
        std::vector<DisplayInfo<VData, std::vector<int>>> mul_display_info;
        auto get_display_info = [](std::vector<std::shared_ptr<Vertex<VData>>>& vertices, std::vector<std::shared_ptr<Face<VData>>>& polygons, float r, float g, float b) -> DisplayInfo<VData, std::vector<int>> {
            DisplayInfo<Point3D, std::vector<int>> display_info;
            std::unordered_map<std::shared_ptr<Vertex<VData>>, int> index_cache;
            int count = 0;
            for (const auto& vertex : vertices)
            {
                // file << "v " << vertex->position.x << " " << vertex->position.y << " " << vertex->position.z << std::endl;
                display_info.vertices.emplace_back(vertex->position);
                index_cache.emplace(vertex, count);
                count++;
            }

            for (const auto& face : polygons)
            {
                auto iter = face->half_edge;
                auto head = iter;
                std::vector<int> vec_face;
                do
                {
                    int index = index_cache.at(iter->vertex);
                    vec_face.emplace_back(index);
                    iter = iter->next;
                } while (iter != head);
                display_info.triangles.emplace_back(vec_face);
                display_info.normals.emplace_back(face->normal());
            }
            display_info.r = r;
            display_info.g = g;
            display_info.b = b;
            return display_info;
        };
        std::vector<std::shared_ptr<Face<VData>>> purified_mesh_faces;
        for (auto& face : this->faces)
        {
            if (!excluded_faces.contains(face))
            {
                purified_mesh_faces.emplace_back(face);
            }
        }
        auto mesh_display_info = get_display_info(this->vertices, purified_mesh_faces, 0, 1.0f, 0);

        auto collapsed_vertices = get_vertices(new_faces);
        auto collapsed_display_info = get_display_info(collapsed_vertices, new_faces, 1.0f, 0, 0);

        mul_display_info.emplace_back(mesh_display_info);
        mul_display_info.emplace_back(collapsed_display_info);
        show_triangles_with_model_viewer(mul_display_info);
    }
    // check if mesh has intersection with collapsed edge
    if (this->intersect(new_faces, excluded_faces))
    {
        return true;
    }

    if (this->is_manifold(half_edge, new_faces))
    {
        return false;
    }
    else
    {
        return true;
    }

    return false;
}

/**
 * @brief check if mesh is manifold if those faces that sourround half_edge replaced by faces
 * 
 * @tparam VData 
 * @param half_edge 
 * @param faces 
 * @return true 
 * @return false 
 */
template <typename VData>
bool Mesh<VData>::is_manifold(std::shared_ptr<HalfEdge<VData>> half_edge, std::vector<std::shared_ptr<Face<VData>>> faces)
{
    auto boundary_connected_vertice = this->get_boundary_connected_vertices(half_edge);
    std::unordered_set<std::pair<VData, VData>> new_face_connected_vertice;
    for (auto& new_face : faces)
    {
        auto he_iter = new_face->half_edge;
        auto he_head = he_iter;
        do
        {
            std::pair<VData, VData> search_item;
            search_item.first = he_iter->prev->vertex->position;
            search_item.second = he_iter->vertex->position;
            if (he_iter->opposite == nullptr)
            {
                if (!boundary_connected_vertice.contains(search_item))
                {
                    // cannot found opposite edge in src mesh, so it's not manifold
                    return false;
                }
            }
            new_face_connected_vertice.emplace(search_item);
            he_iter = he_iter->next;
        } while (he_iter != he_head);
    }
    for (auto& boundary_connected_vertex : boundary_connected_vertice)
    {
        if (!new_face_connected_vertice.contains(boundary_connected_vertex))
        {
            // cannot find opposite edge in new face, src mesh will exist one side edge, so it's not manifold
            return false;
        }
    }
    return true;
}

/**
 * @brief check if mesh is manifold by iff edge shared by two faces. In half edge data structure, edge is natually shared by two faces, by check opposite edge is not empty, and two connected vertex is only connected by two half edges.
 * 
 * @tparam VData 
 * @return true 
 * @return false 
 */
template <typename VData>
bool Mesh<VData>::is_manifold()
{
    std::unordered_map<
        std::unordered_set<std::shared_ptr<Vertex<VData>>>,
        int,
        boost::hash<std::unordered_set<std::shared_ptr<Vertex<VData>>>>
    > connected_vertex_cache;
    for (const auto& half_edge : this->half_edges)
    {
        if (half_edge->opposite == nullptr)
        {
            return false;
        }
        std::shared_ptr<Vertex<VData>> start_vertex = half_edge->prev->vertex;
        std::shared_ptr<Vertex<VData>> end_vertex = half_edge->vertex;
        //std::pair<std::shared_ptr<Vertex<VData>>, std::shared_ptr<Vertex<VData>>> connected_vertex;
        std::unordered_set<std::shared_ptr<Vertex<VData>>> connected_vertex{
            start_vertex, end_vertex
        };
        if (connected_vertex_cache.contains(connected_vertex))
        {
            connected_vertex_cache.at(connected_vertex)++;
            if (connected_vertex_cache.at(connected_vertex) > 2)
            {
                return false;
            }
        }
        else
        {
            connected_vertex_cache.emplace(connected_vertex, 1);
        }
    }
    // duplicate face check
    std::unordered_set<
        std::unordered_set<std::shared_ptr<Vertex<VData>>>,
        boost::hash<std::unordered_set<std::shared_ptr<Vertex<VData>>>>
    > face_cache;
    for(const auto& face: this->faces)
    {
        std::unordered_set<std::shared_ptr<Vertex<VData>>> face_vertice;
        auto start_edge = face->half_edge;
        auto iter_edge = start_edge;
        do
        {
            // emplace all vertice to the face_vertice
            face_vertice.emplace(iter_edge->vertex);
            iter_edge = iter_edge->next;
        } while (iter_edge != start_edge);
        if (face_cache.contains(face_vertice))
        {
            return false;
        }
        else
        {
            face_cache.emplace(face_vertice);
        }
    }
    return true;
}

template <typename VData>
std::unordered_set<std::pair<VData, VData>> Mesh<VData>::get_boundary_connected_vertices(std::shared_ptr<HalfEdge<VData>> half_edge)
{
    std::unordered_set<std::shared_ptr<HalfEdge<VData>>> avoided_edges;
    auto get_avoided_edges_lambda = [&](std::shared_ptr<HalfEdge<VData>> in_edge) ->bool {
        avoided_edges.emplace(in_edge);
        avoided_edges.emplace(in_edge->opposite);
        return true;
    };
    std::function<bool(std::shared_ptr<HalfEdge<VData>>)> get_avoided_edges_func = get_avoided_edges_lambda;
    half_edge->prev->vertex->handle_in_edge(get_avoided_edges_func);
    half_edge->vertex->handle_in_edge(get_avoided_edges_func);

    std::unordered_set<std::shared_ptr<HalfEdge<VData>>> opposite_half_edges1 = half_edge->vertex->get_opposite_half_edges();
    std::unordered_set<std::shared_ptr<HalfEdge<VData>>> opposite_half_edges2 = half_edge->prev->vertex->get_opposite_half_edges();

    opposite_half_edges1.merge(opposite_half_edges2);

    std::unordered_set<std::pair<VData, VData>> opposite_connected_vertices;
    for(const auto& elem: opposite_half_edges1)
    {
        if(!avoided_edges.contains(elem))
        {
            VData start_pos = elem->prev->vertex->position;
            VData end_pos = elem->vertex->position;
            opposite_connected_vertices.emplace(std::make_pair(start_pos, end_pos));
        }
    }
    return opposite_connected_vertices;
}

template <typename VData>
std::tuple<
    std::vector<std::shared_ptr<Face<VData>>>,
    std::vector<std::shared_ptr<HalfEdge<VData>>>,
    std::vector<std::shared_ptr<Vertex<VData>>>,
    std::shared_ptr<HalfEdge<VData>>
> Mesh<VData>::copy_faces(std::shared_ptr<HalfEdge<VData>> half_edge)
{
    std::shared_ptr<HalfEdge<VData>> copied_half_edge;
    std::unordered_map<std::shared_ptr<Vertex<VData>>, std::shared_ptr<Vertex<VData>>> old_new_vertex_map;
    std::unordered_set<std::shared_ptr<HalfEdge<VData>>> visited_half_edges;
    auto old_start_vertex = half_edge->prev->vertex;
    auto old_end_vertex = half_edge->vertex;
    auto new_start_vertex = std::make_shared<Vertex<VData>>();
    new_start_vertex->position = old_start_vertex->position;
    auto new_end_vertex = std::make_shared<Vertex<VData>>();
    new_end_vertex->position = old_end_vertex->position;

    old_new_vertex_map.emplace(old_start_vertex, new_start_vertex);
    old_new_vertex_map.emplace(old_end_vertex, new_end_vertex);

    std::vector<std::shared_ptr<Vertex<VData>>> new_vertices;
    new_vertices.emplace_back(new_start_vertex);
    new_vertices.emplace_back(new_end_vertex);
    std::vector<std::shared_ptr<HalfEdge<VData>>> new_half_edges;
    std::vector<std::shared_ptr<Face<VData>>> new_faces;
    auto copy_face = [&](std::shared_ptr<HalfEdge<VData>> in_edge) -> bool {
        if (visited_half_edges.contains(in_edge))
        {
            // already copied
            return true;
        }
        std::vector<std::shared_ptr<HalfEdge<VData>>> temp_new_half_edges;
        auto in_edge_head = in_edge;
        auto in_edge_iter = in_edge;
        std::shared_ptr<Face<VData>> new_face = std::make_shared<Face<VData>>();
        new_faces.emplace_back(new_face);
        bool first_edge = true;
        do
        {
            visited_half_edges.emplace(in_edge_iter);
            // new half edge
            std::shared_ptr<HalfEdge<VData>> new_edge = std::make_shared<HalfEdge<VData>>();
            temp_new_half_edges.emplace_back(new_edge);
            if (in_edge_iter == half_edge)
            {
                copied_half_edge = new_edge;
            }
            if (first_edge)
            {
                first_edge = false;
                new_face->half_edge = new_edge;
            }
            new_edge->face = new_face;
            auto old_vertex = in_edge_iter->vertex;
            if (old_new_vertex_map.contains(old_vertex))
            {
                auto new_vertex = old_new_vertex_map.at(old_vertex);
                new_edge->vertex = new_vertex;
                new_vertex->half_edge = new_edge;
            }
            else
            {
                // new vertex if not exists
                std::shared_ptr<Vertex<VData>> new_vertex = std::make_shared<Vertex<VData>>();
                new_vertex->half_edge = new_edge;
                new_vertex->position = old_vertex->position;
                new_vertices.emplace_back(new_vertex);
                new_edge->vertex = new_vertex;
                old_new_vertex_map.emplace(old_vertex, new_vertex);
            }
            in_edge_iter = in_edge_iter->next;
        } while (in_edge_iter != in_edge_head);

        // finish prev and next chain
        for (int he_index = 0; he_index < temp_new_half_edges.size(); he_index++)
        {
            auto he_first = temp_new_half_edges[he_index % (temp_new_half_edges.size())];
            auto he_second = temp_new_half_edges[(he_index + 1) % (temp_new_half_edges.size())];
            he_first->next = he_second;
            he_second->prev = he_first;
        }
        new_half_edges.insert(new_half_edges.end(), temp_new_half_edges.begin(), temp_new_half_edges.end());
        return true;
    };
    std::function<bool(std::shared_ptr<HalfEdge<VData>>)> copy_face_func = copy_face;

    old_start_vertex->handle_in_edge(copy_face_func);
    old_end_vertex->handle_in_edge(copy_face_func);

    // finish opposite
    for (auto new_he : new_half_edges)
    {
        for (auto other_new_he : new_half_edges)
        {
            if (new_he->vertex == other_new_he->prev->vertex && new_he->prev->vertex == other_new_he->vertex)
            {
                new_he->opposite = other_new_he;
                other_new_he->opposite = new_he;
            }
        }
    }

    return std::make_tuple(new_faces, new_half_edges, new_vertices, copied_half_edge);
}

template <typename VData>
std::unordered_set<std::shared_ptr<Face<VData>>> Mesh<VData>::get_sorround_faces(std::shared_ptr<HalfEdge<VData>> half_edge)
{
    std::unordered_set<std::shared_ptr<Face<VData>>> sorround_faces;
    auto get_excluded_faces = [&](std::shared_ptr<HalfEdge<VData>> in_edge) -> bool {
        sorround_faces.emplace(in_edge->face);
        return true;
    };
    std::function<bool(std::shared_ptr<HalfEdge<VData>>)> get_excluded_faces_func = get_excluded_faces;
    half_edge->vertex->handle_in_edge(get_excluded_faces_func);
    half_edge->prev->vertex->handle_in_edge(get_excluded_faces_func);
    return sorround_faces;
}


};


#endif