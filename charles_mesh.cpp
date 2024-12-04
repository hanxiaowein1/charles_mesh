#include <string>
#include <filesystem>
#include <limits>

#include <Eigen/Dense>

#include "charles_mesh.h"
#include "unit_test.h"
#include "charles_bvh.h"

namespace charles_mesh
{

Vector3D Face::normal()
{
    // use first three point to calculate normal
    Vector3D face_normal;
    auto iter = this->half_edge;
    std::vector<Point3D> points;
    for(int i = 0; i < 3; i++)
    {
        points.emplace_back(iter->vertex->position);
        iter = iter->next;
    }
    Vector3D vec1 = points[0] - points[1];
    Vector3D vec2 = points[1] - points[2];
    return vec1.cross(vec2);
}

std::vector<double> Face::plane()
{
    auto normal = this->normal();
    std::vector<double> equation(4);
    Point3D point = this->half_edge->vertex->position;
    equation[0] = normal[0];
    equation[1] = normal[1];
    equation[2] = normal[2];
    equation[3] = -(normal[0] * point.x + normal[1] * point.y + normal[2] * point.z);
    return equation;
}

bool Face::point_inside(const Point3D& point)
{
    // check if point is the left/right of all edges
    auto iter = this->half_edge;
    auto head = iter;
    Vector3D first_dir;
    do
    {
        const auto& v0 = iter->prev->vertex->position;
        const auto& v1 = iter->vertex->position;
        Vector3D edge = v1 - v0;
        Vector3D inner_edge = point - v0;
        if(iter == head)
        {
            // initialize first_dir and then do nothing
            first_dir = edge.cross(inner_edge);
        }
        else
        {
            Vector3D current_dir = edge.cross(inner_edge);
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

bool Face::intersect(const Point3D& point1, const Point3D& point2, Point3D& intersect_p)
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
bool Face::intersect(std::shared_ptr<Object> object)
{
    auto face = std::dynamic_pointer_cast<Face>(object);
    // test this and face intersect
    // if has an edge with intersection of other face, then it's intersect, otherwise not intersect
    Point3D intersect_p;
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

double Face::get_min_x()
{
    if(!this->is_valid)
    {
        this->update_bounding();
    }
    return this->min_x;
}

double Face::get_max_x()
{
    if(!this->is_valid)
    {
        this->update_bounding();
    }
    return this->max_x;
}

double Face::get_min_y()
{
    if(!this->is_valid)
    {
        this->update_bounding();
    }
    return this->min_y;
}

double Face::get_max_y()
{
    if(!this->is_valid)
    {
        this->update_bounding();
    }
    return this->max_y;
}

double Face::get_min_z()
{
    if(!this->is_valid)
    {
        this->update_bounding();
    }
    return this->min_z;
}

double Face::get_max_z()
{
    if(!this->is_valid)
    {
        this->update_bounding();
    }
    return this->max_z;
}

void Face::update_bounding()
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
        bool face_first_edge = true;
        std::vector<std::shared_ptr<HalfEdge>> temp_half_edges;
        for(const auto& edge: polygon)
        {
            std::shared_ptr<HalfEdge> he(new HalfEdge());
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

/**
 * @brief if mesh is intersect with polygon
 * 
 * @param polygon 
 * @return true 
 * @return false 
 */
// bool Mesh::intersect(const std::vector<int>& polygon)
bool Mesh::intersect(std::shared_ptr<Face> polygon)
{
    std::vector<std::shared_ptr<Object>> objects;
    for(auto face: this->faces)
    {
        objects.emplace_back(std::dynamic_pointer_cast<Object>(face));
    }
    std::shared_ptr<BVHNode> bvh_tree = build_bvh(objects, 0, objects.size());
    std::shared_ptr<Object> object = std::dynamic_pointer_cast<Object>(polygon);
    if(bvh_intersect(bvh_tree, object))
    {
        return true;
    }
    return false;
}


};
