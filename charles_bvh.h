#ifndef __CHARLES_BVH_H__
#define __CHARLES_BVH_H__

#include <algorithm>
#include <memory>
#include <vector>
#include <limits>

#include "simple_math.h"
#include "mesh_type.h"

namespace charles_mesh
{

class BoundingBox
{
public:
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;

    BoundingBox() : min_x(std::numeric_limits<double>::max()), max_x(std::numeric_limits<double>::min()), min_y(std::numeric_limits<double>::max()), max_y(std::numeric_limits<double>::min()), min_z(std::numeric_limits<double>::max()), max_z(std::numeric_limits<double>::min()) {}

    BoundingBox(std::shared_ptr<Object> object)
    {
        this->min_x = object->get_min_x();
        this->min_y = object->get_min_y();
        this->min_z = object->get_min_z();
        this->max_x = object->get_max_x();
        this->max_y = object->get_max_y();
        this->max_z = object->get_max_z();
    }

    // Expand the bounding box to include another point
    void expand(const double& x, const double& y, const double& z)
    {
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
    }

    void expand(std::shared_ptr<Object> object)
    {
        this->min_x = std::min(this->min_x, object->get_min_x());
        this->max_x = std::max(this->max_x, object->get_max_x());
        this->min_y = std::min(this->min_y, object->get_min_y());
        this->max_y = std::max(this->max_y, object->get_max_y());
        this->min_z = std::min(this->min_z, object->get_min_z());
        this->max_z = std::max(this->max_z, object->get_max_z());
    }

    bool intersect(std::shared_ptr<Object> object)
    {
        BoundingBox bounding_box(object);
        if(this->intersect(bounding_box))
        {
            return true;
        }
        return false;
    }

    bool intersect(const BoundingBox& box)
    {
        if(charles_math::intersect<double>(this->min_x, this->max_x, box.min_x, box.max_x))
        {
            return true;
        }
        if(charles_math::intersect<double>(this->min_y, this->max_y, box.min_y, box.max_y))
        {
            return true;
        }
        if(charles_math::intersect<double>(this->min_z, this->max_z, box.min_z, box.max_z))
        {
            return true;
        }
        return false;
    }
};


class BVHNode
{
public:
    BoundingBox box;
    std::shared_ptr<BVHNode> left;
    std::shared_ptr<BVHNode> right;
    bool is_leaf;
    // store real object pointer
    std::shared_ptr<Object> object;

    BVHNode() : left(nullptr), right(nullptr), is_leaf(true), object(nullptr)
    {

    }
};

template<typename T>
int max_index(const std::vector<T>& vec)
{
    auto it = std::max_element(vec.begin(), vec.end());
    int index = std::distance(vec.begin(), it);
    return index;
}

bool bvh_intersect(std::shared_ptr<BVHNode> node, std::shared_ptr<Object> object);
std::shared_ptr<BVHNode> build_bvh(std::vector<std::shared_ptr<Object>>& objects, int start, int end);

};



#endif