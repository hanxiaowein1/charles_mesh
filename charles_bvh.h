#ifndef __CHARLES_BVH_H__
#define __CHARLES_BVH_H__

#include <algorithm>
#include <memory>
#include <vector>
#include <limits>
#include <format>

#include "simple_math.h"
#include "basic_type.h"

namespace charles_mesh
{

template<typename OBJECT=Object<Point3D>>
class BoundingBox
{
public:
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;

    BoundingBox() : min_x(std::numeric_limits<double>::max()), max_x(std::numeric_limits<double>::min()), min_y(std::numeric_limits<double>::max()), max_y(std::numeric_limits<double>::min()), min_z(std::numeric_limits<double>::max()), max_z(std::numeric_limits<double>::min()) {}

    BoundingBox(std::shared_ptr<OBJECT> object)
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

    void expand(std::shared_ptr<OBJECT> object)
    {
        this->min_x = std::min(this->min_x, object->get_min_x());
        this->max_x = std::max(this->max_x, object->get_max_x());
        this->min_y = std::min(this->min_y, object->get_min_y());
        this->max_y = std::max(this->max_y, object->get_max_y());
        this->min_z = std::min(this->min_z, object->get_min_z());
        this->max_z = std::max(this->max_z, object->get_max_z());
    }

    bool intersect(std::shared_ptr<OBJECT> object)
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


template<typename OBJECT=Object<Point3D>>
class BVHNode
{
public:
    BoundingBox<OBJECT> box;
    std::shared_ptr<BVHNode<OBJECT>> left;
    std::shared_ptr<BVHNode<OBJECT>> right;
    bool is_leaf;
    // store real object pointer
    std::shared_ptr<OBJECT> object;

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

template<typename OBJECT>
bool bvh_intersect(std::shared_ptr<BVHNode<OBJECT>> node, std::shared_ptr<OBJECT> object)
{
    if(node == nullptr)
    {
        return false;
    }
    if(!node->box.intersect(object))
    {
        return false;
    }
    if(node->left == nullptr && node->right == nullptr)
    {
        // check leaf node intersection
        if(object->intersect(node->object))
        {
            return true;
        }
        return false;
    }
    return bvh_intersect(node->left, object) || bvh_intersect(node->right, object);
}

template<typename OBJECT>
std::shared_ptr<BVHNode<OBJECT>> build_bvh(std::vector<std::shared_ptr<OBJECT>>& objects, int start, int end)
{
    std::shared_ptr<BVHNode<OBJECT>> node(new BVHNode<OBJECT>());

    // Calculate the bounding box for the current set of objects
    for (int i = start; i < end; ++i)
    {
        node->box.expand(objects[i]);
    }

    // If there's only one box, this is a leaf node
    if (end - start == 1)
    {
        node->is_leaf = true;
        node->object = objects[start];
        return node;
    }

    // Sort the boxes along the longest axis to split them
    double extent_x = node->box.max_x - node->box.min_x;
    double extent_y = node->box.max_y - node->box.min_y;
    double extent_z = node->box.max_z - node->box.min_z;

    int axis = max_index(std::vector<double>{extent_x, extent_y, extent_z});

    std::sort(objects.begin() + start, objects.begin() + end, [axis](std::shared_ptr<OBJECT> a, std::shared_ptr<OBJECT> b) {
        if(axis == 0)
        {
            return (a->get_min_x() + a->get_max_x()) < (b->get_min_x() + b->get_max_x());
        }
        else if(axis == 1)
        {
            return (a->get_min_y() + a->get_max_y()) < (b->get_min_y() + b->get_max_y());
        }
        else if(axis == 2)
        {
            return (a->get_min_z() + a->get_max_z()) < (b->get_min_z() + b->get_max_z());
        }
        else
        {
            throw std::exception(std::format("axis {} should in [0, 1, 2]", axis).c_str());
        }
    });

    // Split the boxes into two halves and recursively build the left and right subtrees
    int mid = start + (end - start) / 2;
    node->left = build_bvh(objects, start, mid);
    node->right = build_bvh(objects, mid, end);
    node->is_leaf = false;

    return node;
}


};



#endif