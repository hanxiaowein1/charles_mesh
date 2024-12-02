#include "charles_bvh.h"
#include <format>
namespace charles_mesh
{

std::shared_ptr<BVHNode> build_bvh(std::vector<std::shared_ptr<Object>>& objects, int start, int end)
{
    std::shared_ptr<BVHNode> node(new BVHNode());

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

    std::sort(objects.begin() + start, objects.begin() + end, [axis](std::shared_ptr<Object> a, std::shared_ptr<Object> b) {
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

bool bvh_intersect(std::shared_ptr<BVHNode> node, std::shared_ptr<Object> object)
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
    return intersect(node->left, object) || intersect(node->right, object);
}

};
