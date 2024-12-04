#ifndef __CHARLES_MESH_TYPE_H__
#define __CHARLES_MESH_TYPE_H__

#include <memory>
#include <stdexcept>
#include "simple_math.h"

namespace charles_mesh
{
class Point3D
{
public:
    double x, y, z;
    double dot(const Point3D& point) const
    {
        return this->x * point.x + this->y * point.y + this->z * point.z;
    }
    Point3D cross(const Point3D& point) const
    {
        Point3D result;
        result.x = this->y * point.z - this->z * point.y;
        result.y = this->z * point.x - this->x * point.z;
        result.z = this->x * point.y - this->y * point.x;
        return result;
    }
    bool acute_angle(const Point3D& point)
    {
        return this->dot(point) > 0;
    }
    Point3D operator-(const Point3D& point) const
    {
        Point3D result;
        result.x = this->x - point.x;
        result.y = this->y - point.y;
        result.z = this->z - point.z;
        return result;
    }
    double& operator[](int index)
    {
        switch (index)
        {
        case 0:
            return this->x;
            break;
        case 1:
            return this->y;
            break;
        case 2:
            return this->z;
            break;
        default:
            throw std::out_of_range("Index out of range");
        }
    }
    bool between(const Point3D& point1, const Point3D& point2)
    {
        if(!(charles_math::in_interval(this->x, point1.x, point2.x)))
        {
            return false;
        }
        if(!(charles_math::in_interval(this->y, point1.y, point2.y)))
        {
            return false;
        }
        if(!(charles_math::in_interval(this->z, point1.z, point2.z)))
        {
            return false;
        }
        return true;
    }
    bool operator==(const Point3D& point) const
    {
        if(!(this->x == point.x))
        {
            return false;
        }
        if(!(this->y == point.y))
        {
            return false;
        }
        if(!(this->z == point.z))
        {
            return false;
        }
        return true;
    }
};

typedef Point3D Vector3D;

class Object
{
public:
    virtual double get_min_x() = 0;
    virtual double get_max_x() = 0;
    virtual double get_min_y() = 0;
    virtual double get_max_y() = 0;
    virtual double get_min_z() = 0;
    virtual double get_max_z() = 0;
    virtual bool intersect(std::shared_ptr<Object> object) = 0;
    virtual bool point_inside(const Point3D& point) = 0;
};

};

#endif