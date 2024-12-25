#ifndef __CHARLES_BASIC_TYPE_H__
#define __CHARLES_BASIC_TYPE_H__

#include <memory>
#include <stdexcept>
#include <cmath>
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
    Point3D operator+(const Point3D& point) const
    {
        Point3D result;
        result.x = this->x + point.x;
        result.y = this->y + point.y;
        result.z = this->z + point.z;
        return result;
    }
    Point3D operator*(double value) const
    {
        Point3D result;
        result.x = this->x * value;
        result.y = this->y * value;
        result.z = this->z * value;
        return result;
    }
    Point3D operator/(double value) const
    {
        if(value == 0.0f)
        {
            throw std::exception("value cannot be zero!");
        }
        Point3D result;
        result.x = this->x / value;
        result.y = this->y / value;
        result.z = this-> z / value;
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
    double len() const
    {
        return std::pow(std::pow(this->x, 2) + std::pow(this->y, 2) + std::pow(this->z, 2), 0.5f);
    }
    void normalize()
    {
        auto length = this->len();
        this->x = this->x / length;
        this->y = this->y / length;
        this->z = this->z / length;
    }
};

class Point3DHashFunction
{
public:
    size_t operator()(const Point3D& point) const
    {
        size_t x_hash = std::hash<double>()(point.x);
        size_t y_hash = std::hash<double>()(point.y) << 1;
        size_t z_hash = std::hash<double>()(point.z) << 2;
        return x_hash ^ y_hash ^ z_hash;
    }
};

typedef Point3D Vector3D;

template<typename VData>
VData center(const VData& point1, const VData& point2)
{
    VData ret;
    ret.x = (point1.x + point2.x) / 2.0f;
    ret.y = (point1.y + point2.y) / 2.0f;
    ret.z = (point1.z + point2.z) / 2.0f;
    return ret;
}

template<typename VData=Point3D>
class Object
{
public:
    virtual double get_min_x() = 0;
    virtual double get_max_x() = 0;
    virtual double get_min_y() = 0;
    virtual double get_max_y() = 0;
    virtual double get_min_z() = 0;
    virtual double get_max_z() = 0;
    virtual bool intersect(std::shared_ptr<Object<VData>> object) = 0;
    virtual bool point_inside(const VData& point) = 0;
};

};

#endif