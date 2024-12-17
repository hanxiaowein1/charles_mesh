#ifndef __CHARLES_CGAL_H__
#define __CHARLES_CGAL_H__

#include <symengine/expression.h>
#include <symengine/solve.h>
#include <symengine/basic.h>
#include <symengine/add.h>
#include <symengine/matrix.h>
#include <symengine/integer.h>

#include "simple_math.h"
#include "charles_symengine_common.h"

template <typename Point>
bool line_segments_same_plane(const Point& p1, const Point& p2, const Point& q1, const Point& q2)
{
    auto u = p2 - p1;
    auto v = q2 - q1;
    auto w = q1 - p1;
    if (u.dot(v.cross(w)) == 0)
    {
        return true;
    }
    return false;
}

template <typename Point>
bool same_line(const Point& p1, const Point& p2, const Point& p3)
{
    auto ratio_x = (p3.x - p1.x) / (p2.x - p1.x);
    auto ratio_y = (p3.y - p1.y) / (p2.y - p1.y);
    auto ratio_z = (p3.z - p1.z) / (p2.z - p1.z);
    if (ratio_x == ratio_y && ratio_x == ratio_z)
    {
        return true;
    }
    return false;
}

/**
 * @brief check if [a, b] has intersection with [c, d]
 * 
 * @tparam Point 
 * @param a 
 * @param b 
 * @param c 
 * @param d 
 * @return true 
 * @return false 
 */
template <typename Point>
bool line_segments_intersection(const Point& a, const Point& b, const Point& c, const Point& d)
{
    if (a == b || c == d)
    {
        throw std::exception("line segments intersection: please don't pass same point!");
    }
    // if not in same plane, then two line segments won't has intersection
    if (!line_segments_same_plane(a, b, c, d))
    {
        return false;
    }
    // if line segment is vertical to (x,y) plane
    if((b - a).x == 0  && (b - a).y == 0 && (d - c).x == 0 && (d - c).y == 0)
    {
        // must be same line vertical to (x,y) plane can has intersection
        if (b.x != c.x)
        {
            return false;
        }
        if (b.y != c.y)
        {
            return false;
        }
        if(!charles_math::intersect(a.z, b.z, c.z, d.z))
        {
            return false;
        }
        return true;
    }
    // project to (x,y) plane
    // if one same line
    if (same_line(a, b, c) && same_line(a, b, d))
    {
        if (!charles_math::intersect(a.z, b.z, c.z, d.z))
        {
            return false;
        }
        return true;
    }
    // line segment1: a + t * b;
    // line segment2: c + s * b;
    // compute solution by substitute point (a, b, c, d), with limitation 0 <= t <= 1, and 0 <= s <= 1
    auto t = SymEngine::symbol("t");
    auto s = SymEngine::symbol("s");

    SymEngine::Expression t_(t);
    SymEngine::Expression s_(s);

    SymEngine::Expression ex1;
    SymEngine::Expression ex2;

    ex1 = ex1 + a.x + t_ * (b.x - a.x) - (c.x + s_ * (d.x - c.x));
    ex2 = ex2 + a.y + t_ * (b.y - a.y) - (c.y + s_ * (d.y - c.y));
    auto solutions = SymEngine::linsolve({ex1.get_basic(), ex2.get_basic()}, {t, s});
    double solution_t = get_double_from_solution(solutions[0]);
    double solution_s = get_double_from_solution(solutions[1]);

    if(solution_t < 0 || solution_t > 1)
    {
        return false;
    }

    if(solution_s < 0 || solution_s > 1)
    {
        return false;
    }

    return true;
}

#endif