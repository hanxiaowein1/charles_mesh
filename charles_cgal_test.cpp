#include "charles_cgal.h"
#include "unit_test.h"
#include "basic_type.h"

TEST(GlobalTest, line_segment_intersection)
{
    {
        charles_mesh::Point3D a{ 0, 0, 0 };
        charles_mesh::Point3D b{ 1, 1, 1 };
        charles_mesh::Point3D c{ 0.5f, 0.5f, 0.5f };
        charles_mesh::Point3D d{ 2, 2, 2 };
        bool is_intersect = line_segments_intersection(a, b, c, d);
        ASSERT_EQ(true, is_intersect);
    }
    {
        charles_mesh::Point3D a{ 0, 0, 0 };
        charles_mesh::Point3D b{ 1, 1, 0 };
        charles_mesh::Point3D c{ 1, 0, 0 };
        charles_mesh::Point3D d{ 0, 1, 0 };
        bool is_intersect = line_segments_intersection(a, b, c, d);
        ASSERT_EQ(true, is_intersect);
    }
    {
        charles_mesh::Point3D a{ 0, 0, 0 };
        charles_mesh::Point3D b{ 0, 0, 1 };
        charles_mesh::Point3D c{ 0, 0, 0.5f };
        charles_mesh::Point3D d{ 0, 0, 2 };
        bool is_intersect = line_segments_intersection(a, b, c, d);
        ASSERT_EQ(true, is_intersect);
    }
    {
        charles_mesh::Point3D a{ 0, 0, 0 };
        charles_mesh::Point3D b{ 1, 1, 0 };
        charles_mesh::Point3D c{ 1, 0, 0 };
        charles_mesh::Point3D d{ 0.9, 0.1, 0 };
        bool is_intersect = line_segments_intersection(a, b, c, d);
        ASSERT_EQ(false, is_intersect);
    }
}