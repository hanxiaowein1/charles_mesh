#include "basic_type.h"
#include "unit_test.h"

namespace charles_mesh
{

TEST(GlobalTest, vector3d_dot)
{
    {
        Vector3D vec1{0, 1, 0};
        Vector3D vec2{0, 0, 1};
        auto result = vec1.dot(vec2);
        ASSERT_EQ(result, 0.0f);
    }

    {
        Vector3D vec1{0, 1, 0};
        Vector3D vec2{0, 1, 1};
        auto result = vec1.dot(vec2);
        ASSERT_EQ(result, 1.0f);
    }
}

TEST(GlobalTest, vector3d_cross)
{
    {
        Vector3D vec1{1, 0, 0};
        Vector3D vec2{0, 1, 0};
        auto vec3 = vec1.cross(vec2);
        Vector3D expect_vec{0, 0, 1};
        ASSERT_EQ(vec3, expect_vec);
    }

    {
        Vector3D vec2{1, 0, 0};
        Vector3D vec1{0, 1, 0};
        auto vec3 = vec1.cross(vec2);
        Vector3D expect_vec{0, 0, -1};
        ASSERT_EQ(vec3, expect_vec);
    }
}

TEST(GlobalTest, vector3d_acute_angle)
{
    {
        Vector3D vec1{1, 0, 0};
        Vector3D vec2{1, 1, 0};
        bool is_acute_angle = vec1.acute_angle(vec2);

        ASSERT_EQ(is_acute_angle, true);
    }

    {
        Vector3D vec1{1, 0, 0};
        Vector3D vec2{-1, 1, 0};
        bool is_acute_angle = vec1.acute_angle(vec2);

        ASSERT_EQ(is_acute_angle, false);
    }
}

TEST(GlobalTest, vector3d_operator_minus)
{
    Vector3D vec1{1, 2, 3};
    Vector3D vec2{2, 3, 4};
    auto result = vec1 - vec2;
    Vector3D expect_result{-1, -1, -1};
    ASSERT_EQ(result, expect_result);
}

TEST(GlobalTest, vector3d_operator_index)
{
    {
        Vector3D vec1{1, 2, 3};
        ASSERT_EQ(1, vec1[0]);
        ASSERT_EQ(2, vec1[1]);
        ASSERT_EQ(3, vec1[2]);
    }

    {
        Vector3D vec1{1, 2, 3};
        ASSERT_EQ(1, vec1[0]);
        vec1[0] = 4;
        ASSERT_EQ(4, vec1[0]);
    }
}

TEST(GlobalTest, vector3d_between)
{
    {
        Vector3D vec1{0, 1, 0};
        Vector3D vec2{0, 2, 0};
        Vector3D vec3{0, 3, 0};
        ASSERT_EQ(true, vec2.between(vec1, vec3));
    }

    {
        Vector3D vec1{0, 1, 0};
        Vector3D vec2{0, 2, 0};
        Vector3D vec3{0, 3, 0};
        ASSERT_EQ(false, vec3.between(vec1, vec2));
    }
}

TEST(GlobalTest, vector3d_operator_eq)
{
    {
        Vector3D vec1{1, 0, 1};
        Vector3D vec2{1, 0, 1};
        ASSERT_EQ(vec1, vec2);
    }
    {
        Vector3D vec1{1, 0, 1};
        Vector3D vec2{1, 2, 1};
        ASSERT_NE(vec1, vec2);
    }
}

};

