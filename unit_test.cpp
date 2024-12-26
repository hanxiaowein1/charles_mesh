#include "unit_test.h"

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::GTEST_FLAG(filter) = "GlobalTest.draw_small_bunny";
    // ::testing::GTEST_FLAG(filter) = "GlobalTest.line_segment_intersection";
     //::testing::GTEST_FLAG(filter) = "GlobalTest.mesh_copy_faces";
     //::testing::GTEST_FLAG(filter) = "GlobalTest.edge_collapse_intersection_detect";
    //::testing::GTEST_FLAG(filter) = "GlobalTest.mesh_copy_faces";

    int result = RUN_ALL_TESTS();
    return result;
}