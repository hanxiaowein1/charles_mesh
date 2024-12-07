#include "unit_test.h"

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::GTEST_FLAG(filter) = "GlobalTest.mesh_edge_flip_with_intersection_detect";
    int result = RUN_ALL_TESTS();
    return result;
}