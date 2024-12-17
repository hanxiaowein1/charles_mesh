#include "unit_test.h"

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // ::testing::GTEST_FLAG(filter) = "GlobalTest.edge_collapse";
    // ::testing::GTEST_FLAG(filter) = "GlobalTest.line_segment_intersection";
    int result = RUN_ALL_TESTS();
    return result;
}