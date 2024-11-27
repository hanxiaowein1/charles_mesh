#include "unit_test.h"

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::GTEST_FLAG(filter) = "GlobalTest.construct_mesh_with_vertex_polygon";
    int result = RUN_ALL_TESTS();
    return result;
}