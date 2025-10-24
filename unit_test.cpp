#include "unit_test.h"

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::GTEST_FLAG(filter) = "GlobalTest.edge_collapse_small_bunny";
    // ::testing::GTEST_FLAG(filter) = "GlobalTest.mc33_bunny_test";
    // ::testing::GTEST_FLAG(filter) = "GlobalTest.draw_small_bunny";
    // ::testing::GTEST_FLAG(filter) = "GlobalTest.edge_collapse_small_bunny";
    // ::testing::GTEST_FLAG(filter) = "GlobalTest.line_segment_intersection";
     //::testing::GTEST_FLAG(filter) = "GlobalTest.mesh_copy_faces";
     //::testing::GTEST_FLAG(filter) = "GlobalTest.edge_collapse_intersection_detect";
    //::testing::GTEST_FLAG(filter) = "GlobalTest.mesh_copy_faces";
    // ::testing::GTEST_FLAG(filter) = "GlobalTest.vertex_get_opposite_connected_vertices";
    //::testing::GTEST_FLAG(filter) = "GlobalTest.is_manifold_after_collapse";
    //::testing::GTEST_FLAG(filter) = "GlobalTest.mesh_get_boundary_connected_vertices";
    
    int result = RUN_ALL_TESTS();
    return result;
}