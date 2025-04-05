#include<iostream>
#include"include/triangles.hpp"
#include<gtest/gtest.h>

using namespace hw3d;

TEST(UnitTests, relative_locations) {
    Triangle T1(Vector3d(-1, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 0, 0));
    Triangle T2(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(2, 0, 0));
    Triangle T3(Vector3d(-2, 0, 1), Vector3d(0, 2, 1), Vector3d(2, 0, 1));
    Triangle T4(Vector3d(-1, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 0, -1));

    EXPECT_TRUE(T1.lies_on_the_same_plane_with(T2));
    EXPECT_FALSE(T1.lies_on_the_same_plane_with(T3));
    EXPECT_TRUE(T1.lies_on_parallel_planes_with(T3));
    EXPECT_FALSE(T4.lies_on_parallel_planes_with(T1));
    EXPECT_FALSE(T4.lies_on_the_same_plane_with(T1));
}

TEST(UnitTests, is_degenerate) {
    Triangle T1(Vector3d(0, 0, 0), Vector3d(0, 2, 0), Vector3d(0, 4, 0));
    EXPECT_TRUE(T1.is_degenerate());

    Triangle T2(Vector3d(0, 0, 0), Vector3d(0, 2, 1), Vector3d(0, 4, 0));
    EXPECT_FALSE(T2.is_degenerate());

}

TEST(UnitTests, Intersection2d) {
    Triangle T1(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289,2.21191,0));
    Triangle T2(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(2, 0, 0));

    EXPECT_TRUE(intersection_test_2d(T1, T2));
    EXPECT_TRUE(intersection_test_2d(T2, T1));

    Triangle T3(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289,2.21191,0));
    Triangle T4(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(1, 0, 0));

    EXPECT_TRUE(intersection_test_2d(T3, T4));
    EXPECT_TRUE(intersection_test_2d(T4, T3));

    Triangle T5(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289, 2.21191, 0));
    Triangle T6(Vector3d(-2, 0, 0), Vector3d(3.48127, 2.09539, 0), Vector3d(1, 0, 0));

    EXPECT_TRUE(intersection_test_2d(T5, T6));
    EXPECT_TRUE(intersection_test_2d(T6, T5));

    Triangle T7(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289, 2.21191, 0));
    Triangle T8(Vector3d(0.80605, 1.76459, 0), Vector3d(2.0703, 1.36268, 0), Vector3d(1.76179, 0.60273, 0));

    EXPECT_TRUE(intersection_test_2d(T7, T8));
    EXPECT_TRUE(intersection_test_2d(T8, T7));

    Triangle T9(Vector3d(0, 0, 0), Vector3d(2, 0, 0), Vector3d(4, 0, 0));
    Triangle T10(Vector3d(0.80605, 1.76459, 0), Vector3d(2.0703, 1.36268, 0), Vector3d(1.76179, 0.60273, 0));

    EXPECT_FALSE(intersection_test_2d(T9, T10));
    EXPECT_FALSE(intersection_test_2d(T10, T9));
}

TEST(End2End, Intersection3d) {
    Triangle T1(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289,2.21191,0));
    Triangle T2(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(2, 0, 0));

    EXPECT_TRUE(intersection_test_3d(T1, T2));
    EXPECT_TRUE(intersection_test_3d(T2, T1));

    Triangle T3(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289,2.21191,0));
    Triangle T4(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(1, 0, 0));

    EXPECT_TRUE(intersection_test_3d(T3, T4));
    EXPECT_TRUE(intersection_test_3d(T4, T3));

    Triangle T5(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289, 2.21191, 0));
    Triangle T6(Vector3d(-2, 0, 0), Vector3d(3.48127, 2.09539, 0), Vector3d(1, 0, 0));

    EXPECT_TRUE(intersection_test_3d(T5, T6));
    EXPECT_TRUE(intersection_test_3d(T6, T5));

    Triangle T7(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289, 2.21191, 0));
    Triangle T8(Vector3d(0.80605, 1.76459, 0), Vector3d(2.0703, 1.36268, 0), Vector3d(1.76179, 0.60273, 0));

    EXPECT_TRUE(intersection_test_3d(T7, T8));
    EXPECT_TRUE(intersection_test_3d(T8, T7));

    Triangle T9(Vector3d(0, 0, 0), Vector3d(2, 0, 0), Vector3d(4, 0, 0));
    Triangle T10(Vector3d(0.80605, 1.76459, 0), Vector3d(2.0703, 1.36268, 0), Vector3d(1.76179, 0.60273, 0));

    EXPECT_FALSE(intersection_test_3d(T9, T10));
    EXPECT_FALSE(intersection_test_3d(T10, T9));

    Triangle T11(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(2, 0, 0));
    Triangle T12(Vector3d(-2, 0, 1), Vector3d(0, 2, 1), Vector3d(2, 0, 1));

    EXPECT_FALSE(intersection_test_3d(T11, T12));
    EXPECT_FALSE(intersection_test_3d(T12, T11));

    Triangle T13(Vector3d(0, 0, 0), Vector3d(2, 0, 1.22359), Vector3d(4, 0, 0));
    Triangle T14(Vector3d(1.80814, -1.02431, 0), Vector3d(0.80605, 1.76459, 0), Vector3d(1.28348, 0.54633, -1.85131));

    EXPECT_TRUE(intersection_test_3d(T13, T14));
    EXPECT_TRUE(intersection_test_3d(T14, T13));

    Triangle T15(Vector3d(0, 0, 0), Vector3d(2, 0, 1.22359), Vector3d(4, 0, 0));
    Triangle T16(Vector3d(1.80814, -1.02431, 0.59548), Vector3d(0.80605, 1.76459, 0.5727), Vector3d(1.28348, 0.54633, -1.85131));

    EXPECT_TRUE(intersection_test_3d(T15, T16));
    EXPECT_TRUE(intersection_test_3d(T16, T15));

    Triangle T17(Vector3d(0, 0, 0), Vector3d(2, 0, 1.22359), Vector3d(4, 0, 0));
    Triangle T18(Vector3d(1.80814, -1.02431, -0.55653), Vector3d(0.80605, 1.76459, -0.5727), Vector3d(1.28348, 0.54633, -1.85131));

    EXPECT_TRUE(intersection_test_3d(T17, T18));
    EXPECT_TRUE(intersection_test_3d(T18, T17));

    Triangle T19(Vector3d(0, 0, 0), Vector3d(2, 0, 0), Vector3d(4, 0, 0));
    Triangle T20(Vector3d(1.80814, -1.02431, 4), Vector3d(0.80605, 1.76459, 4), Vector3d(1.30151, 0.31348, -1.85131));

    EXPECT_FALSE(intersection_test_3d(T19, T20));
    EXPECT_FALSE(intersection_test_3d(T20, T19));
}
