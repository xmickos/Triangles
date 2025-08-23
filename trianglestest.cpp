#include<iostream>
#include"include/triangles.hpp"
#include<gtest/gtest.h>
#include<numbers>

using namespace hw3d;

TEST(UnitTests, DISABLED_relative_locations) {
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

TEST(UnitTests, DISABLED_is_degenerate) {
    Triangle T1(Vector3d(0, 0, 0), Vector3d(0, 2, 0), Vector3d(0, 4, 0));
    EXPECT_TRUE(T1.is_degenerate());

    Triangle T2(Vector3d(0, 0, 0), Vector3d(0, 2, 1), Vector3d(0, 4, 0));
    EXPECT_FALSE(T2.is_degenerate());

}

TEST(UnitTests, DISABLED_Intersection2d) {
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

TEST(UnitTests, DISABLED_AABB) {
    Triangle T1(Vector3d(9.21269, 0.510527, 3.78626), Vector3d(8.08306, 3.27682, 0.730073), Vector3d(8.42607, 1.32866, 0.572925));
    Triangle T2(Vector3d(7.16461, 1.96139, 2.48755), Vector3d(6.68718, 4.28441, 0.892826), Vector3d(6.62557, 1.08196, 0.75515));

    std::cout << "T1 & T2: " << intersection_test_3d(T1, T2) << std::endl;
}

TEST(UnitTests, Vector3dInplaceRotation) {
    Vector3d axis(-0.332861, -0.523709, -0.784176);
    axis.normalize_inplace();
    Vector3d vec(8.72104, 1.23751, 4.94293);
    Vector3d vec_rotated_expected(8.692969795954639, 4.189590689466975, 2.983309009799692);
    Vector3d vec_rotated = vec.rotate_around_origin(axis, std::numbers::pi / 6);
    for(int i = 0; i < 3; ++i) {
        EXPECT_NEAR(vec_rotated[i], vec_rotated_expected[i], 1e-3);
    }
}

TEST(UnitTests, TriangleRotation) {
    Triangle t1(Vector3d(8.72104, 1.23751, 4.94293), Vector3d(6.85655, 2.34514, 1.70897), Vector3d(3.43423, 3.3827, 7.51132));
    Triangle t2(Vector3d(8.960368405157547, 2.1976996545899965, 4.826846902216941), Vector3d(6.617221594842453, 1.3849503454100034, 1.8250530977830586), Vector3d(3.43423, 3.3827, 7.51132));
    Vector3d t1_median = ((t1[0] + t1[1]) / 2 - t1[2]).normalized();
    Triangle t3 = t1.rotate(t1[2], t1_median, std::numbers::pi / 6);

    std::vector<Vector3d> t1_rotated_one_by_one;
    for(auto it1 = t1.cbegin(), et1 = t1.cend(), it2 = t2.cbegin(); it1 != et1; ++it1) {
        t1_rotated_one_by_one.push_back(it1->rotate(t1[2], t1_median, std::numbers::pi / 6));
    }

    for(
        auto t1_rotated_one_by_one_it = t1_rotated_one_by_one.cbegin(), it3 = t3.cbegin(), t1_rotated_one_by_one_et = t1_rotated_one_by_one.cend();
        t1_rotated_one_by_one_it != t1_rotated_one_by_one_et;
        ++t1_rotated_one_by_one_it, ++it3
    ) {
        for(int i = 0; i < 3; ++i) {
            EXPECT_NEAR((*t1_rotated_one_by_one_it)[i], (*it3)[i], 1e-3);
        }
    }

    for(auto it2 = t2.cbegin(), it3 = t3.cbegin(), et2 = t2.cend(); it2 != et2; ++it2, ++it3) {
        for(int i = 0; i < 3; ++i) {
            EXPECT_NEAR((*it2)[i], (*it3)[i], 1e-3);
        }
    }

}

TEST(UnitTests, DISABLED_Vector3dBasics) {
    Vector3d a(2, 2, 2);
    Vector3d b = a / 2;
    Vector3d c(1, 1, 1);
    Vector3d d(3, 3, 3);
    Vector3d e = a + c;
    Vector3d f = a - c;
    for(int i = 0; i < 3; ++i) {
        EXPECT_NEAR(b[i], c[i], 1e-3);
    }
    for(int i = 0; i < 3; ++i) {
        EXPECT_NEAR(e[i], d[i], 1e-3);
    }
    for(int i = 0; i < 3; ++i) {
        EXPECT_NEAR(f[i], c[i], 1e-3);
    }
}

TEST(End2End, DISABLED_Intersection3d) {
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
