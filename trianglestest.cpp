#include"include/generate_tests.hpp"
#include"include/triangles.hpp"
#include<iostream>
#include<gtest/gtest.h>
#include<numbers>


using namespace hw3d;

TEST(UnitTests, relative_locations) {
    Triangle t1(Vector3d(-1, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 0, 0));
    Triangle t2(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(2, 0, 0));
    Triangle t3(Vector3d(-2, 0, 1), Vector3d(0, 2, 1), Vector3d(2, 0, 1));
    Triangle t4(Vector3d(-1, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 0, -1));

    EXPECT_TRUE(t1.lies_on_the_same_plane_with(t2));
    EXPECT_FALSE(t1.lies_on_the_same_plane_with(t3));
    EXPECT_TRUE(t1.lies_on_parallel_planes_with(t3));
    EXPECT_FALSE(t4.lies_on_parallel_planes_with(t1));
    EXPECT_FALSE(t4.lies_on_the_same_plane_with(t1));
}

TEST(UnitTests, is_degenerate) {
    Triangle t1(Vector3d(0, 0, 0), Vector3d(0, 2, 0), Vector3d(0, 4, 0));
    EXPECT_TRUE(t1.is_degenerate());

    Triangle t2(Vector3d(0, 0, 0), Vector3d(0, 2, 1), Vector3d(0, 4, 0));
    EXPECT_FALSE(t2.is_degenerate());

}

TEST(UnitTests, Intersection2d) {
    Triangle t1(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289,2.21191,0));
    Triangle t2(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(2, 0, 0));

    EXPECT_TRUE(intersection_test_2d(t1, t2));
    EXPECT_TRUE(intersection_test_2d(t2, t1));

    Triangle t3(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289,2.21191,0));
    Triangle t4(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(1, 0, 0));

    EXPECT_TRUE(intersection_test_2d(t3, t4));
    EXPECT_TRUE(intersection_test_2d(t4, t3));

    Triangle t5(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289, 2.21191, 0));
    Triangle t6(Vector3d(-2, 0, 0), Vector3d(3.48127, 2.09539, 0), Vector3d(1, 0, 0));

    EXPECT_TRUE(intersection_test_2d(t5, t6));
    EXPECT_TRUE(intersection_test_2d(t6, t5));

    Triangle t7(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289, 2.21191, 0));
    Triangle t8(Vector3d(0.80605, 1.76459, 0), Vector3d(2.0703, 1.36268, 0), Vector3d(1.76179, 0.60273, 0));

    EXPECT_TRUE(intersection_test_2d(t7, t8));
    EXPECT_TRUE(intersection_test_2d(t8, t7));

    Triangle t9(Vector3d(0, 0, 0), Vector3d(2, 0, 0), Vector3d(4, 0, 0));
    Triangle t10(Vector3d(0.80605, 1.76459, 0), Vector3d(2.0703, 1.36268, 0), Vector3d(1.76179, 0.60273, 0));

    EXPECT_FALSE(intersection_test_2d(t9, t10));
    EXPECT_FALSE(intersection_test_2d(t10, t9));
}

TEST(UnitTests, AABB) {
    Triangle t1(Vector3d(9.21269, 0.510527, 3.78626), Vector3d(8.08306, 3.27682, 0.730073), Vector3d(8.42607, 1.32866, 0.572925));
    Triangle t2(Vector3d(7.16461, 1.96139, 2.48755), Vector3d(6.68718, 4.28441, 0.892826), Vector3d(6.62557, 1.08196, 0.75515));

    EXPECT_FALSE(intersection_test_3d(t1, t2));
    EXPECT_FALSE(intersection_test_3d(t2, t1));
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

TEST(UnitTests, Vector3dBasics) {
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

TEST(UnitTests, Intersection3d) {

    Triangle t1(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289,2.21191,0));
    Triangle t2(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(2, 0, 0));

    EXPECT_TRUE(intersection_test_3d(t1, t2));
    EXPECT_TRUE(intersection_test_3d(t2, t1));

    Triangle t3(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289,2.21191,0));
    Triangle t4(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(1, 0, 0));

    EXPECT_TRUE(intersection_test_3d(t3, t4));
    EXPECT_TRUE(intersection_test_3d(t4, t3));

    Triangle t5(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289, 2.21191, 0));
    Triangle t6(Vector3d(-2, 0, 0), Vector3d(3.48127, 2.09539, 0), Vector3d(1, 0, 0));

    EXPECT_TRUE(intersection_test_3d(t5, t6));
    EXPECT_TRUE(intersection_test_3d(t6, t5));

    Triangle t7(Vector3d(0, 2, 0), Vector3d(2, 0, 0), Vector3d(2.74289, 2.21191, 0));
    Triangle t8(Vector3d(0.80605, 1.76459, 0), Vector3d(2.0703, 1.36268, 0), Vector3d(1.76179, 0.60273, 0));

    EXPECT_TRUE(intersection_test_3d(t7, t8));
    EXPECT_TRUE(intersection_test_3d(t8, t7));

    Triangle t9(Vector3d(0, 0, 0), Vector3d(2, 0, 0), Vector3d(4, 0, 0));
    Triangle t10(Vector3d(0.80605, 1.76459, 0), Vector3d(2.0703, 1.36268, 0), Vector3d(1.76179, 0.60273, 0));

    EXPECT_FALSE(intersection_test_3d(t9, t10));
    EXPECT_FALSE(intersection_test_3d(t10, t9));

    Triangle t11(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(2, 0, 0));
    Triangle t12(Vector3d(-2, 0, 1), Vector3d(0, 2, 1), Vector3d(2, 0, 1));

    EXPECT_FALSE(intersection_test_3d(t11, t12));
    EXPECT_FALSE(intersection_test_3d(t12, t11));

    Triangle t13(Vector3d(0, 0, 0), Vector3d(2, 0, 1.22359), Vector3d(4, 0, 0));
    Triangle t14(Vector3d(1.80814, -1.02431, 0), Vector3d(0.80605, 1.76459, 0), Vector3d(1.28348, 0.54633, -1.85131));

    EXPECT_TRUE(intersection_test_3d(t13, t14));
    EXPECT_TRUE(intersection_test_3d(t14, t13));

    Triangle t15(Vector3d(0, 0, 0), Vector3d(2, 0, 1.22359), Vector3d(4, 0, 0));
    Triangle t16(Vector3d(1.80814, -1.02431, 0.59548), Vector3d(0.80605, 1.76459, 0.5727), Vector3d(1.28348, 0.54633, -1.85131));

    EXPECT_TRUE(intersection_test_3d(t15, t16));
    EXPECT_TRUE(intersection_test_3d(t16, t15));

    Triangle t17(Vector3d(0, 0, 0), Vector3d(2, 0, 1.22359), Vector3d(4, 0, 0));
    Triangle t18(Vector3d(1.80814, -1.02431, -0.55653), Vector3d(0.80605, 1.76459, -0.5727), Vector3d(1.28348, 0.54633, -1.85131));

    EXPECT_FALSE(intersection_test_3d(t17, t18));
    EXPECT_FALSE(intersection_test_3d(t18, t17));

    Triangle t19(Vector3d(0, 0, 0), Vector3d(2, 0, 0), Vector3d(4, 0, 0));
    Triangle t20(Vector3d(1.80814, -1.02431, 4), Vector3d(0.80605, 1.76459, 4), Vector3d(1.30151, 0.31348, -1.85131));

    EXPECT_FALSE(intersection_test_3d(t19, t20));
    EXPECT_FALSE(intersection_test_3d(t20, t19));

    Triangle t21(Vector3d(6.40007, 8.32122, 7.16085), Vector3d(7.89707, 9.64212, 6.71809), Vector3d(8.84919, 6.87275, 8.15116));
    Triangle t22(Vector3d(6.45422, 8.61347, 7.59351), Vector3d(7.84292, 9.34987, 6.28543), Vector3d(8.84919, 6.87275, 8.15116));

    EXPECT_TRUE(intersection_test_3d(t21, t22));
    EXPECT_TRUE(intersection_test_3d(t22, t21));

    Triangle t23(Vector3d(13.2473, 12.1462, 15.3973), Vector3d(14.1533, 16.5503, 10.9817), Vector3d(15.0053, 14.4069, 13.6928));
    Triangle t24(Vector3d(13.5954, 10.3765, 8.84058), Vector3d(13.9047, 8.06557, 6.10273), Vector3d(14.9951, 13.3611, 8.21311));

    EXPECT_FALSE(intersection_test_3d(t23, t24));
    EXPECT_FALSE(intersection_test_3d(t24, t23));

    Triangle t25(Vector3d(15.5805, 7.37613, 12.3125), Vector3d(16.0595, 8.31296, 13.5955), Vector3d(16.68, 8.14817, 13.6254));
    Triangle t26(Vector3d(11.433, 9.42941, 12.7986), Vector3d(11.9465, 9.13992, 13.6813), Vector3d(12.7467, 9.14475, 12.2916));

    EXPECT_FALSE(intersection_test_3d(t25, t26));
    EXPECT_FALSE(intersection_test_3d(t26, t25));

    Triangle t27(Vector3d(0, 0, 0), Vector3d(0, 2, 0), Vector3d(2, 0, 0));
    Triangle t28(Vector3d(0.56, 0.76, 0), Vector3d(0, 0, 3), Vector3d(1, 1, 3));

    EXPECT_TRUE(intersection_test_3d(t27, t28));
    EXPECT_TRUE(intersection_test_3d(t28, t27));
}

TEST(End2End, N_intersections) {
    Vector3d min(0, 0, 0), max(100, 100, 100);
    N_intersections_test_output output = generate_N_intersections_test(1);
    std::vector<Triangle> triangles = output.triangles;
    std::cout << triangles.size() << " triangles." << std::endl;
    int N = output.N;
    AABB scene_bb(min, max);
    // for(auto&& tr : triangles) {
    //     std::cout << tr << std::endl;
    // }
    std::cout << "scene_bb: " << scene_bb.max << " and " << scene_bb.min << ", " << triangles.size() << " triangles." << std::endl;
    size_t max_depth = 2;

    Octree scene_tree(scene_bb, max_depth);

    for(int i = 0; i < triangles.size(); ++i) {
        scene_tree.insert(triangles[i], i);
    }

    auto set = scene_tree.count_intersections(triangles);
    // std::for_each(set.begin(), set.end(), [](auto i){ std::cout << i << " "; });
    std::cout << std::endl << "Empty? " << (set.empty() ? "Yes" : "No") << std::endl;
    std::cout << "Set size is " << set.size() << " and " << N << " were announced.";
    EXPECT_EQ(set.size(), N);
}

TEST(End2End, zero_intersections) {
    Vector3d min(0, 0, 0), max(100, 100, 100);
    std::vector<Triangle> triangles = generate_zero_intersections_test();
    std::cout << triangles.size() << " triangles." << std::endl;
    int N = 0;
    AABB scene_bb(min, max);
    // for(auto&& tr : triangles) {
    //     std::cout << tr << std::endl;
    // }
    std::cout << "scene_bb: " << scene_bb.max << " and " << scene_bb.min << ", " << triangles.size() << " triangles." << std::endl;

    Octree scene_tree(scene_bb);

    for(int i = 0; i < triangles.size(); ++i) {
        scene_tree.insert(triangles[i], i);
    }
    // scene_tree.dump();
    // std::cout << std::endl;

    auto set = scene_tree.count_intersections(triangles);
    // std::for_each(set.begin(), set.end(), [](auto i){ std::cout << i << " "; });
    std::cout << std::endl << "Empty? " << (set.empty() ? "Yes" : "No") << std::endl;
    std::cout << "Set size is " << set.size() << " and " << N << " were announced.";
    EXPECT_EQ(set.size(), N);
}
