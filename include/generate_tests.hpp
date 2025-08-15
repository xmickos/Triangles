#include "bvh.hpp"
#include "triangles.hpp"
#include <random>

std::vector<hw3d::Triangle> generate_zero_intersections_test(hw3d::Vector3d min, hw3d::Vector3d max);

std::vector<hw3d::Triangle> generate_zero_intersections_test(
        double min_lower=1e-3,
        double min_upper=10,
        double max_lower=10.1,
        double max_upper=20
);

inline double scale_to_interval(double x, double a, double b, double c, double d);

struct N_intersections_test_output final {
    std::vector<Triangles> triangles;
    int N;
};

N_intersections_test_output generate_N_intersections_test(Vector3d min, Vector3d max);

std::vector<hw3d::Triangle> insert_triangles_in_scene(hw3d::AABB scene_bb, double x_step, double y_step, double z_step);
