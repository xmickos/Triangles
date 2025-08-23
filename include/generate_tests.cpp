#include "bvh.hpp"
#include "triangles.hpp"
#include <random>
#include <numbers>

using namespace hw3d;

inline double scale_to_interval(double x, double a, double b, double c, double d) {
    // scales x from (a, b] into the (c, d], if a > c, b > d
    return c + (x - a) * (d - c) / (b - a);
}

std::vector<Triangle> insert_triangles_in_scene(AABB scene_bb, double x_step, double y_step, double z_step) {
    int x_cells_count = std::ceil(std::abs(scene_bb.max.cs_[0] - scene_bb.min.cs_[0]) / x_step);
    int y_cells_count = std::ceil(std::abs(scene_bb.max.cs_[1] - scene_bb.min.cs_[1]) / y_step);
    int z_cells_count = std::ceil(std::abs(scene_bb.max.cs_[2] - scene_bb.min.cs_[2]) / z_step);
    std::vector<int> cells_count({x_cells_count, y_cells_count, z_cells_count});
    std::vector<Triangle> triangles;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(scene_bb.min.cs_[0], scene_bb.max.cs_[0]);
    double padding_ratio = 0.1;

    for(int x_i = 0; x_i < x_cells_count; ++x_i) {
        for(int y_i = 0; y_i < y_cells_count; ++y_i) {
            for(int z_i = 0; z_i < z_cells_count; ++z_i) {
                double delta_x = (scene_bb.min.cs_[0] + x_step) * padding_ratio;
                double delta_y = (scene_bb.min.cs_[0] + y_step) * padding_ratio;
                double delta_z = (scene_bb.min.cs_[0] + z_step) * padding_ratio;

                Triangle tr(
                    Vector3d(
                            scale_to_interval(
                                dist(gen),
                                scene_bb.min.cs_[0],
                                scene_bb.max.cs_[0],
                                scene_bb.min.cs_[0] + x_step * x_i + delta_x,
                                scene_bb.min.cs_[0] + x_step * (x_i + 1) - delta_x
                            ),
                            scale_to_interval(
                                dist(gen),
                                scene_bb.min.cs_[0],
                                scene_bb.max.cs_[0],
                                scene_bb.min.cs_[1] + y_step * y_i + delta_y,
                                scene_bb.min.cs_[1] + y_step * (y_i + 1) - delta_y
                            ),
                            scale_to_interval(
                                dist(gen),
                                scene_bb.min.cs_[0],
                                scene_bb.max.cs_[0],
                                scene_bb.min.cs_[2] + z_step * z_i + delta_z,
                                scene_bb.min.cs_[2] + z_step * (z_i + 1) - delta_x
                            )
                        ),
                    Vector3d(
                            scale_to_interval(
                                dist(gen),
                                scene_bb.min.cs_[0],
                                scene_bb.max.cs_[0],
                                scene_bb.min.cs_[0] + x_step * x_i + delta_x,
                                scene_bb.min.cs_[0] + x_step * (x_i + 1) - delta_x
                            ),
                            scale_to_interval(
                                dist(gen),
                                scene_bb.min.cs_[1],
                                scene_bb.max.cs_[1],
                                scene_bb.min.cs_[1] + y_step * y_i + delta_y,
                                scene_bb.min.cs_[1] + y_step * (y_i + 1) - delta_y
                            ),
                            scale_to_interval(
                                dist(gen),
                                scene_bb.min.cs_[2],
                                scene_bb.max.cs_[2],
                                scene_bb.min.cs_[2] + z_step * z_i + delta_z,
                                scene_bb.min.cs_[2] + z_step * (z_i + 1) - delta_z
                            )
                        ),
                    Vector3d(
                            scale_to_interval(
                                dist(gen),
                                scene_bb.min.cs_[0],
                                scene_bb.max.cs_[0],
                                scene_bb.min.cs_[0] + x_step * x_i + delta_x,
                                scene_bb.min.cs_[0] + x_step * (x_i + 1) - delta_x
                            ),
                            scale_to_interval(
                                dist(gen),
                                scene_bb.min.cs_[1],
                                scene_bb.max.cs_[1],
                                scene_bb.min.cs_[1] + y_step * y_i + delta_y,
                                scene_bb.min.cs_[1] + y_step * (y_i + 1) - delta_y
                            ),
                            scale_to_interval(
                                dist(gen),
                                scene_bb.min.cs_[2],
                                scene_bb.max.cs_[2],
                                scene_bb.min.cs_[2] + z_step * z_i + delta_z,
                                scene_bb.min.cs_[2] + z_step * (z_i + 1) - delta_z
                            )
                        )
                    );
                triangles.push_back(tr);
            }
        }
    }

    return triangles;
}

std::vector<Triangle> generate_zero_intersections_test(Vector3d min, Vector3d max) {
    hw3d::AABB scene_bb(min, max);
    std::vector<Triangle> triangles = insert_triangles_in_scene(scene_bb, 5, 2.5, 2.5);
    return triangles;
}

std::vector<Triangle> generate_zero_intersections_test(
        double min_lower=1e-3,
        double min_upper=10,
        double max_lower=10.1,
        double max_upper=20
    ) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist_min(min_lower, min_upper);
    std::uniform_real_distribution<> dist_max(max_lower, max_upper);
    Vector3d min(dist_min(gen), dist_min(gen), dist_min(gen));
    Vector3d max(dist_max(gen), dist_max(gen), dist_max(gen));

    return generate_zero_intersections_test(min, max);
}

struct N_intersections_test_output final {
    std::vector<Triangle> triangles;
    int N;
};

N_intersections_test_output generate_N_intersections_test(Vector3d min, Vector3d max) {
    hw3d::AABB scene_bb(min, max);
    std::vector<Triangle> triangles = insert_triangles_in_scene(scene_bb, 5, 5, 5);
    std::vector<Triangle> triangles_rotated;
    triangles_rotated.reserve(triangles.size());
    std::random_device rd;
    std::mt19937 gen(rd());
    double pi_rad = std::numbers::pi / 6;
    std::uniform_real_distribution<> dist(0, pi_rad); // rotate angle is sampled from [0, 30] degrees

    for(auto it = triangles.begin(), et = triangles.end(); it != et; ++it) {
        Vector3d median = (((*it)[0] + (*it)[1]) / 2 - (*it)[2]).normalized();
        triangles_rotated.push_back(it->rotate((*it)[2], median, std::numbers::pi / 6));
    }

    triangles.insert(triangles.end(), triangles_rotated.begin(), triangles_rotated.end());

    return N_intersections_test_output(triangles, triangles.size());
}
