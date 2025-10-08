#include <iostream>
#include "include/bvh.hpp"
#include "include/utils.hpp"
#include <chrono>

using namespace hw3d;

int main() {

    size_t N;
    double x1, y1, z1, x2, y2, z2, x3, y3, z3;
    if (scanf("%zu", &N) != 1) return 0;

    std::vector<Triangle> triangles;
    triangles.reserve(N);

    #ifdef SPEED_BENCH
        auto start0 = std::chrono::high_resolution_clock::now();
    #endif

    AABB scene_bb, min_bb;

    for(size_t i = 0; i < N; ++i) {
        if(scanf("%lf%lf%lf%lf%lf%lf%lf%lf%lf", &x1,&y1,&z1,&x2,&y2,&z2,&x3,&y3,&z3) != 9) break;

        scene_bb.expand_triangle_raw(x1, y1, z1, x2, y2, z2, x3, y3, z3);
        triangles.emplace_back(Vector3d{x1, y1, z1}, Vector3d{x2, y2, z2}, Vector3d{x3, y3, z3});
        AABB tr_bb(triangles.back());
        if(tr_bb.diag_len() < min_bb.diag_len()) { min_bb = tr_bb; }
    }

    size_t depth =  std::floor(std::log2(scene_bb.diag_len() / (min_bb.diag_len() + 1e-8)));
    if(depth > 5) depth = 5;

    #if 0
        std::cout << "depth: " << depth << std::endl;
    #endif

    #ifdef SPEED_BENCH
        auto start1 = std::chrono::high_resolution_clock::now();
    #endif

    Octree scene_tree(scene_bb, depth);

    for(size_t i = 0; i < triangles.size(); ++i) {
        scene_tree.insert(triangles[i], i);         // TODO add padding for final AABB & cache AABBs for triangles
    }                                               // instead of constructing a new one in each insert

    #if SPEED_BENCH
        std::cout << "Empty nodes: " << scene_tree.count_empty_nodes() << std::endl;
        std::cout << "Total nodes: " << scene_tree.total_nodes_count() << std::endl;
        auto start1_1 = std::chrono::high_resolution_clock::now();
    #endif

    scene_tree.count_subtrees_triangles();
    // scene_tree.dump();

    #ifdef SPEED_BENCH
        auto start2 = std::chrono::high_resolution_clock::now();
    #endif

    std::unordered_set<size_t> indexes = scene_tree.count_intersections(triangles);

    #ifdef SPEED_BENCH
        auto start3 = std::chrono::high_resolution_clock::now();
    #endif

    std::vector<size_t> final_indexes(indexes.begin(), indexes.end());
    std::sort(final_indexes.begin(), final_indexes.end());
    #ifndef SPEED_BENCH
        std::for_each(final_indexes.begin(), final_indexes.end(), [](auto it){ std::cout << it << std::endl; });
    #endif
    #ifdef SPEED_BENCH
        auto start4 = std::chrono::high_resolution_clock::now();

        auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(start1 - start0);
        auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(start2 - start1_1);
        auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(start1_1 - start1);
        auto duration4 = std::chrono::duration_cast<std::chrono::milliseconds>(start3 - start2);
        auto duration5 = std::chrono::duration_cast<std::chrono::milliseconds>(start4 - start3);
        auto duration_full = std::chrono::duration_cast<std::chrono::milliseconds>(start4 - start0);

        std::cout << "Collection Triangles vector: " << duration1 << std::endl;
        std::cout << "Octree optimizing: " << duration2 << std::endl;
        std::cout << "Building Octree: " << duration3 << std::endl;
        std::cout << "Computing intersections: " << duration4 << std::endl;
        std::cout << "Sorting and printing: " << duration5 << std::endl;
        std::cout << "Total time spent: " << duration_full << std::endl;
    #endif

    return 0;
}
