#include"triangles.hpp"

namespace hw3d{

    bool intersection_test_2d(const Triangle& a, const Triangle& b);

    bool intersection_test_3d(const Triangle& a, const Triangle& b) {
        if(a.is_degenerate() || b.is_degenerate()) { return false; }

        std::vector<bool> relative_locations_a(3); // contains booleans describing how
        std::vector<bool> relative_locations_b(3); // points of 'b' are positioned relative to 'a' and vice versa
        std::transform(b.cbegin(), b.cend(), relative_locations_a.begin(), [&](auto it){ return plane_point_location(a, it); });
        std::transform(a.cbegin(), a.cend(), relative_locations_b.begin(), [&](auto it){ return plane_point_location(b, it); });

        if(
            std::all_of(relative_locations_a.begin(), relative_locations_a.end(), [](auto it){ return it > 0; }) ||
            std::all_of(relative_locations_a.begin(), relative_locations_a.end(), [](auto it){ return it < 0; })
        ) { return false; }

        if(a.lies_on_parallel_planes_with(b)) {
            if(a.lies_on_the_same_plane_with(b)) {
                return intersection_test_2d(a, b);
            } else {
                return false;
            }
        }

        if(
            std::all_of(relative_locations_b.begin(), relative_locations_b.end(), [](auto it){ return it > 0; }) ||
            std::all_of(relative_locations_b.begin(), relative_locations_b.end(), [](auto it){ return it < 0; })
        ) { return false; }



    }

    bool plane_point_location(const Triangle& tr, const Vector3d& p) {
        auto tr_it = tr.cbegin();
        Vector3d p1 = *(tr_it++);
        Vector3d p2 = *(tr_it++);
        Vector3d p3 = *(tr_it++);

        Vector3d u = p1 - p2;
        Vector3d v = p1 - p3;
        Vector3d n = u.cross_product(v);

        return n.dot_product(p) > 0;
    }
}
