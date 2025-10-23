#include<iostream>
#include"triangles.hpp"
#include"bvh.hpp"

namespace hw3d {

    std::ostream& operator<<(std::ostream& os, const Triangle& tr) {
        // os << "△(";
        os << "(";
        for(auto it = tr.cbegin(), et = tr.cend(); it != et; it++) {
            os << *it << ", ";
        }
        os << ")";
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const Vector3d& vec) {
        os << "(" << vec[0] << ", " << vec[1] << ", " << vec[2] << ")";
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const AABB& aabb) {
        os << "[" << aabb.min << ", " << aabb.max << "]";
        return os;
    }

    Vector3d Vector3d::rotate_around_origin(const Vector3d& axis, double angle_rad) const {
        double cos = std::cos(angle_rad);
        double sin = std::sin(-angle_rad);
        return dot_product(axis) * (1 - cos) * axis + axis.cross_product(*this) * sin + cos * (*this);
    }

    Vector3d operator*(double lhs, const Vector3d& rhs) {
        Vector3d tmp(rhs);
        std::transform(tmp.begin(), tmp.end(), tmp.begin(), [&](auto it){ return lhs * it; });
        return tmp;
    }

    Vector3d operator/(const Vector3d& lhs, const Vector3d& rhs) {
        Vector3d tmp(lhs);
        std::transform(tmp.cbegin(), tmp.cend(), rhs.cs_.begin(), tmp.cs_.begin(), std::divides<>());
        return tmp;
    }

    Vector3d operator/(double lhs, Vector3d& rhs) {
        Vector3d tmp(rhs);
        std::transform(tmp.begin(), tmp.end(), tmp.begin(), [&](auto it) { return lhs / it; });
        return tmp;
    }

    Vector3d operator*(const Vector3d& lhs, double rhs) {
        Vector3d tmp(lhs);
        std::transform(tmp.begin(), tmp.end(), tmp.begin(), [&](auto it){ return rhs * it; });
        return tmp;
    }

    Vector3d operator/(const Vector3d lhs, double s) {
        return Vector3d({lhs.cs_[0] / s, lhs.cs_[1] / s, lhs.cs_[2] / s});
    }

    Vector3d line_plane_intersection(const Line& l, const Plane& pq) {
        Vector3d na = pq.n;
        Vector3d pa = pq.p;
        Vector3d ld = l.d();
        Vector3d lp = l.p();
        if(std::fabs(na.dot_product(ld)) < hw3d::float_tolerance) {
            return Vector3d(NAN, NAN, NAN);
        } else {
            double t = na.dot_product(pa - lp) / na.dot_product(ld);
            return Vector3d(lp + t * ld);
        }
    }

    Vector3d line_plane_intersection(const Line& l, const Triangle& a) {
        return line_plane_intersection(l, a.get_plane_equation());
    }

    int plane_point_location(const Triangle& tr, const Vector3d& p) {
        Plane pq = tr.get_plane_equation();
        return std::fabs(pq.n.dot_product(p - pq.p)) < hw3d::float_tolerance ? 0 : (std::signbit(pq.n.dot_product(p - pq.p)) ? -1 : 1);
    }

    bool intersection_test_2d(const Triangle& a, const Triangle& b) {
        if(a.is_degenerate() || b.is_degenerate()) return false;
        Plane a_plane = a.get_plane_equation();
        Vector3d n = a_plane.n;

        bool result = std::all_of(a.ecbegin(), a.ecend(),
            [&](auto eit) {
                Vector3d tmp(eit.second - eit.first);
                Vector3d d = n.cross_product(tmp);
                auto pair_a = compute_interval(a, d);
                auto pair_b = compute_interval(b, d);
                return pair_a.second >= pair_b.first && pair_a.first <= pair_b.second;
            });
        return result;
    }

    std::pair<double, double> compute_interval(const Triangle& t, const Vector3d& v) {
        auto it = t.cbegin();
        double min = v.dot_product(*it), max = min;
        it++;
        std::for_each(it, t.cend(),
            [&](auto et){
                double value = v.dot_product(et);
                if(value < min) {
                    min = value;
                } else if(value > max) {
                    max = value;
                }
            }
        );
        return std::pair<double, double>(min, max);
    }

    std::vector<Vector3d> plane_triangle_intersection(const Triangle& tr, const Plane& pl) {
        std::vector<Vector3d> out, tmp;
        auto eq_pred = [&](Vector3d& v, Vector3d& q){ return (v - q).norm() < hw3d::float_tolerance; };
        for(auto edge = tr.ecbegin(); edge != tr.ecend(); ++edge) {
            if(pl.contains_point(edge->first)) {
                tmp.push_back(edge->first);
                continue;
            }
            Line l(edge->second, edge->first);
            Vector3d p = line_plane_intersection(l, pl);
            if(p.is_valid() && tr.contains_point(p)) { tmp.push_back(p); }
        }
        bool unique = true;
        for(size_t i = 0; i < tmp.size(); ++i) {
            for(size_t j = 0; j < i; ++j) {
                if(eq_pred(tmp[i], tmp[j])) unique = false;
            }
            if(unique) out.push_back(tmp[i]);
            unique = true;
        }
        return out;
    }

    bool intersection_test_3d(const Triangle& a, const Triangle& b) {
        if(a.is_degenerate() || b.is_degenerate()) {
            return false;
        }

        std::vector<int> b_points_related_to_a(3); // contains booleans describing how
        std::vector<int> a_points_related_to_b(3); // points of 'b' are positioned relative to 'a' and vice versa
        std::transform(b.cbegin(), b.cend(), b_points_related_to_a.begin(),
            [&](auto it){ return plane_point_location(a, it); }
        );
        std::transform(a.cbegin(), a.cend(), a_points_related_to_b.begin(),
            [&](auto it){ return plane_point_location(b, it); }
        );

        if(
            std::all_of(b_points_related_to_a.begin(), b_points_related_to_a.end(), [](auto it){ return it < 0; }) ||
            std::all_of(b_points_related_to_a.begin(), b_points_related_to_a.end(), [](auto it){ return it > 0; })
        ) { return false; }

        if(a.lies_on_parallel_planes_with(b)) {
            if(a.lies_on_the_same_plane_with(b)) {
                return intersection_test_2d(a, b);
            } else {
                return false;
            }
        }

        if(
            std::all_of(a_points_related_to_b.begin(), a_points_related_to_b.end(), [](auto it){ return it < 0; }) ||
            std::all_of(a_points_related_to_b.begin(), a_points_related_to_b.end(), [](auto it){ return it > 0; })
        ) { return false; }

        std::vector<Vector3d> a_intersections = plane_triangle_intersection(a, b.get_plane_equation());
        std::vector<Vector3d> b_intersections = plane_triangle_intersection(b, a.get_plane_equation());

        if(a_intersections.size() == 1) {
            return b.contains_point(a_intersections[0]);
        }
        if(b_intersections.size() == 1) {
            return a.contains_point(b_intersections[0]);
        }

        Interval intersection_interval_a(a_intersections[0], a_intersections[1]);
        Interval intersection_interval_b(b_intersections[0], b_intersections[1]);

        return intersection_interval_a.overlaps_with(intersection_interval_b);
    }
}
