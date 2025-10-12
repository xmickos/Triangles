#include<iostream>
#include"triangles.hpp"
#include"bvh.hpp"

namespace hw3d {

    template <typename T>
    int my_sign(T val) {
        return (T(0) < val) - (val < T(0));
    }


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

    Vector3d line_plane_intersection(const Line& l, const Triangle& a) {
        double float_tolerance = 1e-9;
        Plane pq = a.get_plane_equation();
        Vector3d na = pq.n;
        Vector3d pa = pq.p;
        Vector3d ld = l.d();
        Vector3d lp = l.p();
        if(std::fabs(na.dot_product(ld)) < float_tolerance) {
            return Vector3d(NAN, NAN, NAN);
        } else {
            double t = na.dot_product(pa - lp) / na.dot_product(ld);
            return Vector3d(lp + t * ld);
        }
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

    int plane_point_location(const Triangle& tr, const Vector3d& p) {
        Plane pq = tr.get_plane_equation();
        Vector3d n_ = pq.n;
        Vector3d p_ = pq.p;

        return my_sign(n_.dot_product(p_ - p));
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
            std::all_of(a_points_related_to_b.begin(), a_points_related_to_b.end(),
                [](auto it){ return it < 0; }) ||
            std::all_of(a_points_related_to_b.begin(), a_points_related_to_b.end(),
                [](auto it){ return it > 0; })
        ) { return false; }

        Interval intersection_interval1;
        Interval intersection_interval2;

        int num_of_zeros_a = std::count(
            b_points_related_to_a.begin(),
            b_points_related_to_a.end(),
            0
        );

        int num_of_zeros_b = std::count(
            a_points_related_to_b.begin(),
            a_points_related_to_b.end(),
            0
        );

        switch(num_of_zeros_a) {
            case 2:
                if(b_points_related_to_a[0] != 0) {
                    Vector3d a_anchor = a[0];
                    Vector3d p1 = line_plane_intersection(Line(a_anchor, a[1]), b);
                    Vector3d p2 = line_plane_intersection(Line(a_anchor, a[2]), b);
                    intersection_interval1 = Interval(p1, p2);
                } else if(b_points_related_to_a[1] != 0) {
                    Vector3d a_anchor = a[1];
                    Vector3d p1 = line_plane_intersection(Line(a_anchor, a[0]), b);
                    Vector3d p2 = line_plane_intersection(Line(a_anchor, a[2]), b);
                    intersection_interval1 = Interval(p1, p2);
                } else if(b_points_related_to_a[2] != 0) {
                    Vector3d a_anchor = a[2];
                    Vector3d p1 = line_plane_intersection(Line(a_anchor, a[0]), b);
                    Vector3d p2 = line_plane_intersection(Line(a_anchor, a[1]), b);
                    intersection_interval1 = Interval(p1, p2);
                }
            case 1:
                if(b_points_related_to_a[0] == 0) {
                    return b.contains_point(a[0]);
                }
                if(b_points_related_to_a[1] == 0) {
                    return b.contains_point(a[1]);
                }
                return b.contains_point(a[2]);
            case 0:
                if(b_points_related_to_a[0] < 0) {
                    std::transform(b_points_related_to_a.begin(), b_points_related_to_a.end(), b_points_related_to_a.begin(), [](auto it){ return -it;});
                }
                if(b_points_related_to_a[1] > 0) { // [ + + - ]
                    Vector3d b_anchor = b[2];
                    Vector3d p1 = line_plane_intersection(Line(b_anchor, b[0]), a);
                    Vector3d p2 = line_plane_intersection(Line(b_anchor, b[1]), a);
                    intersection_interval1 = Interval(p1, p2);
                } else {
                    if(b_points_related_to_a[2] > 0) { // [ + - + ]
                        Vector3d b_anchor = b[1];
                        Vector3d p1 = line_plane_intersection(Line(b_anchor, b[0]), a);
                        Vector3d p2 = line_plane_intersection(Line(b_anchor, b[2]), a);
                        intersection_interval1 = Interval(p1, p2);
                    } else { // [ + - - ]
                        Vector3d b_anchor = b[0];
                        Vector3d p1 = line_plane_intersection(Line(b_anchor, b[1]), a);
                        Vector3d p2 = line_plane_intersection(Line(b_anchor, b[2]), a);
                        intersection_interval1 = Interval(p1, p2);
                    }
                }
        }

        switch(num_of_zeros_b) {
            case 2:
                if(a_points_related_to_b[0] != 0) {
                    Vector3d b_anchor = b[0];
                    Vector3d p1 = line_plane_intersection(Line(b_anchor, b[1]), a);
                    Vector3d p2 = line_plane_intersection(Line(b_anchor, b[2]), a);
                    intersection_interval1 = Interval(p1, p2);
                } else if(a_points_related_to_b[1] != 0) {
                    Vector3d b_anchor = b[1];
                    Vector3d p1 = line_plane_intersection(Line(b_anchor, b[0]), a);
                    Vector3d p2 = line_plane_intersection(Line(b_anchor, b[2]), a);
                    intersection_interval1 = Interval(p1, p2);
                } else if(a_points_related_to_b[2] != 0) {
                    Vector3d b_anchor = b[2];
                    Vector3d p1 = line_plane_intersection(Line(b_anchor, b[0]), a);
                    Vector3d p2 = line_plane_intersection(Line(b_anchor, b[1]), a);
                    intersection_interval1 = Interval(p1, p2);
                }
            case 1:
                if(a_points_related_to_b[0] == 0) {
                    return a.contains_point(b[0]);
                }
                if(a_points_related_to_b[1] == 0) {
                    return a.contains_point(b[1]);
                }
                return a.contains_point(b[2]);
            case 0:
                if(a_points_related_to_b[0] < 0) {
                    std::transform(a_points_related_to_b.begin(), a_points_related_to_b.end(), a_points_related_to_b.begin(), [](auto it){ return -it;});
                }
                if(a_points_related_to_b[1] > 0) { // [ + + - ]
                    Vector3d a_anchor = a[2];
                    Vector3d p1 = line_plane_intersection(Line(a_anchor, a[0]), b);
                    Vector3d p2 = line_plane_intersection(Line(a_anchor, a[1]), b);
                    intersection_interval2 = Interval(p1, p2);
                } else {
                    if(a_points_related_to_b[2] > 0) { // [ + - + ]
                        Vector3d a_anchor = a[1];
                        Vector3d p1 = line_plane_intersection(Line(a_anchor, a[0]), b);
                        Vector3d p2 = line_plane_intersection(Line(a_anchor, a[2]), b);
                        intersection_interval2 = Interval(p1, p2);
                    } else { // [ + - - ]
                        Vector3d a_anchor = a[0];
                        Vector3d p1 = line_plane_intersection(Line(a_anchor, a[1]), b);
                        Vector3d p2 = line_plane_intersection(Line(a_anchor, a[2]), b);
                        intersection_interval2 = Interval(p1, p2);
                    }
                }
        }

        return intersection_interval1.overlaps_with(intersection_interval2);
    }
}
