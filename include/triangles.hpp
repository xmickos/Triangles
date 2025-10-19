#include <vector>
#include <array>
#include <cmath>
#include <utility>
#include <algorithm>
#include <stdexcept>
#include <iostream>

#pragma once

namespace hw3d {

    struct Vector3d;
    struct AABB;

    using edge = std::pair<Vector3d, Vector3d>;

    struct Vector3d final {
        std::array<double, 3> cs_;

        const static inline double float_tolerance = 1e-9;

        Vector3d() = default;

        Vector3d(double x_, double y_, double z_) : cs_{x_, y_, z_} {}

        Vector3d(const edge& e) {
            std::transform(
                e.second.cbegin(), e.second.cend(), e.first.cbegin(), cs_.begin(),
                std::minus<>{}
            );
        }

        Vector3d operator+(const Vector3d& rhs) const {
            Vector3d tmp(*this);
            std::transform(tmp.cbegin(), tmp.cend(), rhs.cs_.cbegin(), tmp.cs_.begin(), std::plus<>());
            return tmp;
        }

        Vector3d operator-(const Vector3d& rhs) const {
            Vector3d tmp(*this);
            std::transform(
                tmp.cbegin(), tmp.cend(), rhs.cs_.begin(), tmp.cs_.begin(),
                [&](auto it1, auto it2){ return it1 -= it2; }
            );
            return tmp;
        }

        Vector3d cross_product(const Vector3d& rhs) const { // [*this x rhs]
            return Vector3d(
                cs_[1] * rhs.cs_[2] - rhs.cs_[1] * cs_[2], // y1z2 - y2z1
                cs_[2] * rhs.cs_[0] - cs_[0] * rhs.cs_[2], // z1x2 - x1z2
                cs_[0] * rhs.cs_[1] - rhs.cs_[0] * cs_[1]  // x1y2 - x2y1
            );
        }

        double dot_product(const Vector3d& rhs) const {
            return cs_[0] * rhs.cs_[0] + cs_[1] * rhs.cs_[1] + cs_[2] * rhs.cs_[2];
        }

        double norm() const {
            return std::sqrt(cs_[0] * cs_[0] + cs_[1] * cs_[1] + cs_[2] * cs_[2]);
        }

        Vector3d normalized() const {
            double len2 = cs_[0] * cs_[0] + cs_[1] * cs_[1] + cs_[2] * cs_[2];
            if(len2 < float_tolerance) {
                throw std::invalid_argument("Vector has zero norm.");
            }
            double inv = 1.0 / std::sqrt(len2);
            return {cs_[0] * inv, cs_[1] * inv, cs_[2] * inv};
        }

        void normalize_inplace() {
            Vector3d tmp = normalized();
            std::swap(tmp, *this);
        }

        Vector3d rotate_around_origin(const Vector3d& axis, double angle_rad) const;

        Vector3d rotate(const Vector3d& O, const Vector3d axis, double angle_rad) const {
            Vector3d tmp(*this);
            tmp = (tmp - O).rotate_around_origin(axis, angle_rad) + O;
            return tmp;
        }

        std::array<double, 3>::const_iterator cbegin() const noexcept { return cs_.cbegin(); }
        std::array<double, 3>::const_iterator cend() const noexcept { return cs_.cend(); }
        std::array<double, 3>::iterator begin() noexcept { return cs_.begin(); }
        std::array<double, 3>::iterator end() noexcept { return cs_.end(); }
        double operator[](int i) const { return cs_[i]; }
        double& operator[](int i) { return cs_[i]; }
    };

    Vector3d operator*(double lhs, const Vector3d& rhs);

    Vector3d operator*(const Vector3d& lhs, double rhs);

    Vector3d operator/(double lhs, Vector3d& rhs);

    Vector3d operator/(const Vector3d& lhs, const Vector3d& rhs);

    Vector3d operator/(double lhs, Vector3d& rhs);

    Vector3d operator*(const Vector3d& lhs, double rhs);

    Vector3d operator/(const Vector3d lhs, double s);

    std::ostream& operator<<(std::ostream& os, const Vector3d& vec);

    class Line final {
        private:
            Vector3d p_, d_;
            double float_tolerance = 1e-9;

        public:
            Line(const Vector3d& a, const Vector3d& b) : p_(a), d_(b - a) {}

            const Vector3d& p() const noexcept { return p_; }
            const Vector3d& d() const noexcept { return d_; }

            bool contains(const Vector3d& p) const {
                Vector3d tmp = p - p_;
                return std::fabs(tmp.cross_product(d_).norm()) < float_tolerance;
            }
    };

    struct Interval final {
        Vector3d p1, p2;

        Interval() {}
        Interval(const Vector3d& p1_, const Vector3d& p2_) : p1(p1_), p2(p2_) {}

        bool contains(const Vector3d& rhs) const noexcept {
            double float_tolerance = 1e-9;
            return Line(p1, p2).contains(rhs) &&
                ((p1 - rhs).dot_product(p2 - p1) * (p2 - rhs).dot_product(p2 - p1) < 0 ||
                std::fabs((p1 - rhs).dot_product(p2 - p1) * (p2 - rhs).dot_product(p2 - p1)) < float_tolerance);
        }

        bool overlaps_with(const Interval& rhs) const noexcept {
            return contains(rhs.p1) || contains(rhs.p2) || rhs.contains(p1) || rhs.contains(p2);
        }
    };

    struct Plane final {
        Vector3d n, p;
        double a, b, c, d;

        Plane(const Vector3d& n_, const Vector3d p_) : n(n_), p(p_) {
            a = n[0];
            b = n[1];
            c = n[2];
            d = -n.dot_product(p);
        }
    };

    class Triangle final {
        private:
            std::array<Vector3d, 3> vertexes;
            std::array<edge, 3> edges;
        public:
            const static inline double float_tolerance = 1e-9;

            Triangle(const Vector3d& a, const Vector3d& b, const Vector3d& c) : vertexes({a, b, c}), edges({edge(a, b), edge(b, c), edge(c, a)}) {}

            std::array<Vector3d, 3>::const_iterator cbegin() const noexcept { return vertexes.cbegin(); }
            std::array<Vector3d, 3>::const_iterator cend() const noexcept { return vertexes.cend(); }
            std::array<edge, 3>::const_iterator ecbegin() const noexcept { return edges.cbegin(); }
            std::array<edge, 3>::const_iterator ecend() const noexcept { return edges.cend(); }
            const Vector3d operator[](int i) const { return vertexes[i]; }

            bool is_degenerate() const {
                Vector3d tmp1(edges[0]);
                Vector3d tmp2(edges[1]);

                return tmp1.cross_product(tmp2).norm() < float_tolerance;
            }

            Plane get_plane_equation() const {
                Vector3d n = (vertexes[1] - vertexes[0]).cross_product(vertexes[2] - vertexes[0]).normalized();
                return Plane(n, vertexes[1]);
            }

            bool lies_on_parallel_planes_with(const Triangle& rhs) const {
                Plane pq1 = get_plane_equation();
                Plane pq2 = rhs.get_plane_equation();
                return pq1.n.cross_product(pq2.n).norm() < float_tolerance;
            }

            bool lies_on_the_same_plane_with(const Triangle& rhs) const {
                Plane pq1 = get_plane_equation();
                Plane pq2 = rhs.get_plane_equation();
                return std::all_of(rhs.ecbegin(), rhs.ecend(),
                    [&](auto edge){ return pq1.n.dot_product(edge.second - edge.first) < float_tolerance; }
                ) && std::fabs(pq1.d - pq2.d) < float_tolerance;
            }

            bool contains_point(Vector3d p_) const {
                if(is_degenerate()) return false;
                Plane pl = get_plane_equation();
                if(std::fabs(pl.n.dot_product(pl.p - p_)) > float_tolerance) {
                    return false;
                } else {
                    std::array<double, 3> s;
                    for(int i = 0; i < 3; ++i) {
                        Vector3d p_i = p_ - vertexes[i];
                        s[i] = (edges[i].second - edges[i].first).cross_product(p_i).dot_product(pl.n);
                    }
                    return std::all_of(s.begin(), s.end(), [](auto it){ return it > float_tolerance; }) || std::all_of(s.begin(), s.end(), [](auto it){ return it <= float_tolerance; });
                }
                return false;
            }

            Triangle rotate(Vector3d O, Vector3d axis, double angle_rad) const {
                std::array<Vector3d, 3> init_vecs = vertexes;
                std::transform(init_vecs.begin(), init_vecs.end(), init_vecs.begin(), [&](auto vec){ return (vec - O).rotate_around_origin(axis, angle_rad) + O; });
                return Triangle(init_vecs[0], init_vecs[1], init_vecs[2]);
            }
    };

    std::ostream& operator<<(std::ostream& os, const Triangle& tr);

    bool check_interval_overlap(const Line& a, const Line& b);

    Vector3d line_plane_intersection(const Line& l, const Triangle& a);

    bool intersection_test_2d(const Triangle& a, const Triangle& b);

    int plane_point_location(const Triangle& tr, const Vector3d& p);

    bool intersection_test_3d(const Triangle& a, const Triangle& b);

    std::pair<double, double> compute_interval(const Triangle& t, const Vector3d& v);

    std::ostream& operator<<(std::ostream& os, const AABB& aabb);
}
