#include<vector>

#pragma once

namespace hw3d {

    class Vector3d final {
        private:
            std::vector<double> cs_;

        public:
            Vector3d(double x_, double y_, double z_) {
                cs_.push_back(x_);
                cs_.push_back(y_);
                cs_.push_back(z_);
            }

            Vector3d(const Vector3d& rhs) : cs_(rhs.cbegin(), rhs.cend()) {}
            Vector3d(Vector3d&& rhs) { std::swap(cs_, rhs.cs_); }
            Vector3d& operator=(Vector3d& rhs) {
                if(this == &rhs) return *this;
                std::swap(*this, rhs);
                return *this;
            }
            Vector3d& operator=(Vector3d&& rhs) {
                if(this == &rhs) return *this;
                std::swap(cs_, rhs.cs_);
                return *this;
            }

            Vector3d operator+(const Vector3d& rhs) const {
                Vector3d tmp(*this);
                std::transform(tmp.cbegin(), tmp.cend(), rhs.cs_.begin(), tmp.cs_.begin(), [&](auto it1, auto it2){ return it1 += it2; });
                return tmp;
            }

            Vector3d operator-(const Vector3d& rhs) const {
                Vector3d tmp(*this);
                std::transform(tmp.cbegin(), tmp.cend(), rhs.cs_.begin(), tmp.cs_.begin(), [&](auto it1, auto it2){ return it1 -= it2; });
                return tmp;
            }

            Vector3d cross_product(const Vector3d& rhs) const {
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

            std::vector<double>::const_iterator cbegin() const noexcept { return cs_.cbegin(); }
            std::vector<double>::const_iterator cend() const noexcept { return cs_.cend(); }
            std::vector<double>::const_iterator begin() const noexcept { return cs_.begin(); }
            std::vector<double>::const_iterator end() const noexcept { return cs_.end(); }
    };

    Vector3d operator*(double lhs, Vector3d& rhs);

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

    struct Plane final {
        Vector3d n, p;

        Plane(const Vector3d& n_, const Vector3d p_) : n(n_), p(p_) {}

        // Line get_plane_intersection(const Plane& rhs) const {
        // }
    };

    class Triangle final {
        private:
            std::vector<Vector3d> vs_;
            double float_tolerance = 1e-9;

        public:
            Triangle(const Vector3d& a, const Vector3d& b, const Vector3d& c) {
                vs_.push_back(a);
                vs_.push_back(b);
                vs_.push_back(c);
            }

            std::vector<Vector3d>::const_iterator cbegin() const noexcept { return vs_.cbegin(); }
            std::vector<Vector3d>::const_iterator cend() const noexcept { return vs_.cend(); }

            bool is_degenerate() const {
                Vector3d tmp1(vs_[0] - vs_[1]);
                Vector3d tmp2(vs_[1] - vs_[2]);

                return std::fabs(tmp1.cross_product(tmp2).norm()) < float_tolerance;
            }

            Plane get_plane_equation() const {
                Vector3d p = vs_[0] - vs_[1];
                Vector3d q = vs_[1] - vs_[2];
                Vector3d u = vs_[0] - vs_[2];
                Vector3d n = p.cross_product(q);

                return Plane(p, n);
            }

            bool lies_on_parallel_planes_with(const Triangle& rhs) const {
                Plane pq1 = get_plane_equation();
                Plane pq2 = rhs.get_plane_equation();
                Vector3d n1 = pq1.n;
                Vector3d n2 = pq2.n;

                return n1.cross_product(n2).norm() > float_tolerance;
            }

            bool lies_on_the_same_plane_with(const Triangle& rhs) const {
                Plane pq1 = get_plane_equation();
                Plane pq2 = rhs.get_plane_equation();
                Vector3d n1 = pq1.n;
                Vector3d n2 = pq2.n;
                Vector3d p1 = pq1.p;
                Vector3d p2 = pq2.p;

                double d1 = n1.dot_product(p1);
                double d2 = n2.dot_product(p2);

                return std::fabs(d1 - d2) < float_tolerance;
            }
    };

    bool check_interval_overlap(const Line& a, const Line& b);

    Vector3d line_plane_intersection(const Line& l, const Triangle& a);

    bool intersection_test_2d(const Triangle& a, const Triangle& b);

    bool plane_point_location(const Triangle& tr, const Vector3d& p);

    bool intersection_test_3d(const Triangle& a, const Triangle& b);

}
