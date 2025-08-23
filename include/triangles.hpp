#include<vector>
#include<format>

#pragma once

namespace hw3d {

    class Vector3d;

    using edge = std::pair<Vector3d, Vector3d>;

    class Vector3d final {
        public:
            std::vector<double> cs_;

        private:
            double float_tolerance = 1e-9;

        public:
            Vector3d() : cs_(3, 0.0) {}

            Vector3d(double x_, double y_, double z_) {
                cs_.push_back(x_);
                cs_.push_back(y_);
                cs_.push_back(z_);
            }

            Vector3d(double q_) : cs_(3, q_) {}

            Vector3d(const edge& e) : cs_(3) {
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
                    throw std::invalid_argument(
                        std::format("Vector {} has zero norm.", static_cast<const void*>(this))
                    );
                }
                double inv = 1.0 / std::sqrt(len2);
                return {cs_[0] * inv, cs_[1] * inv, cs_[2] * inv};
            }

            void normalize_inplace() {
                Vector3d tmp = normalized();
                std::swap(tmp, *this);
            }

            #if 0
            std::ostream& operator<<(std::ostream& os) const {
                os << "(" << cs_[0] << ", " << cs_[1] << ", " << cs_[2] << ")";
                return os;
            }
            #endif

            Vector3d rotate_around_origin(const Vector3d& axis, double angle_rad) const;

            Vector3d rotate(const Vector3d& O, const Vector3d axis, double angle_rad) const {
                Vector3d tmp(*this);
                tmp = (tmp - O).rotate_around_origin(axis, angle_rad) + O;
                return tmp;
            }

            std::vector<double>::const_iterator cbegin() const noexcept { return cs_.cbegin(); }
            std::vector<double>::const_iterator cend() const noexcept { return cs_.cend(); }
            std::vector<double>::iterator begin() noexcept { return cs_.begin(); }
            std::vector<double>::iterator end() noexcept { return cs_.end(); }
            const double operator[](int i) const { return cs_[i]; }
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
            std::vector<Vector3d> vertexes;
            std::vector<edge> edges;
            double float_tolerance = 1e-9;

        public:
            Triangle(const Vector3d& a, const Vector3d& b, const Vector3d& c) {
                vertexes.push_back(a);
                vertexes.push_back(b);
                vertexes.push_back(c);

                edges.push_back(edge(a, b));
                edges.push_back(edge(b, c));
                edges.push_back(edge(c, a));

            }

            std::vector<Vector3d>::const_iterator cbegin() const noexcept { return vertexes.cbegin(); }
            std::vector<Vector3d>::const_iterator cend() const noexcept { return vertexes.cend(); }
            std::vector<edge>::const_iterator ecbegin() const noexcept { return edges.cbegin(); }
            std::vector<edge>::const_iterator ecend() const noexcept { return edges.cend(); }
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

            Triangle rotate(Vector3d O, Vector3d axis, double angle_rad) const {
                std::vector<Vector3d> init_vecs = vertexes;
                std::transform(init_vecs.begin(), init_vecs.end(), init_vecs.begin(), [&](auto vec){ return (vec - O).rotate_around_origin(axis, angle_rad) + O; });
                return Triangle(init_vecs[0], init_vecs[1], init_vecs[2]);
            }
    };

    std::ostream& operator<<(std::ostream& os, const Triangle& tr);

    bool check_interval_overlap(const Line& a, const Line& b);

    Vector3d line_plane_intersection(const Line& l, const Triangle& a);

    bool intersection_test_2d(const Triangle& a, const Triangle& b);

    bool plane_point_location(const Triangle& tr, const Vector3d& p);

    bool intersection_test_3d(const Triangle& a, const Triangle& b);

    std::pair<double, double> compute_interval(const Triangle& t, const Vector3d& v);

}
