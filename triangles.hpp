#include<vector>

namespace hw3d {

    class Point final {
        private:
            std::vector<double> cs_;

        public:
            Point(double x_, double y_, double z_) {
                cs_.push_back(x_);
                cs_.push_back(y_);
                cs_.push_back(z_);
            }

            Point(const Point& rhs) : cs_(rhs.cbegin(), rhs.cend()) {}
            Point(Point&& rhs) { std::swap(cs_, rhs.cs_); }
            Point& operator=(Point& rhs) {
                if(this == &rhs) return *this;
                std::swap(*this, rhs);
                return *this;
            }
            Point& operator=(Point&& rhs) {
                if(this == &rhs) return *this;
                std::swap(cs_, rhs.cs_);
                return *this;
            }

            Point operator+(const Point& rhs) {
                Point tmp(*this);
                std::transform(tmp.cbegin(), tmp.cend(), rhs.cs_.begin(), tmp.cs_.begin(), [&](auto it1, auto it2){ return *it1 += *it2; });
                return tmp;
            }

            Point operator-(const Point& rhs) const {
                Point tmp(*this);
                std::transform(tmp.cbegin(), tmp.cend(), rhs.cs_.begin(), tmp.cs_.begin(), [&](auto it1, auto it2){ return *it1 -= *it2; });
                return tmp;
            }

            std::vector<double>::const_iterator cbegin() const noexcept { return cs_.cbegin(); }
            std::vector<double>::const_iterator cend() const noexcept { return cs_.cend(); }
    };

    class Line final {
        private:
            Point p_, d_;

        public:
            Line(const Point& a, const Point& b) : p_(a), d_(b - a) {}

            const Point& p() const noexcept { return p_; }
            const Point& d() const noexcept { return d_; }
    };

    class Triangle final {
        private:
            std::vector<Point> vs_;

        public:
            Triangle(const Point& a, const Point& b, const Point& c) {
                vs_.push_back(a);
                vs_.push_back(b);
                vs_.push_back(c);
            }

            std::vector<Point>::const_iterator cbegin() const noexcept { return vs_.cbegin(); }
            std::vector<Point>::const_iterator cend() const noexcept { return vs_.cend(); }
    };

    bool intersect_triangles(const Triangle& a, const Triangle& b) {}

}
