#include<iostream>
#include"triangles.hpp"

using namespace hw3d;

int main() {

    Triangle T1(Vector3d(-3, 0, 0), Vector3d(0, -3, 0), Vector3d(0, 0, 2));
    Triangle T2(Vector3d(-5, 0, 0), Vector3d(0, -3.62, 0), Vector3d(0, 0, 4));

    std::cout << intersection_test_3d(T1, T2) << std::endl;

    return 0;
}
