#include<iostream>
#include"triangles.hpp"

using namespace hw3d;

int main() {

    Triangle T1(Vector3d(0, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 0, 0));
    Triangle T2(Vector3d(0, 0, 0), Vector3d(0, 2, 0), Vector3d(2, 0, 0));

    std::cout << intersection_test_3d(T1, T2) << std::endl;

    return 0;
}
