# HW3D — Triangles

### Input:
* An integer 0 < N < 1,000,000 from standard input, followed by
* N sets of points representing 3D triangles (each defined by 3 vertices in 3D space).

### Task:
Output the indices of all triangles that intersect with at least one other triangle in the set.

## How to build the project
```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cd build
make
```
## How to run tests
```
cd build
ctest
```

## Example
```
Input: <N> <\n> <x1> <y1> <z1> <x2> <y2> <z2> <x3> <y3> <z3> <\n> ...
Output: <Number of intersecting triangles>
```
