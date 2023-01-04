## Wahba's Problem

Given a set of unit vectors $\mathbf{v}_i$ and $\mathbf{w}_i$. Find a rotation matrix $R$ that minimizes the sum of the squares of the distances between the rotated $\mathbf{v}_i$ and the original $\mathbf{w}_i$ with a set of nonegative weights.

Wahba proposed a solution to this problem in 1962. Rotations are represented by the quaternion, which is subject to fewer constraints than the rotation matrix.

In this repository, I implement the solutions, namely TRIAD and QUEST, proposed by Shuster.
## Run

```bash
mkdir build
cd build
cmake ..
make
./shuster_solution
```
