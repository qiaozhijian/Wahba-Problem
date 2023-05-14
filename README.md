## Wahba's Problem

Given a set of unit vectors $\mathbf{v}_i$ and $\mathbf{w}_i$. Find a rotation matrix $R$ that minimizes the sum of the squares of the distances between the rotated $\mathbf{v}_i$ and the original $\mathbf{w}_i$ with a set of nonegative weights.

Wahba proposed a solution to this problem in 1962. Rotations are represented by the quaternion, which is subject to fewer constraints than the rotation matrix. The minimum solution to this problem requires matching two pairs of non-collinear vectors.

Demos
- TRIAD and QUEST proposed by Shuster [1]
- Pseudoinverse method. (This method is not recommended because it is not numerically stable)
- Procrustes method. 
## Run

```bash
mkdir build
cd build
cmake ..
make
# To run the solutions of TRIAD and QUEST
./shuster_solution
# To run the solutions of Pseudoinverse and Procrustes
./procrustes_sol
```

## Reference
[1] Shuster, M. D. (1981). Three-axis attitude determination from vector observations. Journal of Guidance, Control, and Dynamics, 4(1), 70-77.