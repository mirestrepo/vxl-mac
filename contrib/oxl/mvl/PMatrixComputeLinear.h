#ifndef PMatrixComputeLinear_h_
#define PMatrixComputeLinear_h_
//:
// \file
//
// PMatrixComputeLinear contains a linear method to calculate the P matrix
// from at least six 3D-to-2D point correspondences.  Precisely, given
// 3D points $\vec X_i$ and their images $\vec u_i$, we compute $P$ such that
// $P \vec X_i \sim \vec u_i$.

#include <mvl/PMatrixCompute.h>

class PMatrixComputeLinear : public PMatrixCompute {
public:
  bool compute(vcl_vector<HomgPoint2D> const& points1, vcl_vector<HomgPoint3D> const& points2, PMatrix *P);
};

#endif // PMatrixComputeLinear_h_
