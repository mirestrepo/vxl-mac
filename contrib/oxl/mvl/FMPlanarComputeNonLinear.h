#ifndef FMPlanarComputeNonLinear_h_
#define FMPlanarComputeNonLinear_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME    FMPlanarComputeNonLinear - Nonlinear least squares planar fundamental matrix fit
// .LIBRARY MViewCompute
// .HEADER  MultiView Package
// .INCLUDE mvl/FMPlanarComputeNonLinear.h
// .FILE    FMPlanarComputeNonLinear.cxx
//
// .SECTION Description
//
//    FMPlanarComputeNonLinear fits a planar fundamental matrix to point matches
//    by minimizing the Luong-Faugeras [ECCV '92] error function
//    @{\[
//    E = \sum_{i=1}^n d^2({\bf x}'_i, {\tt F} {\bf x}_i) + d^2({\bf x}_i, {\tt F}^\top {\bf x}'_i)
//    \]@}
//    Minimization currently uses vnl_levenberg_marquardt with finite-difference
//    derivatives, and does not minimize a Huber function---all matches
//    are assumed to be inliers.
//
// .SECTION Author
//     Martin Armstrong, Oxford 21/11/96
//
// .SECTION Modifications:
//     <none yet>
//
//-----------------------------------------------------------------------------

#include <mvl/HomgMetric.h>
#include <mvl/FMatrixCompute.h>

class ImageMetric;
class PairMatchSetCorner;
class FMatrixPlanar;

class FMPlanarComputeNonLinear : public FMatrixCompute {
public:

  // Constructors/Destructors--------------------------------------------------
  FMPlanarComputeNonLinear(const ImageMetric*, const ImageMetric*, double outlier_threshold = 0);

  // Operations----------------------------------------------------------------

  // Computations--------------------------------------------------------------

  bool compute_planar(PairMatchSetCorner& matches, FMatrixPlanar* F);
  bool compute_planar(vcl_vector<HomgPoint2D>& points1, vcl_vector<HomgPoint2D>& points2, FMatrixPlanar* F);

  // FMatrixCompute virtuals
  bool compute(PairMatchSetCorner& matches, FMatrix* F);
  bool compute(vcl_vector<HomgPoint2D>& points1, vcl_vector<HomgPoint2D>& points2, FMatrix* F);

protected:
  // Data Members--------------------------------------------------------------
  double _outlier_distance_squared;

  const ImageMetric* _image_metric1;
  const ImageMetric* _image_metric2;
};

#endif // FMPlanarComputeNonLinear_h_
