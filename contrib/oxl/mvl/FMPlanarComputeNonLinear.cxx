#ifdef __GNUC__
#pragma implementation
#endif

#include <mvl/PairSetCorner.h>
#include <mvl/FMatrixPlanar.h>
#include <mvl/FMPlanarComputeNonLinear.h>
#include <mvl/FMPlanarNonLinFun.h>

// -- Constructor.  The parameter outlier_threshold is not currently used, but
// may be in future if this is converted to a Huber function.
FMPlanarComputeNonLinear::FMPlanarComputeNonLinear(const ImageMetric* image_metric1, const ImageMetric* image_metric2, double outlier_threshold):
  FMatrixCompute()
{
  _image_metric1 = image_metric1;
  _image_metric2 = image_metric2;
  _outlier_distance_squared = outlier_threshold * outlier_threshold;
}

// -- Compute from given PairMatchSetCorner
bool FMPlanarComputeNonLinear::compute_planar(PairMatchSetCorner& matches, FMatrixPlanar* F)
{
  PairSetCorner inliers(matches);
  return compute_planar(inliers.points1, inliers.points2, F);
}

// -- Compute from given pair of vcl_vector<HomgPoint2D>
bool FMPlanarComputeNonLinear::compute_planar(vcl_vector<HomgPoint2D>& points1,
					      vcl_vector<HomgPoint2D>& points2, FMatrixPlanar* F)
{
  cout << "FMPlanarComputeNonLinear: Fitting planar-motion F matrix [e1]_x [l]_x [e2]_x\n";
  FMPlanarNonLinFun computor(_image_metric1, _image_metric2, _outlier_distance_squared, points1, points2);
  return computor.compute(F);
}

bool FMPlanarComputeNonLinear::compute(PairMatchSetCorner& matches, FMatrix* F)
{
  FMatrixPlanar fplanar;
  fplanar.init(*F);
  if (!compute_planar(matches, &fplanar))
    return false;
  
  // Slice Fplanar into F
  *F = fplanar;
  return true;
}

bool FMPlanarComputeNonLinear::compute(vcl_vector<HomgPoint2D>& points1, vcl_vector<HomgPoint2D>& points2, FMatrix* F)
{
  FMatrixPlanar fplanar;
  fplanar.init(*F);
  if (!compute_planar(points1, points2, &fplanar))
    return false;
  
  // Slice Fplanar into F
  *F = fplanar;
  return true;
}

