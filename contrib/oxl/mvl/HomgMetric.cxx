#ifdef __GNUC__
#pragma implementation
#endif

#include "HomgMetric.h"

#include <vcl_iostream.h>
#include <vnl/vnl_identity_3x3.h>
#include <vnl/vnl_double_2.h>

#include <mvl/HomgPoint2D.h>
#include <mvl/HomgLine2D.h>
#include <mvl/HomgLineSeg2D.h>
#include <mvl/HomgOperator2D.h>
#include <mvl/ImageMetric.h>

#include <mvl/PMatrix.h>
#include <mvl/FMatrix.h>
#include <mvl/HMatrix2D.h>
#include <mvl/TriTensor.h>

// ** Constructors / Destructor

HomgMetric::HomgMetric(const ImageMetric* metric)
{
  _metric = (ImageMetric*)metric; // const violation!
}

HomgMetric::~HomgMetric()
{
  // _metric;
}

// -- Print HomgMetric to ostream.
ostream& HomgMetric::print(ostream & s) const
{
  if (_metric) 
    s << "[HomgMetric: " << *_metric << "]";
  else
    s << "[HomgMetric: Null ImageMetric]";

  return s;
}

// ** Conversion to/from homogeneous coordinates
vnl_double_2 HomgMetric::homg_to_image(const HomgPoint2D& p) const
{
  if (_metric) return _metric->homg_to_image(p);

  double x,y;
  p.get_nonhomogeneous(x, y);
  return vnl_double_2(x, y);
}

void HomgMetric::homg_to_image(const HomgPoint2D& homg, double* ix, double* iy) const
{
  if (_metric) {
    vnl_double_2 p = _metric->homg_to_image(homg);
    *ix = p[0];
    *iy = p[1];
  } else {
    homg.get_nonhomogeneous(*ix, *iy);
  }
}

HomgPoint2D HomgMetric::homg_to_imagehomg(const HomgPoint2D& p) const
{
  if (_metric) return _metric->homg_to_imagehomg(p);
  else return p;
}


HomgPoint2D HomgMetric::image_to_homg(const vnl_double_2& p) const
{
  if (_metric) return _metric->image_to_homg(p);
  else return HomgPoint2D(p.x(), p.y(), 1.0);
}

HomgPoint2D HomgMetric::image_to_homg(double x, double y) const
{
  if (_metric) return _metric->image_to_homg(x, y);
  else return HomgPoint2D(x, y, 1.0);
}  

HomgPoint2D HomgMetric::imagehomg_to_homg(const HomgPoint2D& p) const
{
  if (_metric) return _metric->imagehomg_to_homg(p);
  else return p;
}


HomgLine2D HomgMetric::homg_to_image_line(const HomgLine2D& l) const
{
  if (_metric) return _metric->homg_to_image_line(l);
  else return l;
}

HomgLine2D HomgMetric::image_to_homg_line(const HomgLine2D& l) const
{
  if (_metric) return _metric->image_to_homg_line(l);
  else return l;
}

HomgLineSeg2D HomgMetric::image_to_homg(const HomgLineSeg2D& l) const
{
  if (_metric) return _metric->image_to_homg(l);
  else return l;
}

HomgLineSeg2D HomgMetric::homg_to_image(const HomgLineSeg2D& l) const
{
  if (_metric) return _metric->homg_to_image(l);
  else return l;
}

// ** Measurements
double HomgMetric::perp_dist_squared(const HomgPoint2D& p, const HomgLine2D& l) const
{
  if (_metric) return _metric->perp_dist_squared(p, l);
  else return HomgOperator2D::perp_dist_squared(p, l);
}

HomgPoint2D HomgMetric::perp_projection(const HomgLine2D& l, const HomgPoint2D& p) const
{
  if (_metric) return _metric->perp_projection(l, p);
  else return HomgOperator2D::perp_projection(l, p);
}

double HomgMetric::distance_squared(const HomgPoint2D& p1, const HomgPoint2D& p2) const
{
  if (_metric) return _metric->distance_squared(p1, p2);
  else return HomgOperator2D::distance_squared(p1, p2);
}

double HomgMetric::distance_squared(const HomgLineSeg2D& segment, const HomgLine2D& line) const
{
  if (_metric) return _metric->distance_squared(segment, line);
  else return HomgOperator2D::distance_squared(segment, line);
}

bool HomgMetric::is_within_distance(const HomgPoint2D& p1, const HomgPoint2D& p2, double distance) const
{
  if (_metric) return _metric->is_within_distance(p1, p2, distance);
  else return HomgOperator2D::is_within_distance(p1, p2, distance);
}

// @{ SPEEDUPS AVAILABLE FOR CERTAIN SYSTEMS. @}

// -- Return true if the conditioner's action can be described by a planar homography.
bool HomgMetric::is_linear() const
{
  if (_metric) return _metric->is_linear();
  else return true;
}

static vnl_identity_3x3 I;

// -- Return the planar homography C s.t. C x converts x from conditioned to image coordinates.
const vnl_matrix<double>& HomgMetric::get_C() const
{
  if (_metric) return _metric->get_C();
  else return I;
}

// -- Return $C^{-1}$.
const vnl_matrix<double>& HomgMetric::get_C_inverse() const
{
  if (_metric) return _metric->get_C_inverse();
  else return I;
}

// @{ CONVERTING DISTANCES @}

// -- Return true if the metric is rotationally symmetric, i.e. can invert
// distances.
bool HomgMetric::can_invert_distance() const
{
  if (_metric) return _metric->can_invert_distance();
  else return true;
}

// -- Given that can_invert_distance is true, convert an image distance (in
// pixels) to a conditioned distance.
double HomgMetric::image_to_homg_distance(double image_distance) const
{
  if (_metric) return _metric->image_to_homg_distance(image_distance);
  else return image_distance;
}

// -- Convert a conditioned distance to an image distance.
double HomgMetric::homg_to_image_distance(double image_distance) const
{
  if (_metric) return _metric->homg_to_image_distance(image_distance);
  else return image_distance;
}

// -- As above, but for squared distances
double HomgMetric::image_to_homg_distance_sqr(double image_distance_2) const
{
  if (_metric)
    return vnl_math_sqr(_metric->image_to_homg_distance(sqrt(image_distance_2)));
  else
    return image_distance_2;
}

// -- 
double HomgMetric::homg_to_image_distance_sqr(double image_distance) const
{
  if (_metric)
    return vnl_math_sqr(_metric->homg_to_image_distance(sqrt(image_distance)));
  else
    return image_distance;
}

static ostream& warning(char const * fn) {
  return cerr << "HomgMetric::" << fn << "() WARNING: ";
}

// Static functions to condition/decondition image relations-----------------

// -- Convert a P matrix to image coords if possible.
PMatrix HomgMetric::homg_to_image_P(const PMatrix& P, const HomgMetric& c)
{
  if (!c.is_linear()) warning("homg_to_image_P") << "ImageMetric for image 1 is nonlinear\n";
  return PMatrix(c.get_C() * P.get_matrix());
}

PMatrix HomgMetric::image_to_homg_P(const PMatrix& P, const HomgMetric& c)
{
  if (!c.is_linear()) warning("homg_to_image_P") << "ImageMetric for image 1 is nonlinear\n";
  return PMatrix(c.get_C_inverse() * P.get_matrix());
}

// -- Decondition a fundamental matrix (convert it from conditioned to image
// coordinates) if possible.  If not possible, print a warning and do it
// approximately.
FMatrix HomgMetric::homg_to_image_F(const FMatrix& F, const HomgMetric& c1, const HomgMetric& c2)
{
  if (!c1.is_linear()) warning("homg_to_image_F") << "ImageMetric for image 1 is nonlinear\n";
  if (!c2.is_linear()) warning("homg_to_image_F") << "ImageMetric for image 2 is nonlinear\n";
  
  vnl_double_3x3 C1inv = c1.get_C_inverse();
  vnl_double_3x3 C2inv = c2.get_C_inverse();
  return FMatrix(C2inv.transpose() * F.get_matrix() * C1inv);
}

// -- Condition a fundamental matrix.
FMatrix HomgMetric::image_to_homg_F(const FMatrix& F, const HomgMetric& c1, const HomgMetric& c2)
{
  if (!c1.is_linear()) warning("image_to_homg_F") << "ImageMetric for image 1 is nonlinear\n";
  if (!c2.is_linear()) warning("image_to_homg_F") << "ImageMetric for image 2 is nonlinear\n";
  
  vnl_double_3x3 C1 = c1.get_C();
  vnl_double_3x3 C2 = c2.get_C();
  return FMatrix(C2.transpose() * F.get_matrix() * C1);
}

// -- Decondition a planar homogaphy.
HMatrix2D HomgMetric::homg_to_image_H(const HMatrix2D& H, const HomgMetric& c1, const HomgMetric& c2)
{
  if (!c1.is_linear()) warning("homg_to_image_H") << "ImageMetric for image 1 is nonlinear\n";
  if (!c2.is_linear()) warning("homg_to_image_H") << "ImageMetric for image 2 is nonlinear\n";
  
  vnl_double_3x3 C1i = c1.get_C_inverse();
  vnl_double_3x3 C2 = c2.get_C();
  return HMatrix2D(C2 * H.get_matrix() * C1i);
}

// -- Condition a planar homogaphy.
HMatrix2D HomgMetric::image_to_homg_H(const HMatrix2D& H, const HomgMetric& c1, const HomgMetric& c2)
{
  if (!c1.is_linear()) warning("image_to_homg_H") << "ImageMetric for image 1 is nonlinear\n";
  if (!c2.is_linear()) warning("image_to_homg_H") << "ImageMetric for image 2 is nonlinear\n";
  
  vnl_double_3x3 C1 = c1.get_C();
  vnl_double_3x3 C2i = c2.get_C_inverse();
  return HMatrix2D(C2i * H.get_matrix() * C1);
}

// -- Decondition a trifocal tensor.
TriTensor HomgMetric::homg_to_image_T(const TriTensor& T, const HomgMetric& c1, const HomgMetric& c2, const HomgMetric& c3)
{
  // Need line conditioners, so it's inverse transpose
  vnl_double_3x3 C1i = c1.get_C_inverse();
  vnl_double_3x3 C2 = c2.get_C();
  vnl_double_3x3 C3 = c3.get_C();
  
  return T.decondition(C1i.transpose(), C2.transpose(), C3.transpose());
}

// -- Condition a trifocal tensor.
TriTensor HomgMetric::image_to_homg_T(const TriTensor& T, const HomgMetric& c1, const HomgMetric& c2, const HomgMetric& c3)
{
  // Need line conditioners, so it's inverse transpose
  vnl_double_3x3 C1 = c1.get_C();
  vnl_double_3x3 C2i = c2.get_C_inverse();
  vnl_double_3x3 C3i = c3.get_C_inverse();
  
  return T.decondition(C1.transpose(), C2i.transpose(), C3i.transpose());
}
