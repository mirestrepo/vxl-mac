#ifdef __GNUG__
#pragma implementation
#endif
//--------------------------------------------------------------
//
// Class : HomgOperator2D
//
// Modifications : see HomgOperator2D.h
//
//-----------------------------------------------------------------------------

#include <vcl/vcl_cassert.h>
#include <vcl/vcl_iostream.h>
#include <vcl/vcl_vector.h>

#include <vnl/vnl_math.h>
#include <vnl/vnl_matrix.h>
#include <vnl/algo/vnl_scatter_3x3.h>

#include <mvl/HomgLine2D.h>
#include <mvl/HomgPoint2D.h>
#include <mvl/HomgLineSeg2D.h>
#include <mvl/HomgOperator2D.h>

// @{ BASICS @}

//-----------------------------------------------------------------------------
// -- Cross product of two Homg2Ds
void HomgOperator2D::cross(const Homg2D& a, const Homg2D& b, Homg2D* a_cross_b)
{
  double x1 = a.get_x();
  double y1 = a.get_y();
  double w1 = a.get_w();
  
  double x2 = b.get_x();
  double y2 = b.get_y();
  double w2 = b.get_w();

  a_cross_b->set (y1 * w2 - w1 * y2,
		  w1 * x2 - x1 * w2,
		  x1 * y2 - y1 * x2);
}

//-----------------------------------------------------------------------------
// -- Dot product of two Homg2Ds
double HomgOperator2D::dot(const Homg2D& a, const Homg2D& b)
{
  double x1 = a.get_x();
  double y1 = a.get_y();
  double w1 = a.get_w();
  
  double x2 = b.get_x();
  double y2 = b.get_y();
  double w2 = b.get_w();

  return x1*x2 + y1*y2 + w1*w2;
}

//-----------------------------------------------------------------------------
// -- Normalize Homg2D to unit magnitude

void HomgOperator2D::unitize(Homg2D* a)
{
  double x = a->get_x();
  double y = a->get_y();
  double z = a->get_w();

  double norm = sqrt (vnl_math_sqr(x) + vnl_math_sqr(y) + vnl_math_sqr(z));
  
  if (norm == 0.0) {
    cerr << "HomgOperator2D::unitize() -- Zero length vector\n";
    return;
  }
  
  norm = 1.0/norm;
  a->set(x*norm, y*norm, z*norm);
}

// @{ DISTANCE MEASUREMENTS IN IMAGE COORDINATES @}

// -- Get the square of the 2D distance between the two points.
double HomgOperator2D::distance_squared (const HomgPoint2D& point1,
					 const HomgPoint2D& point2)
{
  double x1 = point1.get_x();
  double y1 = point1.get_y();
  double z1 = point1.get_w();

  double x2 = point2.get_x();
  double y2 = point2.get_y();
  double z2 = point2.get_w();

  if (z1 == 0 || z2 == 0) {
    cerr << "HomgOperator2D::distance_squared() -- point at infinity";
    return Homg::infinity;
  }

  double scale1 = 1.0/z1;
  double scale2 = 1.0/z2;
  
  return (vnl_math_sqr (x1 * scale1 - x2 * scale2) +
	  vnl_math_sqr (y1 * scale1 - y2 * scale2));
}

//-----------------------------------------------------------------------------
//
// -- Get the square of the perpendicular distance to a line.
// This is just the homogeneous form of the familiar 
// @{ $ \frac{a x + b y + c}{\sqrt{a^2+b^2}} $ @}:
// @{ \[ d = \frac{(l^\top p)}{p_z\sqrt{l_x^2 + l_y^2}} \] @}
// If either the point or the line are at infinity an error message is
// printed and Homg::infinity is returned.

double HomgOperator2D::perp_dist_squared (const HomgPoint2D& point, const HomgLine2D& line)
{
  if (line.check_infinity() || point.check_infinity()) {
    cerr << "HomgOperator2D::perp_dist_squared() -- line or point at infinity";
    return Homg::infinity;
  }
  
  double numerator = vnl_math_sqr (dot(line, point));
  double denominator = (vnl_math_sqr (line.get_x()) + vnl_math_sqr(line.get_y())) *
    vnl_math_sqr (point.get_w());

  return numerator / denominator;
}

// -- Return the distance of a line segment to a line.  This is defined as the maximum of the
// distances of the two endpoints to the line.
double HomgOperator2D::distance_squared(const HomgLineSeg2D& segment, const HomgLine2D& line)
{
  return vnl_math_max(perp_dist_squared(segment.get_point1(), line),
		   perp_dist_squared(segment.get_point2(), line));
}


// -- Return distance between line segments.
double HomgOperator2D::distance(const HomgLineSeg2D& ls, const HomgLineSeg2D& ll, double OVERLAP_THRESH)
{
  double norm = sqrt(ll.get_line().get_x()*ll.get_line().get_x()+ll.get_line().get_y()*ll.get_line().get_y());
  HomgLine2D lll;
  lll.set(ll.get_line().x()/norm,ll.get_line().y()/norm,ll.get_line().w()/norm);
  double dist1 = ls.get_point1().get_x()/ls.get_point1().get_w() * lll.get_x() + 
    ls.get_point1().get_y()/ls.get_point1().get_w() * lll.get_y() + lll.get_w(); 


  double dist2 = ls.get_point2().get_x()/ls.get_point2().get_w() * lll.get_x() + 
    ls.get_point2().get_y()/ls.get_point2().get_w() * lll.get_y() +  lll.get_w();

  //  cout << "dist 1 is " <<dist1 << " dist 2 is " <<dist2 << endl;

  double dist = (fabs(dist1) + fabs(dist2))/2;
   
  // compute overlap
  // if smaller than OVERLAP_THRESH then reject


  //project ls.point1 and point2 onto ll
   
  HomgPoint2D p1,p2,p3,p4;
  p1.set(ls.get_point1().get_x()/ls.get_point1().get_w()-dist1*(lll.get_x()),
	 ls.get_point1().get_y()/ls.get_point1().get_w()-dist1*(lll.get_y()),1);

  p2.set(ls.get_point2().get_x()/ls.get_point2().get_w()-dist2*(lll.get_x()),
	 ls.get_point2().get_y()/ls.get_point2().get_w()-dist2*(lll.get_y()),1);

  p3.set(ll.get_point1().get_x()/ll.get_point1().get_w(),
	 ll.get_point1().get_y()/ll.get_point1().get_w(),1);

  p4.set(ll.get_point2().get_x()/ll.get_point2().get_w(),
	 ll.get_point2().get_y()/ll.get_point2().get_w(),1);

  vnl_double_2 v1(p2.get_x() - p1.get_x(), p2.get_y() - p1.get_y());
  vnl_double_2 v2(p3.get_x() - p1.get_x(), p3.get_y() - p1.get_y());
  
  double r3 = v2(0)/v1(0);
   
  vnl_double_2 v3(p4.get_x() - p1.get_x(), p4.get_y() - p1.get_y());
   
  double r4 = v3(0)/v1(0);
   
  if (r3 > 1)
    r3 = 1;
  if (r3 < 0)
    r3 =0;
   
  if (r4 > 1)
    r4 = 1;
  if (r4 < 0)
    r4 =0;

  double r = fabs(r3-r4);

  r = r * sqrt(v1(0)*v1(0)+v1(1)*v1(1));
   
  if (r < OVERLAP_THRESH)
    dist = 1000000;
   
  return dist;
}


// -- Return the "Schmid distance" from a point to a line segment.  This is the
// distance to the closest point on the segment, be it endpoint or interior.
// UNTESTED.
double HomgOperator2D::distance_squared (const HomgLineSeg2D& lineseg, const HomgPoint2D& p)
{
  const HomgPoint2D& p1 = lineseg.get_point1();
  const HomgPoint2D& p2 = lineseg.get_point2();

  double p1x = p1[0] / p1[2];
  double p1y = p1[1] / p1[2];

  double p2x = p2[0] / p2[2];
  double p2y = p2[1] / p2[2];

  double dx = p2x - p1x;
  double dy = p2y - p1y;

  double l = sqrt(dx*dx + dy*dy);

  double px = p[0] / p[2];
  double py = p[1] / p[2];

  double d = ((px - p1x)*dx + (py - p1y)*dy);
  if (d < 0)
    return distance_squared(p, p1);
  if (d > l)
    return distance_squared(p, p2);

  return perp_dist_squared(p, lineseg);
}

// @{ ANGLES @}

//-----------------------------------------------------------------------------
// -- Get the anticlockwise angle between a line and the x axis.
double HomgOperator2D::line_angle(const HomgLine2D& line)
{
  return atan2 (line.get_y(), line.get_x());
}

//-----------------------------------------------------------------------------
// -- Get the 0 to pi/2 angle between two lines
double HomgOperator2D::abs_angle(const HomgLine2D& line1, const HomgLine2D& line2)
{
  double angle1 = line_angle (line1);
  double angle2 = line_angle (line2);

  double diff = vnl_math_abs(angle2 - angle1);
  
  if (diff > vnl_math::pi)
    diff -= vnl_math::pi;

  if (diff > vnl_math::pi/2)
    return vnl_math::pi - diff;
  else
    return diff;
}

//-----------------------------------------------------------------------------
//
// -- Get the angle between two lines.  Although homogeneous coordinates are
// only defined up to scale, here it is assumed that a line with homogeneous
// coordinates (m) is at 180 degrees to a line (-m), and this is why the term
// "oriented_line" is used.  However, the overall scale (apart from sign) is
// not significant.
// 

double HomgOperator2D::angle_between_oriented_lines (const HomgLine2D& line1,
						     const HomgLine2D& line2)
{
  double angle1 = line_angle (line1);
  double angle2 = line_angle (line2);

  double diff = angle2 - angle1;
  
  if (diff > vnl_math::pi)
    return diff - 2.0 * vnl_math::pi;
  
  if (diff < -vnl_math::pi)
    return diff + 2.0 * vnl_math::pi;
  
  return diff;
}

// @{ JOINS/INTERSECTIONS @}

//-----------------------------------------------------------------------------
//
// -- Get the line through two points (the cross-product).
// 

HomgLine2D HomgOperator2D::join (const HomgPoint2D& point1, const HomgPoint2D& point2)
{
  HomgLine2D answer;
  cross(point1, point2, &answer);
  return answer;
}

//-----------------------------------------------------------------------------
//
// -- Get the line through two points (the cross-product).  In this case, we assume
// that the points are oriented, and ensure the cross is computed with positive point
// omegas.
// 

HomgLine2D HomgOperator2D::join_oriented (const HomgPoint2D& point1, const HomgPoint2D& point2)
{
  double x1 = point1.get_x();
  double y1 = point1.get_y();
  double w1 = point1.get_w();
  bool s1 = w1 < 0;
  
  double x2 = point2.get_x();
  double y2 = point2.get_y();
  double w2 = point2.get_w();
  bool s2 = w2 < 0;

  if (s1 ^ s2)
    return HomgLine2D(-y1 * w2 + w1 * y2, -w1 * x2 + x1 * w2, -x1 * y2 + y1 * x2);
  else
    return HomgLine2D( y1 * w2 - w1 * y2,  w1 * x2 - x1 * w2,  x1 * y2 - y1 * x2);
}

//-----------------------------------------------------------------------------
//
// -- Get the intersection point of two lines (the cross-product).
// 

HomgPoint2D HomgOperator2D::intersection (const HomgLine2D& line1, const HomgLine2D& line2)
{
  HomgPoint2D answer;
  cross(line1, line2, &answer);
  return answer;
}

//-----------------------------------------------------------------------------
//
// -- @{ Get the perpendicular line to line which passes through point.
// Params are line $(a,b,c)$ and point $(x,y,1)$.
// Then the cross product of $(x,y,1)$ and the line's direction $(a,b,0)$,
// called $(p,q,r)$ satisfies
// 
//   $ap+bq=0$ (perpendicular condition) and
//   
//   $px+qy+r=0$ (incidence condition).
// @}

HomgLine2D HomgOperator2D::perp_line_through_point (const HomgLine2D& line,
						    const HomgPoint2D& point)
{
  HomgLine2D direction(line.get_x(), line.get_y(), 0);  
  HomgLine2D answer;
  cross(direction, point, &answer);
  unitize(&answer);
  return answer;
}

//-----------------------------------------------------------------------------
//
// -- Get the perpendicular projection of point onto line.
// 

HomgPoint2D HomgOperator2D::perp_projection (const HomgLine2D& line,
					     const HomgPoint2D& point)
{
  HomgLine2D perpline = perp_line_through_point (line, point);
  HomgPoint2D answer;
  cross(line, perpline, &answer);
  unitize(&answer);
  return answer;
}

// -- Return the midpoint of the line joining two homogeneous points
HomgPoint2D HomgOperator2D::midpoint (const HomgPoint2D& p1, const HomgPoint2D& p2)
{
  return p1 * (1/(2*p1[2])) + p2*(1/(2*p2[2]));
}

// @{ FITTING @}

// - Kanatani sect 2.2.2.
static vnl_vector<double> most_orthogonal_vector(const vcl_vector<HomgLine2D>& inpoints)
{
  vnl_scatter_3x3<double> scatter_matrix;

  for (unsigned i = 0; i < inpoints.size(); i++)
    scatter_matrix.add_outer_product(inpoints[i].get_vector());

  return scatter_matrix.minimum_eigenvector();
}

#include <vnl/algo/vnl_svd.h>

static vnl_vector<double> most_orthogonal_vector_svd(const vcl_vector<HomgLine2D>& lines)
{
  vnl_matrix<double> D(lines.size(), 3);

  for (unsigned i = 0; i < lines.size(); i++)
    D.set_row(i, lines[i].get_vector());
  
  vnl_svd<double> svd(D);
  cout << "[movrank " << svd.W() << "]";
  
  return svd.nullvector();
}

bool lines_to_point_use_svd = false;

// -- Intersect a set of 2D lines to find the least-square point of intersection.
// @{ This finds the point $\bf x$ that minimizes $\|\tt L \bf x\|$, where $\tt L$ is the matrix whose
// rows are the lines. The current implementation uses the vnl_scatter_3x3<double> class from
// Numerics to accumulate and compute the nullspace of $\tt L^\top \tt L$  @}
HomgPoint2D HomgOperator2D::lines_to_point(const vcl_vector<HomgLine2D>& lines)
{
  // ho_triveccam_aspect_lines_to_point
  assert(lines.size() >= 2);

  if (lines_to_point_use_svd)
    return most_orthogonal_vector_svd(lines);
  else
    return most_orthogonal_vector(lines);
}

// @{ MISCELLANEOUS @}
//
// -- Clip line to lineseg.
// The infinite line is clipped against the viewport with
// lower left corner (x0,y0) and upper right corner (x1,y1)

HomgLineSeg2D HomgOperator2D::clip_line_to_lineseg(const HomgLine2D& line,
						   double x0, double y0,
						   double x1, double y1)
{
  double nx = line.get_x();
  double ny = line.get_y();
  double nz = line.get_w();
  
  bool intersect_lr = vnl_math_abs(ny) > vnl_math_abs(nx);

  if (intersect_lr) {
    // Clip against verticals
    HomgPoint2D p1(x0 * ny, -(nz + x0 * nx), ny);
    HomgPoint2D p2(x1 * ny, -(nz + x1 * nx), ny);
    return HomgLineSeg2D(p1, p2);
  } else {
    HomgPoint2D p1(-(nz + y0 * ny), y0 * nx, nx);
    HomgPoint2D p2(-(nz + y1 * ny), y1 * nx, nx);
    return HomgLineSeg2D(p1, p2);
  }
}

double HomgOperator2D::perp_distance_squared (const HomgLine2D& line, const HomgPoint2D& point)
{
  cerr << "HomgOperator2D::perp_distance_squared should be replaced by perp_dist_squared\n";
  return perp_dist_squared(point, line);
}

//-----------------------------------------------------------------------------
// -- Calculates the crossratio of four collinear points p1, p2, p3 and p4.
// This number is projectively invariant, and it is the coordinate of p4
// in the reference frame where p2 is the origin (coordinate 0), p3 is
// the unity (coordinate 1) and p1 is the point at infinity.
// This cross ratio is often denoted as ((p1, p2; p3, p4)) (which also
// equals ((p3, p4; p1, p2)) or ((p2, p1; p4, p3)) or ((p4, p3; p2, p1)) )
// and is calculated as
//                      p1 - p3   p2 - p3      (p1-p3)(p2-p4)
//                      ------- : --------  =  --------------
//                      p1 - p4   p2 - p4      (p1-p4)(p2-p3)
//
// In principle, any single nonhomogeneous coordinate from the four points
// can be used as parameters for CrossRatio (but of course the same for all
// points). The most reliable answer will be obtained when the coordinate with
// the largest spacing is used, i.e., the one with smallest slope.
//
double HomgOperator2D::CrossRatio(const Homg2D& a, const Homg2D& b, const Homg2D& c, const Homg2D& d)
{
  double x1 = a.get_x(), y1 = a.get_y(), w1 = a.get_w();
  double x2 = b.get_x(), y2 = b.get_y(), w2 = b.get_w();
  double x3 = c.get_x(), y3 = c.get_y(), w3 = c.get_w();
  double x4 = d.get_x(), y4 = d.get_y(), w4 = d.get_w();
  double x = x1 - x2; if (x<0) x = -x; // assuming a != b ;-)
  double y = y1 - y2; if (y<0) y = -y;
  double n = (x>y) ? (x1*w3-x3*w1)*(x2*w4-x4*w2) : (y1*w3-y3*w1)*(y2*w4-y4*w2);
  double m = (x>y) ? (x1*w4-x4*w1)*(x2*w3-x3*w2) : (y1*w4-y4*w1)*(y2*w3-y3*w2);
  if (n == 0 && m == 0)
    cerr << "CrossRatio not defined: three of the given points coincide" << endl;
  return n/m;
}

// -- Conjugate point of three given colinear points.
// If cross ratio cr is given (default: -1), the generalized conjugate point
// returned is such that ((x1,x2;x3,answer)) = cr.
Homg2D HomgOperator2D::Conjugate(const Homg2D& a, const Homg2D& b, const Homg2D& c, double cr)
// Default for cr is -1.
{
  double x1 = a.get_x(), y1 = a.get_y(), w1 = a.get_w();
  double x2 = b.get_x(), y2 = b.get_y(), w2 = b.get_w();
  double x3 = c.get_x(), y3 = c.get_y(), w3 = c.get_w();
  double kx = x1*w3 - x3*w1, mx = x2*w3 - x3*w2, nx = kx*w2-cr*mx*w1;
  double ky = y1*w3 - y3*w1, my = y2*w3 - y3*w2, ny = ky*w2-cr*my*w1;
  return Homg2D((x2*kx-cr*x1*mx)*ny, (y2*ky-cr*y1*my)*nx, nx*ny);
}

