// This is vxl/vgl/algo/vgl_homg_operators_2d.txx
#ifndef vgl_homg_operators_2d_txx_
#define vgl_homg_operators_2d_txx_

#include <vcl_cassert.h>
#include <vcl_iostream.h>
#include <vcl_cmath.h> // for vcl_sqrt()

#include "vgl_homg_operators_2d.h"
#include <vgl/vgl_homg_line_2d.h>
#include <vgl/vgl_homg_point_2d.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_conic.h>
#include <vgl/vgl_box_2d.h>

#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_math.h>
#include <vnl/algo/vnl_scatter_3x3.h> // used in most_orthogonal_vector()
#include <vnl/vnl_numeric_limits.h> // for infinity
#include <vnl/algo/vnl_real_eigensystem.h> // used for conic intersection
#include <vnl/vnl_diag_matrix.h>  // used for conic intersection

//-----------------------------------------------------------------------------

template <class T>
inline static vgl_homg_line_2d<T> cross(const vgl_homg_point_2d<T>& p1,
                                        const vgl_homg_point_2d<T>& p2)
{
  return vgl_homg_line_2d<T> (p1.y() * p2.w() - p1.w() * p2.y(),
                              p1.w() * p2.x() - p1.x() * p2.w(),
                              p1.x() * p2.y() - p1.y() * p2.x());
}

template <class T>
inline static vgl_homg_point_2d<T> cross(const vgl_homg_line_2d<T>& l1,
                                         const vgl_homg_line_2d<T>& l2)
{
  return vgl_homg_point_2d<T> (l1.b() * l2.c() - l1.c() * l2.b(),
                               l1.c() * l2.a() - l1.a() * l2.c(),
                               l1.a() * l2.b() - l1.b() * l2.a());
}

template <class T>
inline static double dot(vgl_homg_line_2d<T> const& l,
                         vgl_homg_point_2d<T> const& p)
{
  return l.a()*p.x() + l.b()*p.y() + l.c()*p.w();
}

//-----------------------------------------------------------------------------

template <class T>
vnl_vector<T> vgl_homg_operators_2d<T>::get_vector(vgl_homg_point_2d<T> const& p)
{
  // make a vnl_vector for the point p

  vnl_vector<T> v(3);
  v.put(0,p.x());
  v.put(1,p.y());
  v.put(2,p.w());

  return v;
}

template <class T>
vnl_vector<T> vgl_homg_operators_2d<T>::get_vector(vgl_homg_line_2d<T> const& l)
{
  // make a vnl_vector for the line l

  vnl_vector<T> v(3);
  v.put(0,l.a());
  v.put(1,l.b());
  v.put(2,l.c());

  return v;
}

template <class T>
vnl_vector<T> vgl_homg_operators_2d<T>::get_vector(vgl_conic<T> const& c)
{
  // make a vnl_vector for the conic c

  vnl_vector<T> v(6);
  v.put(0,c.a());
  v.put(1,c.b());
  v.put(2,c.c());
  v.put(3,c.d());
  v.put(4,c.e());
  v.put(5,c.f());

  return v;
}

//-----------------------------------------------------------------------------
//: Normalize vgl_homg_point_2d<T> to unit magnitude

template <class T>
void vgl_homg_operators_2d<T>::unitize(vgl_homg_point_2d<T>& a)
{
  double norm = vcl_sqrt (a.x()*a.x() + a.y()*a.y() + a.w()*a.w());

  if (norm == 0.0) {
    vcl_cerr<< "vgl_homg_operators_2d<T>::unitize() -- Zero length vector\n";
    return;
  }
  norm = 1.0/norm;
  a.set(a.x()*norm, a.y()*norm, a.w()*norm);
}

//  DISTANCE MEASUREMENTS IN EUCLIDEAN COORDINATES

//: Get the square of the 2D distance between the two points.
template <class T>
double
vgl_homg_operators_2d<T>::distance_squared(
                                  const vgl_homg_point_2d<T>& p1,
                                  const vgl_homg_point_2d<T>& p2)
{
  if (p1 == p2) return 0.0; // quick return if possible

  if (p1.w() == 0 || p2.w() == 0) {
    vcl_cerr << "vgl_homg_operators_2d<T>::distance_squared() -- point at infinity\n";
    return vnl_numeric_limits<T>::infinity();
  }

  double scale1 = 1.0/p1.w();
  double scale2 = 1.0/p2.w();

  return (vnl_math_sqr (p1.x() * scale1 - p2.x() * scale2) +
          vnl_math_sqr (p1.y() * scale1 - p2.y() * scale2));
}

//: Get the square of the perpendicular distance to a line.
// This is just the homogeneous form of the familiar
// \f$ \frac{a x + b y + c}{\sqrt{a^2+b^2}} \f$ :
// \[ d = \frac{(l^\top p)}{p_z\sqrt{l_x^2 + l_y^2}} \]
// If either the point or the line are at infinity an error message is
// printed and Homg::infinity is returned.

template <class T>
double
vgl_homg_operators_2d<T>::perp_dist_squared(
                                  const vgl_homg_point_2d<T>& point,
                                  const vgl_homg_line_2d<T>& line)
{
  if ((line.a()==0 && line.b()== 0) || point.w()==0) {
    vcl_cerr << "vgl_homg_operators_2d<T>::perp_dist_squared() -- line or point at infinity";
    // return Homg::infinity;
    return 10000000; // TODO make an infinity for homg operators
  }

  double numerator = vnl_math_sqr (dot(line, point));
  if (numerator == 0) return 0.0; // efficiency
  double denominator = (vnl_math_sqr (line.a()) + vnl_math_sqr(line.b()))
                       * vnl_math_sqr (point.w());

  return numerator / denominator;
}


//  ANGLES

//-----------------------------------------------------------------------------
//: Get the anticlockwise angle between a line and the x axis.
template <class T>
double
vgl_homg_operators_2d<T>::line_angle(const vgl_homg_line_2d<T>& line)
{
  return vcl_atan2 (line.b(), line.a());
}

//-----------------------------------------------------------------------------
//: Get the 0 to pi/2 angle between two lines
template <class T>
double
vgl_homg_operators_2d<T>::abs_angle(const vgl_homg_line_2d<T>& line1,
                                    const vgl_homg_line_2d<T>& line2)
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
//: Get the angle between two lines.  Although homogeneous coordinates are
// only defined up to scale, here it is assumed that a line with homogeneous
// coordinates (m) is at 180 degrees to a line (-m), and this is why the term
// "oriented_line" is used.  However, the overall scale (apart from sign) is
// not significant.
//

template <class T>
double
vgl_homg_operators_2d<T>::angle_between_oriented_lines(
                                  const vgl_homg_line_2d<T>& line1,
                                  const vgl_homg_line_2d<T>& line2)
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

//  JOINS/INTERSECTIONS 

//-----------------------------------------------------------------------------
//
//: Get the line through two points (the cross-product).
//

template <class T>
vgl_homg_line_2d<T>
vgl_homg_operators_2d<T>::join (const vgl_homg_point_2d<T>& p1,
                                const vgl_homg_point_2d<T>& p2)
{
  return cross(p1,p2);
}

//-----------------------------------------------------------------------------
//
//: Get the line through two points (the cross-product).  In this case, we assume
// that the points are oriented, and ensure the cross is computed with positive point
// omegas.
//

template <class T>
vgl_homg_line_2d<T>
vgl_homg_operators_2d<T>::join_oriented(const vgl_homg_point_2d<T>&p1,
                                        const vgl_homg_point_2d<T>&p2)
{
  int s1 = p1.w() < 0;
  int s2 = p2.w() < 0;

  if (s1 ^ s2)
    return cross(p2,p1);
  else
    return cross(p1,p2);
}

//-----------------------------------------------------------------------------
//
//: Get the intersection point of two lines (the cross-product).
//

template <class T>
vgl_homg_point_2d<T>
vgl_homg_operators_2d<T>::intersection ( const vgl_homg_line_2d<T>& l1,
                                         const vgl_homg_line_2d<T>& l2)
{
  return cross(l1,l2);
}

//-----------------------------------------------------------------------------
//
//: Get the perpendicular line to line which passes through point.
// Params are line \f$(a,b,c)\f$ and point \f$(x,y,1)\f$.
// Then the cross product of \f$(x,y,1)\f$ and the line's direction \f$(a,b,0)\f$,
// called \f$(p,q,r)\f$ satisfies
//
//   \f$ap+bq=0\f$ (perpendicular condition) and
//
//   \f$px+qy+r=0\f$ (incidence condition).


template <class T>
vgl_homg_line_2d<T>
vgl_homg_operators_2d<T>::perp_line_through_point ( const vgl_homg_line_2d<T>& l,
                                                    const vgl_homg_point_2d<T>& p)
{
  vgl_homg_point_2d<T> d(l.a(), l.b(), 0);
  return cross(d, p);
}

//-----------------------------------------------------------------------------
//
//: Get the perpendicular projection of point onto line.
//

template <class T>
vgl_homg_point_2d<T> vgl_homg_operators_2d<T>::perp_projection(
                                  const vgl_homg_line_2d<T>& line,
                                  const vgl_homg_point_2d<T>& point)
{
  vgl_homg_line_2d<T> perpline = perp_line_through_point (line, point);
  vgl_homg_point_2d<T> answer = cross(line, perpline);
  unitize(answer);
  return answer;
}

//: Return the midpoint of the line joining two homogeneous points
template <class T>
vgl_homg_point_2d<T>
vgl_homg_operators_2d<T>::midpoint( const vgl_homg_point_2d<T>& p1,
                                    const vgl_homg_point_2d<T>& p2)
{
  // return p1 * (1/(2*p1[2])) + p2*(1/(2*p2[2]));

  double x = p1.x() /(2*p1.w()) + p2.x() / (2*p2.w());
  double y = p1.y() /(2*p1.w()) + p2.y() / (2*p2.w());
  double w = p1.w() /(2*p1.w()) + p2.w() / (2*p2.w());

  return vgl_homg_point_2d<T>(x,y,w);
}

//  FITTING

// - Kanatani sect 2.2.2.
template <class T>
vnl_vector<T>
vgl_homg_operators_2d<T>::most_orthogonal_vector(const vcl_list<vgl_homg_line_2d<T> >& inpoints)
{
  vnl_scatter_3x3<T> scatter_matrix;

  for (vcl_list<vgl_homg_line_2d<T> >::const_iterator i = inpoints.begin();
       i != inpoints.end(); ++i)
    scatter_matrix.add_outer_product(get_vector(*i));

  return scatter_matrix.minimum_eigenvector();
}

#include <vnl/algo/vnl_svd.h>

template <class T>
vnl_vector<T>
vgl_homg_operators_2d<T>::most_orthogonal_vector_svd(const vcl_list<vgl_homg_line_2d<T> >& lines)
{
  vnl_matrix<T> D(lines.size(), 3);

  vcl_list<vgl_homg_line_2d<T> >::const_iterator i = lines.begin();
  for (unsigned j = 0; i != lines.end(); ++i,++j)
    D.set_row(j, get_vector(*i));

  vnl_svd<T> svd(D);
#ifdef DEBUG
  vcl_cout << "[movrank " << svd.W() << "]";
#endif

  return svd.nullvector();
}

//: Intersect a set of 2D lines to find the least-square point of intersection.
// This finds the point $\bf x$ that minimizes $\|\tt L \bf x\|$, where $\tt L$ is the matrix whose
// rows are the lines. The current implementation uses the Scatter3x3 class from
// Numerics to accumulate and compute the nullspace of $\tt L^\top \tt L$.
template <class T>
vgl_homg_point_2d<T>
vgl_homg_operators_2d<T>::lines_to_point(const vcl_list<vgl_homg_line_2d<T> >& lines)
{
  // ho_triveccam_aspect_lines_to_point
  assert(lines.size() >= 2);

#ifdef VGL_HOMG_OPERATORS_2D_LINES_TO_POINT_USE_SVD
  vnl_vector<T> mov = most_orthogonal_vector_svd(lines);
#else
  vnl_vector<T> mov = most_orthogonal_vector(lines);
#endif
  return vgl_homg_point_2d<T>(mov[0], mov[1], mov[2]);
}

//  MISCELLANEOUS 


template <class T>
double vgl_homg_operators_2d<T>::perp_distance_squared(const vgl_homg_line_2d<T>& line,
                                                       const vgl_homg_point_2d<T>& point)
{
  vcl_cerr << "vgl_homg_operators_2d<T>::perp_distance_squared should be replaced by perp_dist_squared\n";
  return perp_dist_squared(point, line);
}

//-----------------------------------------------------------------------------
//: Calculates the crossratio of four collinear points p1, p2, p3 and p4.
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
// can be used as parameters for cross_ratio (but of course the same for all
// points). The most reliable answer will be obtained when the coordinate with
// the largest spacing is used, i.e., the one with smallest slope.
//
template <class T>
double vgl_homg_operators_2d<T>::cross_ratio(const vgl_homg_point_2d<T>& a,
                                             const vgl_homg_point_2d<T>& b,
                                             const vgl_homg_point_2d<T>& c,
                                             const vgl_homg_point_2d<T>& d)
{
  double x1 = a.x(), y1 = a.y(), w1 = a.w();
  double x2 = b.x(), y2 = b.y(), w2 = b.w();
  double x3 = c.x(), y3 = c.y(), w3 = c.w();
  double x4 = d.x(), y4 = d.y(), w4 = d.w();
  double x = x1 - x2; if (x<0) x = -x; // assuming a != b ;-)
  double y = y1 - y2; if (y<0) y = -y;
  double n = (x>y) ? (x1*w3-x3*w1)*(x2*w4-x4*w2) : (y1*w3-y3*w1)*(y2*w4-y4*w2);
  double m = (x>y) ? (x1*w4-x4*w1)*(x2*w3-x3*w2) : (y1*w4-y4*w1)*(y2*w3-y3*w2);
  if (n == 0 && m == 0)
    vcl_cerr << "cross ratio not defined: three of the given points coincide\n";
  return n/m;
}

//: Conjugate point of three given colinear points.
// If cross ratio cr is given (default: -1), the generalized conjugate point
// returned is such that ((x1,x2;x3,answer)) = cr.
template <class T>
vgl_homg_point_2d<T> vgl_homg_operators_2d<T>::conjugate(
                                  const vgl_homg_point_2d<T>& a,
                                  const vgl_homg_point_2d<T>& b,
                                  const vgl_homg_point_2d<T>& c,
                                  double cr)
// Default for cr is -1.
{
  double x1 = a.x(), y1 = a.y(), w1 = a.w();
  double x2 = b.x(), y2 = b.y(), w2 = b.w();
  double x3 = c.x(), y3 = c.y(), w3 = c.w();
  double kx = x1*w3 - x3*w1, mx = x2*w3 - x3*w2, nx = kx*w2-cr*mx*w1;
  double ky = y1*w3 - y3*w1, my = y2*w3 - y3*w2, ny = ky*w2-cr*my*w1;
  return vgl_homg_point_2d<T>((x2*kx-cr*x1*mx)*ny,(y2*ky-cr*y1*my)*nx,nx*ny);
}

//: returns the vgl_conic which has the given matrix as its matrix
// \verbatim
// [A,B,C,D,E,F] <-   [ A  B/2 D/2 ]
//                    [ B/2 C  E/2 ]
//                    [ D/2 E/2 F  ]
// \endverbatim

template <class T>
vgl_conic<T>
vgl_homg_operators_2d<T>::vgl_conic_from_matrix(vnl_matrix<T> const& mat)
{
  assert ( mat.rows() == 3 && mat.columns() == 3 );
  return vgl_conic<T>(mat[0][0], mat[1][0]+mat[0][1], mat[1][1], mat[0][2]+mat[2][0], mat[1][2]+mat[2][1], mat[2][2]);
}

//: returns 3x3 matrix containing conic coefficients:
// \verbatim
// [A,B,C,D,E,F] ->   [ A  B/2 D/2 ]
//                    [ B/2 C  E/2 ]
//                    [ D/2 E/2 F  ]
// \endverbatim
//
template <class T>
vnl_matrix<T>
vgl_homg_operators_2d<T>::matrix_from_conic(vgl_conic<T> const& c)
{
  vnl_matrix<T> mat = vnl_matrix<T>(3,3);
  T A = c.a(), B = c.b()/2, C = c.c(), D = c.d()/2, E = c.e()/2, F = c.f();

  mat[0][0] = A; mat[0][1] = B; mat[0][2] = D;
  mat[1][0] = B; mat[1][1] = C; mat[1][2] = E;
  mat[2][0] = D; mat[2][1] = E; mat[2][2] = F;

  return mat;
}


//-------------------------------------------------------------------------
//: returns 3x3 matrix containing conic coefficients of dual conic, i.e.,
// the inverse matrix (up to a scale factor) of the conic matrix.
//
template <class T>
vnl_matrix<T>
vgl_homg_operators_2d<T>::matrix_from_dual_conic(vgl_conic<T> const& c)
{
  vnl_matrix<T> mat = vnl_matrix<T>(3,3);
  T A = c.a(), B = c.b()/2, C = c.c(), D = c.d()/2, E = c.e()/2, F = c.f();

  mat[0][0] = C*F-E*E; mat[0][1] = E*D-B*F; mat[0][2] = B*E-C*D;
  mat[1][0] = E*D-B*F; mat[1][1] = A*F-D*D; mat[1][2] = B*D-A*E;
  mat[2][0] = B*E-C*D; mat[2][1] = B*D-A*E; mat[2][2] = A*C-B*B;

  return mat;
}

// This function is called from within intersection(vgl_conic<T>,vgl_conic<T>).
// The two conics passed to this function MUST NOT be degenerate!
template <class T>
vcl_list<vgl_homg_point_2d<T> >
vgl_homg_operators_2d<T>::do_intersect(vgl_conic<T> const& c1,
                                       vgl_conic<T> const& c2)
{
  T A=c1.a(),B=c1.b(),C=c1.c(),D=c1.d(),E=c1.e(),F=c1.f();
  T a=c2.a(),b=c2.b(),c=c2.c(),d=c2.d(),e=c2.e(),f=c2.f();
  T ab=a*B-A*b, ac=a*C-A*c, ad=a*D-A*d, ae=a*E-A*e, af=a*F-A*f, BD=b*D+B*d;

  vnl_vector<T> coef(5,0);
  coef(0) = ac*ac-ab*(b*C-B*c);
  coef(1) = 2*ac*ae-ab*(b*E-B*e)-BD*(a*C+A*c)+2*A*b*C*d+2*a*B*c*D;
  coef(2) = ae*ae-ab*(b*F-B*f)+ad*(c*D-C*d)-BD*(a*E+A*e)+2*a*B*e*D+2*A*b*E*d+2*ac*af;
  coef(3) = 2*ae*af-ad*(d*E-D*e)-BD*(a*F+A*f)+2*A*b*d*F+2*a*B*D*f;
  coef(4) = af*af-ad*(d*F-D*f);

  // Solutions of the fourth order equation
  //   4      3      2
  //  x  +  bx  +  cx  +  dx  +  e  =  0
  // are the eigenvalues of the matrix
  // [ -b   -c   -d   -e ]
  // [  1    0    0    0 ]
  // [  0    1    0    0 ]
  // [  0    0    1    0 ]

  if (coef(0) == 0 && coef(1) == 0) {
    T dis = coef(3)*coef(3)-4*coef(2)*coef(4); // discriminant
    if (dis < 0) return vcl_list<vgl_homg_point_2d<T> >(); // no real solutions.
    T y;
    if (coef(2) == 0) dis=0, y=-coef(4)/coef(3);
    else              y = -0.5*coef(3)/coef(2);
    T x = -(y*y*ac+y*ae+af)/(y*ab+ad);
    if (dis == 0) {return vcl_list<vgl_homg_point_2d<T> >(1,vgl_homg_point_2d<T>(x,y,1));}
    dis = 0.5 * vcl_sqrt(dis) / coef(2);
    vcl_list<vgl_homg_point_2d<T> > solutions;
    y -= dis; x = -(y*y*ac+y*ae+af)/(y*ab+ad);
    solutions.push_back(vgl_homg_point_2d<T>(x,y,1));
    y += 2*dis; x = -(y*y*ac+y*ae+af)/(y*ab+ad);
    solutions.push_back(vgl_homg_point_2d<T>(x,y,1));
    return solutions;
  }
  if (coef(0) == 0) {
    coef /= -coef(1);
    double data[]={coef(2),coef(3),coef(4), 1,0,0, 0,1,0};
    vnl_matrix<double> M(data,3,3);
    vnl_real_eigensystem eig(M);
    vnl_diag_matrix<vcl_complex<double> >  polysolutions = eig.D ;
    vcl_list<vgl_homg_point_2d<T> > solutions;
    for (int i=0;i<3;i++)
      if(vcl_abs(vcl_imag(polysolutions(i))) < 1e-7) {// only want the real solutions
        double y = vcl_real(polysolutions(i));
        double x = -(y*y*ac+y*ae+af)/(y*ab+ad);
        solutions.push_back(vgl_homg_point_2d<T>(x,y,1));
      }
    return solutions;
  }

  coef /= -coef(0);
  double data[]={coef(1),coef(2),coef(3),coef(4), 1,0,0,0, 0,1,0,0, 0,0,1,0};
  vnl_matrix<double> M(data,4,4);
  vnl_real_eigensystem eig(M);

  vnl_diag_matrix<vcl_complex<double> >  polysolutions = eig.D ;

  // These are only the solutions of the fourth order equation.
  // The solutions of the intersection of the two conics are:

  vcl_list<vgl_homg_point_2d<T> > solutions;

  for (int i=0;i<4;i++)
    if(vcl_abs(vcl_imag(polysolutions(i))) < 1e-7) { // only want the real solutions
      double y = vcl_real(polysolutions(i));
      double x = -(y*y*ac+y*ae+af)/(y*ab+ad);
      solutions.push_back(vgl_homg_point_2d<T>(x,y,1));
    }
  return solutions;
}

// This function is called from within intersection(vgl_conic<T>,vgl_homg_line_2d<T>).
// The conic passed to this function MUST NOT be degenerate!
template <class T>
vcl_list<vgl_homg_point_2d<T> >
vgl_homg_operators_2d<T>::do_intersect(vgl_conic<T> const& q,
                                       vgl_homg_line_2d<T> const& l)
{
  T A=q.a(), B=q.b(), C=q.c(), D=q.d(), E=q.e(), F=q.f();
  T a=l.a(), b=l.b(), c=l.c();

  if (a==0 && b==0) { // line at infinity
    if (A==0)
      return vcl_list<vgl_homg_point_2d<T> >(2,vgl_homg_point_2d<T>(1,0,0));
    T d = B*B-4*A*C; // discriminant
    if (d < 0) return vcl_list<vgl_homg_point_2d<T> >(); // no solutions
    if (d == 0)
      return vcl_list<vgl_homg_point_2d<T> >(2,vgl_homg_point_2d<T>(-B,2*A,0));
    d = vcl_sqrt(d);
    vcl_list<vgl_homg_point_2d<T> > v(1, vgl_homg_point_2d<T>(-B+d, 2*A, 0));
    v.push_back(vgl_homg_point_2d<T>(-B-d, 2*A, 0));
    return v;
  }
  if (a==0) { // write y in terms of w and solve for (x,w)
    T y = -c/b; B = B*y+D;
    T d = B*B-4*A*(C*y*y+E*y+F); // discriminant
    if (d < 0) return vcl_list<vgl_homg_point_2d<T> >(); // no solutions
    if (d == 0 && A == 0)
      return vcl_list<vgl_homg_point_2d<T> >(2,vgl_homg_point_2d<T>(1,0,0));
    if (d == 0)
      return vcl_list<vgl_homg_point_2d<T> >(2,vgl_homg_point_2d<T>(-B,y*2*A,2*A));
    if (A == 0) {
      vcl_list<vgl_homg_point_2d<T> > v(1, vgl_homg_point_2d<T>(1,0,0));
      v.push_back(vgl_homg_point_2d<T>(C*y*y+E*y+F, -y*B, -B));
      return v;
    }
    d = vcl_sqrt(d);
    vcl_list<vgl_homg_point_2d<T> > v(1, vgl_homg_point_2d<T>(-B+d, y*2*A, 2*A));
    v.push_back(vgl_homg_point_2d<T>(-B-d, y*2*A, 2*A));
    return v;
  }
  b /= -a; c /= -a; // now x = b*y+c*w.
  T AA = A*b*b+B*b+C;
  B = 2*A*b*c+B*c+D*b+E;
  T d = B*B-4*AA*(A*c*c+D*c+F); // discriminant
  if (d < 0) return vcl_list<vgl_homg_point_2d<T> >(); // no solutions
  if (d == 0 && AA == 0)
    return vcl_list<vgl_homg_point_2d<T> >(2,vgl_homg_point_2d<T>(b,1,0));
  if (d == 0)
    return vcl_list<vgl_homg_point_2d<T> >(2,vgl_homg_point_2d<T>(c*2*AA-b*B,-B,2*AA));
  if (AA == 0) {
    vcl_list<vgl_homg_point_2d<T> > v(1, vgl_homg_point_2d<T>(b,1,0));
    v.push_back(vgl_homg_point_2d<T>(b*A*c*c+b*D*c+b*F-c*B, A*c*c+D*c+F, -B));
    return v;
  }
  d = vcl_sqrt(d);
  vcl_list<vgl_homg_point_2d<T> > v(1, vgl_homg_point_2d<T>(c*2*AA-b*B+b*d, -B+d, 2*AA));
  v.push_back(vgl_homg_point_2d<T>(c*2*AA-b*B-b*d, -B-d, 2*AA));
  return v;
}

//: Return the (real) intersection points of a conic and a line.
template <class T>
vcl_list<vgl_homg_point_2d<T> >
vgl_homg_operators_2d<T>::intersection(vgl_conic<T> const& c,
                                       vgl_homg_line_2d<T> const& l)
{
  if (c.type()==vgl_conic<T>::no_type
   || c.type()==vgl_conic<T>::complex_parallel_lines
   || c.type()==vgl_conic<T>::complex_intersecting_lines
   || c.type()==vgl_conic<T>::imaginary_ellipse
   || c.type()==vgl_conic<T>::imaginary_circle)
    return vcl_list<vgl_homg_point_2d<T> >(); // empty list
  // let's hope the intersection point of the two complex lines is not on the line..

  if (c.type() == vgl_conic<T>::coincident_lines) {
    vgl_homg_point_2d<T> p = intersection(l, c.components().front());
    vcl_list<vgl_homg_point_2d<T> > list(2, p); // intersection is *two* coincident points
    return list;
  }

  if (c.type() == vgl_conic<T>::real_intersecting_lines || c.type() == vgl_conic<T>::real_parallel_lines) {
    vcl_list<vgl_homg_point_2d<T> > list;
    list.push_back(intersection(l, c.components().front()));
    list.push_back(intersection(l, c.components().back()));
    return list;
  }
  return do_intersect(c, l);
}

//: Return the (real, finite) intersection points of two conics.
// The returned points have protection level 0.
template <class T>
vcl_list<vgl_homg_point_2d<T> >
vgl_homg_operators_2d<T>::intersection(vgl_conic<T> const& c1, vgl_conic<T> const& c2)
{
  if ((c1.type()==vgl_conic<T>::complex_parallel_lines ||
       c1.type()==vgl_conic<T>::complex_intersecting_lines)
      && c2.contains(c1.centre()))
    return vcl_list<vgl_homg_point_2d<T> >(2, c1.centre()); // double intersection point
  if ((c2.type()==vgl_conic<T>::complex_parallel_lines ||
       c2.type()==vgl_conic<T>::complex_intersecting_lines)
      && c1.contains(c2.centre()))
    return vcl_list<vgl_homg_point_2d<T> >(2, c2.centre()); // double intersection point
  if (c1.type() == vgl_conic<T>::no_type   ||  c2.type() == vgl_conic<T>::no_type
   || c1.type()==vgl_conic<T>::complex_parallel_lines||c2.type()==vgl_conic<T>::complex_parallel_lines
   || c1.type()==vgl_conic<T>::complex_intersecting_lines||c2.type()==vgl_conic<T>::complex_intersecting_lines
   || c1.type() == vgl_conic<T>::imaginary_ellipse|| c2.type() == vgl_conic<T>::imaginary_ellipse
   || c1.type() == vgl_conic<T>::imaginary_circle || c2.type() == vgl_conic<T>::imaginary_circle)
    return vcl_list<vgl_homg_point_2d<T> >(); // empty list

  if (c1.type() == vgl_conic<T>::coincident_lines ||
      c1.type() == vgl_conic<T>::real_intersecting_lines ||
      c1.type() == vgl_conic<T>::real_parallel_lines) {
    vcl_list<vgl_homg_point_2d<T> > l1=intersection(c2,c1.components().front());
    vcl_list<vgl_homg_point_2d<T> > l2=intersection(c2,c1.components().back());
    l1.insert(l1.end(), l2.begin(), l2.end()); // append l2 to l1
    return l1;
  }

  if (c2.type() == vgl_conic<T>::coincident_lines ||
      c2.type() == vgl_conic<T>::real_intersecting_lines ||
      c2.type() == vgl_conic<T>::real_parallel_lines) {
    vcl_list<vgl_homg_point_2d<T> > l1=intersection(c1,c2.components().front());
    vcl_list<vgl_homg_point_2d<T> > l2=intersection(c1,c2.components().back());
    l1.insert(l1.end(), l2.begin(), l2.end()); // append l2 to l1
    return l1;
  }

  return do_intersect(c1, c2);
}

//: Return the (at most) two tangent lines that pass through p and are tangent to the conic.
// For points on the conic, exactly 1 line is returned: the tangent at that point.
// For points inside the conic, an empty list is returned.
// For points outside the conic, there are always two tangents returned.
// Found by intersecting the dual conic with the dual line.
// If the conic is degenerate, an empty list is returned, unless the point is
// on the conic (in which case the component is returned to which it belongs,
// or even both components in the exclusive case that the point is the centre).
template <class T>
vcl_list<vgl_homg_line_2d<T> >
vgl_homg_operators_2d<T>::tangent_from(vgl_conic<T> const& c,
                                       vgl_homg_point_2d<T> const& p)
{
  if (c.is_degenerate()) {
    if (!c.contains(p)) return vcl_list<vgl_homg_line_2d<T> >();
    vcl_list<vgl_homg_line_2d<T> > v = c.components();
    if (c.type() == vgl_conic<T>::coincident_lines || p == c.centre())
      return v;
    if (v.size() > 0 && dot(v.front(),p) == 0)
      return vcl_list<vgl_homg_line_2d<T> >(1,v.front());
    else if (v.size() > 1 && dot(v.back(),p) == 0)
      return vcl_list<vgl_homg_line_2d<T> >(1,v.back());
    else
      return vcl_list<vgl_homg_line_2d<T> >();
  }

  vgl_conic<T> C = c.dual_conic();
  vgl_homg_line_2d<T>  l(p.x(),p.y(),p.w()); // dual line
  vcl_list<vgl_homg_point_2d<T> > dualpts = intersection(C,l);
  vcl_list<vgl_homg_line_2d<T> > v;
  vcl_list<vgl_homg_point_2d<T> >::iterator it = dualpts.begin();
  for (; !(it == dualpts.end()); ++it)
    v.push_back(vgl_homg_line_2d<T>((*it).x(), (*it).y(), (*it).w()));
  return v;
}

//: Return the list of common tangent lines of two conics.
// This is done by finding the intersection points of the two dual conics,
// which are the duals of the common tangent lines.
// If one of the conics is degenerate, an empty list is returned.
template <class T>
vcl_list<vgl_homg_line_2d<T> >
vgl_homg_operators_2d<T>::common_tangents(vgl_conic<T> const& c1,
                                          vgl_conic<T> const& c2)
{
  if ((c1.type() != vgl_conic<T>::parabola && ! c1.is_central())  ||
      (c2.type() != vgl_conic<T>::parabola && ! c2.is_central()))
    return vcl_list<vgl_homg_line_2d<T> >();//empty list: degenerate conic has no dual

  vgl_conic<T> C1 = c1.dual_conic();
  vgl_conic<T> C2 = c2.dual_conic();
  vcl_list<vgl_homg_point_2d<T> > dualpts = intersection(C1,C2);
  vcl_list<vgl_homg_line_2d<T> > v;
  vcl_list<vgl_homg_point_2d<T> >::iterator it = dualpts.begin();
  for (; !(it == dualpts.end()); ++it)
    v.push_back(vgl_homg_line_2d<T>((*it).x(), (*it).y(), (*it).w()));
  return v;
}

//: Return the point on the conic closest to the given point
template <class T>
vgl_homg_point_2d<T>
vgl_homg_operators_2d<T>::closest_point(vgl_conic<T> const& c,
                                        vgl_homg_point_2d<T> const& pt)
{
  // First check if the point is on the curve, since this simplifies things:
  if (c.contains(pt)) return pt;

  // The nearest point must have a polar line which is orthogonal to its
  // connection line with the given point; all points with this property form
  // a certain conic  (actually an orthogonal hyperbola) :
  vcl_list<vgl_homg_point_2d<T> > candidates; // all intersection points
  if (pt.w() == 0) { // givel point is at infinity
    // ==> degenerate hyperbola: line + line at infinity
    vgl_homg_line_2d<T> l(c.a()*pt.y()*2-c.b()*pt.x(),
                         -c.c()*pt.x()*2+c.b()*pt.y(),
                          c.d()*pt.y()-c.e()*pt.x());
    candidates = intersection(c, l);
    if (candidates.size() == 0)
      return vgl_homg_point_2d<T>(0,0,0); // this cannot happen
    // just return any of the candidates, since distance makes no sense at infinity:
    else return candidates.front();
  } else if (c.b()==0 && c.a()==c.c()) { // the given conic is a circle
    // ==> degenerate hyperbola: line thru centre & point  +  line at infinity
    vgl_homg_point_2d<T> centre = c.centre();
    if (centre == pt) // in this case, any point of the circle is all right
      centre = vgl_homg_point_2d<T>(1,0,0); // take a horizontal line thru pt
    candidates = intersection(c, vgl_homg_line_2d<T>(centre,pt));
  } else {
    // general case:
    vgl_conic<T> conic(pt.w()*c.b(),
                       pt.w()*(c.c()-c.a())*2,
                      -pt.w()*c.b(),
                       pt.y()*c.a()*2-pt.x()*c.b()+pt.w()*c.e(),
                      -pt.x()*c.c()*2+pt.y()*c.b()-pt.w()*c.d(),
                       pt.y()*c.d()-pt.x()*c.e());
    // Now it suffices to intersect the hyperbola with the given conic:
    candidates = intersection(c, conic);
  }
  if (candidates.size() == 0)
    vcl_cerr << "Warning: vgl_homg_operators_2d<T>::closest_point: no intersection\n";

  // And find the intersection point closest to the given location:
  vgl_homg_point_2d<T> p = candidates.front();
  double dist = 1e31; // vnl_numeric_limits<T>::infinity();
  vcl_list<vgl_homg_point_2d<T> >::iterator it = candidates.begin();
  for (; it != candidates.end(); ++it) {
    if ((*it).w() == 0) continue;
    double d = vgl_homg_operators_2d<T>::distance_squared(*it,pt);
    if (d < dist) { p = (*it); dist = d; }
  }
  return p;
}

//: Return the point on the conic closest to the given point.
//  Still return a homogeneous point, even if the argument is non-homogeneous.
template <class T>
vgl_homg_point_2d<T>
vgl_homg_operators_2d<T>::closest_point(vgl_conic<T> const& c,
                                        vgl_point_2d<T> const& pt)
{
  return closest_point(c,vgl_homg_point_2d<T>(pt));
}

//: Compute the bounding box of an ellipse
// This is done by finding the tangent lines to the ellipse from the two points
// at infinity (1,0,0) and (0,1,0).
template <class T>
vgl_box_2d<T>
vgl_homg_operators_2d<T>::compute_bounding_box(vgl_conic<T> const& c)
{
  // Only ellipses have a finite bounding box:

  if (c.real_type() == "complex intersecting lines") { // a single point:
    vgl_homg_point_2d<T> pt = c.centre();
    return vgl_box_2d<T>(vgl_point_2d<T>(pt), vgl_point_2d<T>(pt));
  }

  if (c.real_type() == "imaginary ellipse"
   || c.real_type() == "imaginary circle"
   || c.real_type() == "complex parallel lines")
    return vgl_box_2d<T>((T)1, (T)0, (T)1, (T)0); // empty box

  if (c.real_type() != "real ellipse" && c.real_type() != "real circle")
    return vgl_box_2d<T>(T(-1e33), T(1e33), T(-1e33), T(1e33)); // everything

  // Now for the ellipses:

  vgl_homg_point_2d<T> px (1,0,0); // point at infinity of the X axis
  vgl_homg_point_2d<T> py (0,1,0); // point at infinity of the Y axis

  vcl_list<vgl_homg_line_2d<T> > lx = vgl_homg_operators_2d<T>::tangent_from(c, px);
  vcl_list<vgl_homg_line_2d<T> > ly = vgl_homg_operators_2d<T>::tangent_from(c, py);

  T y1 = - lx.front().c() / lx.front().b(); // lx are two horizontal lines
  T y2 = - lx.back().c() / lx.back().b();
  if (y1 > y2) { T t = y1; y1 = y2; y2 = t; }
  T x1 = - ly.front().c() / ly.front().a(); // ly are two vertical lines
  T x2 = - ly.back().c() / ly.back().a();
  if (x1 > x2) { T t = x1; x1 = x2; x2 = t; }

  return vgl_box_2d<T>(x1, x2, y1, y2);
}

#endif // vgl_homg_operators_2d_txx_
