#ifdef __GNUG__
#pragma implementation
#endif

#include <vcl/vcl_iostream.h>

#include <vnl/vnl_math.h>
#include <vnl/algo/vnl_svd.h>

#include <mvl/Homg2D.h>
#include <mvl/HomgLine2D.h>
#include <mvl/HomgLineSeg2D.h>
#include <mvl/HomgOperator2D.h>

////////////////#include <Geometry/IUPoint.h>
////////////////#include <Geometry/ImplicitLine.h>

//--------------------------------------------------------------
// -- Return true iff the line is the line at infinity
// This version assumes x,y will be exactly 0 in this case.

bool HomgLine2D::check_infinity() const
{
  return (get_x() == 0) && (get_y() == 0);
}

//--------------------------------------------------------------
// -- Return true iff the line is the line at infinity
// This version checks @{$min(|x|,|y|) < \mbox{tol} \times |z|$@}

bool HomgLine2D::check_infinity(double tol) const
{
  return vnl_math_min(vnl_math_abs(get_x()), vnl_math_abs(get_y())) < tol*vnl_math_abs(get_w());
}

//--------------------------------------------------------------
//
// -- Construct an ImplicitLine by clipping against the given
// bounding rectangle.  The return line has been allocated using new.

HomgLineSeg2D HomgLine2D::clip(int rect1_x, int rect1_y, int rect2_x, int rect2_y) const
{
  return HomgOperator2D::clip_line_to_lineseg(*this, rect1_x, rect1_y, rect2_x, rect2_y);
}

//--------------------------------------------------------------
//
// -- Return some two points which are on the line.  The algorithm actually
// returns an orthonormal basis for the nullspace of l.
void HomgLine2D::get_2_points_on_line(HomgPoint2D* p1, HomgPoint2D* p2) const
{
  vnl_matrix<double> M(get_vector().data_block(), 1, 3);
  vnl_svd<double> svd(M);
  p1->set(svd.V(0,1), svd.V(1,1), svd.V(2,1));
  p2->set(svd.V(0,2), svd.V(1,2), svd.V(2,2));
}

//-----------------------------------------------------------------------------
//
// -- Print to ostream in the format "<HomgLine2D x y w>"
ostream& operator<<(ostream& s, const HomgLine2D& p)
{
  return s << "<HomgLine2D " << p.get_vector() << ">";
}
