// <begin copyright notice>
// ---------------------------------------------------------------------------
//
//                   Copyright (c) 1997 TargetJr Consortium
//               GE Corporate Research and Development (GE CRD)
//                             1 Research Circle
//                            Niskayuna, NY 12309
//                            All Rights Reserved
//              Reproduction rights limited as described below.
//                               
//      Permission to use, copy, modify, distribute, and sell this software
//      and its documentation for any purpose is hereby granted without fee,
//      provided that (i) the above copyright notice and this permission
//      notice appear in all copies of the software and related documentation,
//      (ii) the name TargetJr Consortium (represented by GE CRD), may not be
//      used in any advertising or publicity relating to the software without
//      the specific, prior written permission of GE CRD, and (iii) any
//      modifications are clearly marked and summarized in a change history
//      log.
//       
//      THE SOFTWARE IS PROVIDED "AS IS" AND WITHOUT WARRANTY OF ANY KIND,
//      EXPRESS, IMPLIED OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY
//      WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
//      IN NO EVENT SHALL THE TARGETJR CONSORTIUM BE LIABLE FOR ANY SPECIAL,
//      INCIDENTAL, INDIRECT OR CONSEQUENTIAL DAMAGES OF ANY KIND OR ANY
//      DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
//      WHETHER OR NOT ADVISED OF THE POSSIBILITY OF SUCH DAMAGES, OR ON
//      ANY THEORY OF LIABILITY ARISING OUT OF OR IN CONNECTION WITH THE
//      USE OR PERFORMANCE OF THIS SOFTWARE.
//
// ---------------------------------------------------------------------------
// <end copyright notice>
#ifdef __GNUG__
#pragma implementation
#endif
//--------------------------------------------------------------
//
// Class : HomgLine3D
//
// Modifications : see HomgLine3D.h
//
//--------------------------------------------------------------

#include <vcl/vcl_iostream.h>
#include <vcl/vcl_cmath.h>
//#include <vcl/vcl_memory.h>
#include <vcl/vcl_cstdlib.h>
#include <mvl/Homg3D.h>
#include <mvl/HomgLine3D.h>

//--------------------------------------------------------------
//
// -- Constructor
HomgLine3D::HomgLine3D()
{
}

//--------------------------------------------------------------
//
// -- Constructor
HomgLine3D::HomgLine3D( const HomgLine3D &that)
  : point_finite_(that.point_finite_)
  , point_infinite_(that.point_infinite_)
{
}

//--------------------------------------------------------------
//
// -- Constructor, initialise using the specified distinct points
// on the line.
HomgLine3D::HomgLine3D (const HomgPoint3D& start,
			const HomgPoint3D& end)
{
  // ho_quadvecstd_points2_to_line
  bool start_finite = start.get_w() != 0;
  bool end_finite = end.get_w() != 0;
  
  if (start_finite && end_finite) {
    point_finite_ = start;

    vnl_double_3 start_trivec = start.get_double3();
    vnl_double_3 end_trivec = end.get_double3();
    vnl_double_3 direction = (end_trivec - start_trivec);
    direction.normalize();

    point_infinite_.set(direction[0], direction[1], direction[2], 0.0);
  } else if (end_finite) {
    // Start infinite
    point_finite_ = end;
    
    const vnl_vector<double>& dir = start.get_vector();
    point_infinite_ = HomgPoint3D(dir / dir.magnitude());
  } else {
    // End infinite -- just assign
    point_finite_ = start;
    const vnl_vector<double>& dir = end.get_vector();
    point_infinite_ = HomgPoint3D(dir / dir.magnitude());
  }
}

//--------------------------------------------------------------
//
// -- Destructor
HomgLine3D::~HomgLine3D()
{
}

//-----------------------------------------------------------------------------
//
// -- print to ostream
ostream& operator<<(ostream& s, const HomgLine3D& l)
{
  return s << "<HomgLine3D " << l.get_point_finite() << " dir " << l.get_point_infinite() << ">";
}

// -- Push point2 off to infinity
void HomgLine3D::force_point2_infinite()
{
}

// -- Return line direction as a 3-vector
vnl_double_3 HomgLine3D::dir() const
{
  const vnl_vector<double>& d = point_infinite_.get_vector();
  if (d[3] != 0) { 
    cerr << *this;
    cerr << "*** HomgLine3D: Infinite point not at infinity!! ***\n";
  }
  return vnl_double_3(d[0], d[1], d[2]);
}
