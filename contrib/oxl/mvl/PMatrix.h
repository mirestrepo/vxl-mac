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
#ifndef _PMatrix_h
#define _PMatrix_h
#ifdef __GNUC__
#pragma interface
#endif

//--*- c++ -*------------------------------------------------------------------
//
// .NAME PMatrix - General 3x4 perspective projection matrix.
// .LIBRARY MViewBasics
// .HEADER  MultiView package
// .INCLUDE mvl/PMatrix.h
// .FILE PMatrix.C
//
// .SECTION Description:
//
// A class to hold a perspective projection matrix and use it to
// perform common operations e.g. projecting point in 3d space to
// its image on the image plane
//
// .SECTION Modifications:
//     010796 AWF Implemented get_focal_point() - awf, july 96
//     011096 AWF Added caching vnl_svd<double>
//     260297 AWF Converted to use vnl_double_3x4
//     110397 PVR Added operator==
//

#include <vcl/vcl_compiler.h> // for mutable
#include <vcl/vcl_iosfwd.h>

#include <vnl/algo/vnl_algo_fwd.h>
#include <vnl/vnl_double_3x4.h>

class HomgPoint2D;
class HomgLine2D;  
class HomgLineSeg2D;

class HomgPoint3D;
class HomgPlane3D;
class HomgLine3D;
class HomgLineSeg3D;
class HMatrix3D;
class HMatrix2D;

class PMatrix {

  // PUBLIC INTERFACE----------------------------------------------------------
  
public:
  
  // Constructors/Initializers/Destructors-------------------------------------
  
  PMatrix();
  PMatrix(istream&);
  PMatrix(const double *c_matrix);
  explicit PMatrix(const vnl_matrix<double>&);
  PMatrix(const vnl_matrix<double>& A, const vnl_vector<double>& a);
  PMatrix(const PMatrix&);
 ~PMatrix();

  static PMatrix read(char const* filename);
  static PMatrix read(istream&);
  
  // Operations----------------------------------------------------------------

  HomgPoint2D   project (const HomgPoint3D& X) const;
  HomgLine2D    project (const HomgLine3D& L) const;
  HomgLineSeg2D project (const HomgLineSeg3D& L) const;

  HomgPoint3D backproject_pseudoinverse (const HomgPoint2D& x) const;
  HomgLine3D  backproject (const HomgPoint2D& x) const;
  HomgPlane3D backproject (const HomgLine2D& l) const;

  PMatrix postmultiply(const HMatrix3D& H) const;
  PMatrix postmultiply(const vnl_matrix<double>& H) const;

  PMatrix premultiply(const HMatrix2D& H) const;
  PMatrix premultiply(const vnl_matrix<double>& H) const;

  vnl_svd<double>* svd() const; // mutable const
  void clear_svd();
  HomgPoint3D get_focal_point() const;
  HMatrix3D get_canonical_H() const;
  bool is_canonical(double tol = 0) const;

  bool is_behind_camera(const HomgPoint3D&);
  void flip_sign();
  bool looks_conditioned();
  void fix_cheirality();

  // Data Access---------------------------------------------------------------
  
  PMatrix& operator=(const PMatrix&);

  bool operator==(PMatrix const& p) const { return _p_matrix == p.get_matrix(); }

  void get(vnl_matrix<double>* A, vnl_vector<double>* a) const;
  void set(const vnl_matrix<double>& A, const vnl_vector<double>& a);

  void get(vnl_vector<double>*, vnl_vector<double>*, vnl_vector<double>*) const;
  void set(const vnl_vector<double>&, const vnl_vector<double>&, const vnl_vector<double>&);

  double get (unsigned int row_index, unsigned int col_index) const;
  void get (double *c_matrix) const;
  void get (vnl_matrix<double>* p_matrix) const;

  void set (const double* p_matrix);
  void set (const double p_matrix [3][4]);
  void set (const vnl_matrix<double>& p_matrix);

  const vnl_matrix<double>& get_matrix() const { return _p_matrix; }
  // Can't implement this as it will blow the svd cache.
  // vnl_matrix<double>& get_matrix() { return _p_matrix; }
  
  // Utility Methods-----------------------------------------------------------
  bool read_ascii(istream& f);
  
  // INTERNALS-----------------------------------------------------------------

protected:
  
private:
  
  // Data Members--------------------------------------------------------------
  

protected:

  vnl_double_3x4 _p_matrix;
  mutable vnl_svd<double>* _svd;
};

ostream& operator<<(ostream& s, const PMatrix& p);
istream& operator>>(istream& i, PMatrix& p);

//inline
//PMatrix operator*(const HMatrix3D& C, const PMatrix& P)
//{
//  return PMatrix(C.get_matrix() * P.get_matrix());
//}
//
inline
PMatrix operator*(const vnl_matrix<double>& C, const PMatrix& P)
{
  return PMatrix(C * P.get_matrix());
}

#endif
// _PMatrix_h
