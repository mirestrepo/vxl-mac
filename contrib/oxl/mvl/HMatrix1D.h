#ifndef _HMatrix1D_h
#define _HMatrix1D_h
#ifdef __GNUC__
#pragma interface
#endif

//-*- c++ -*-------------------------------------------------------------------
//
// .NAME HMatrix1D
// .LIBRARY MViewBasics
// .HEADER MultiView package
// .INCLUDE mvl/HMatrix1D.h
// .FILE HMatrix1D.cxx
//
// .SECTION Description:
// A class to hold a line-to-line projective transformation matrix
// and to perform common operations using it e.g. transfer point.
//

#include <vnl/vnl_double_2x2.h>
#include <mvl/HomgPoint1D.h>
#include <vcl/vcl_iosfwd.h>


class HMatrix1D {

  // PUBLIC INTERFACE----------------------------------------------------------

public:
  // Constructors/Initializers/Destructors-------------------------------------

  HMatrix1D();
  HMatrix1D(const HMatrix1D& M);
  HMatrix1D(const HMatrix1D&,const HMatrix1D&);// product of two HMatrix1Ds
  HMatrix1D(const vnl_matrix<double>& M);
  HMatrix1D(const double* t_matrix);
  HMatrix1D(istream& s);
 ~HMatrix1D();
 static HMatrix1D read(char const* filename);
 static HMatrix1D read(istream&);

  // Operations----------------------------------------------------------------

 // deprecated. also misnomers :
 HomgPoint1D transform_to_plane2(const HomgPoint1D& x1) const;
 HomgPoint1D transform_to_plane1(const HomgPoint1D& x2) const;

 HomgPoint1D operator()(const HomgPoint1D& x1) const;
 HomgPoint1D preimage(const HomgPoint1D& x2) const;

  // Data Access---------------------------------------------------------------

  double get (unsigned int row_index, unsigned int col_index) const;
  void get (double *t_matrix) const;
  void get (vnl_matrix<double>* t_matrix) const;
  const vnl_double_2x2& get_matrix () const { return _t12_matrix; }
  const vnl_double_2x2& get_inverse () const { return _t21_matrix; }

  void set (const double *t_matrix);
  void set (const vnl_matrix<double>& t_matrix);
  void set_inverse (const vnl_matrix<double>& t21_matrix);

  // INTERNALS-----------------------------------------------------------------
private:
  // Data Members--------------------------------------------------------------
  vnl_double_2x2 _t12_matrix;
  vnl_double_2x2 _t21_matrix;
};

ostream& operator << (ostream& s, const HMatrix1D& H);
istream& operator >> (istream& s, HMatrix1D& H);

#endif // _HMatrix1D_h
