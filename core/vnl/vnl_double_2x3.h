#ifndef vnl_double_2x3_h_
#define vnl_double_2x3_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME        vnl_double_2x3
// .LIBRARY     vnl
// .HEADER	vxl package
// .INCLUDE     vnl/vnl_double_2x3.h
// .FILE        vnl/vnl_double_2x3.cxx
//
// .SECTION Author
//     Andrew W. Fitzgibbon, Oxford RRG, 23 Dec 96
//
// .SECTION Modifications:
//     <none yet>
//
//-----------------------------------------------------------------------------

#include <vnl/vnl_matrix_fixed.h>
#include <vnl/vnl_double_3.h>

class vnl_double_2x3 : public vnl_matrix_fixed<double, 2, 3> {
  typedef vnl_matrix_fixed<double, 2, 3> Base;
public:

  vnl_double_2x3() {}
  vnl_double_2x3(const vnl_double_3& row1, const vnl_double_3& row2) {
    vnl_matrix<double>& M = *this;
    M(0,0) = row1[0];    M(0,1) = row1[1];    M(0,2) = row1[2];
    M(1,0) = row2[0];    M(1,1) = row2[1];    M(1,2) = row2[2];
  }

  vnl_double_2x3(double r00, double r01, double r02,
	    double r10, double r11, double r12) {
    vnl_matrix<double>& M = *this;
    M(0,0) = r00;    M(0,1) = r01;    M(0,2) = r02;
    M(1,0) = r10;    M(1,1) = r11;    M(1,2) = r12;
  }
};

#endif   // DO NOT ADD CODE AFTER THIS LINE! END OF DEFINITION FOR CLASS vnl_double_2x3.

