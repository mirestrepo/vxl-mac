// This is ./vxl/vnl/vnl_double_2x2.h
#ifndef vnl_double_2x2_h_
#define vnl_double_2x2_h_
#ifdef __GNUC__
#pragma interface
#endif

//: \file
//  \brief 2x2 matrix of double
//
//    vnl_double_2x2 is a vnl_matrix<double> of fixed size 2x2.  It is
//    merely a typedef for vnl_matrix_fixed<double,2,2>
//
//  \author Andrew W. Fitzgibbon, Oxford RRG
//  \date   04 Aug 96
//
// \verbatim
// Modifications:
// 4/4/01 Tidied documentation
// \endverbatim
//
//
//-----------------------------------------------------------------------------

#include <vnl/vnl_matrix_fixed.h>

typedef vnl_matrix_fixed<double,2,2> vnl_double_2x2;

#endif // vnl_double_2x2_h_
