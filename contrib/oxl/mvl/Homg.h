#ifndef _Homg_h
#define _Homg_h
#ifdef __GNUC__
#pragma interface
#endif
//
// .NAME    Homg - Private base class for homogeneous vectors
// .LIBRARY MViewBasics
// .HEADER  MultiView Package
// .INCLUDE mvl/Homg.h
// .FILE    Homg.cxx
//
// .SECTION Description:
//
// This is the private base class for homogeneous vectors.  It provides the
// get/set interface, and also a static variable Homg::infinity which is used
// throughout when returning infinite nonhomogeneous values.
//
// .SECTION Author:
//             Paul Beardsley, 29.03.96
//             Oxford University, UK
//
// .SECTION Modifications:
//    210297 AWF Switched to fixed-length vectors for speed.
//
//-------------------------------------------------------------------------------

#include <vcl_cassert.h>
#if defined(VCL_GCC_295)
# include <vnl/vnl_vector_fixed.h>
#endif

class Homg {

  // PUBLIC INTERFACE----------------------------------------------------------
public:

//: Standard placeholder for methods that wish to return infinity.
  static double infinity;


//: The tolerance used in "near zero" tests in the Homg subclasses.
  static double infinitesimal_tol;

//: Static method to set the default tolerance used for infinitesimal checks.
// The default is 1e-12.
  static void set_infinitesimal_tol(double tol);
};

#endif // _Homg_h
