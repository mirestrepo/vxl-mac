#ifndef PMatrixDecompAa_h_
#define PMatrixDecompAa_h_
#ifdef __GNUC__
#pragma interface
#endif
//
// .NAME	PMatrixDecompAa - Decompose PMatrix into [A a]
// .LIBRARY	MViewBasics
// .HEADER	MultiView Package
// .INCLUDE	mvl/PMatrixDecompAa.h
// .FILE	PMatrixDecompAa.cxx
// .EXAMPLE	examples/examplePMatrixDecompAa.cxx
//
// .SECTION Description
//    Decompose PMatrix into [A a] where A is 3x3 and a is 3x1.
//
// .SECTION Author
//     Andrew W. Fitzgibbon, Oxford RRG, 14 Feb 97

#include <vnl/vnl_double_3x3.h>
#include <vnl/vnl_double_3.h>
#include <mvl/PMatrix.h>

class PMatrixDecompAa {
public:

// -- Public data members for A and a.
  vnl_double_3x3 A;
  vnl_double_3   a;
  //{genman, do not do anything to this line -- awf}

// -- Default constructor
  PMatrixDecompAa() {}

// -- Construct from PMatrix.
  PMatrixDecompAa(const PMatrix& P) { set(P); }

// --  Set [A a] from PMatrix.
  void set(const PMatrix& P) { P.get(&A, &a); }

// --  Set PMatrix from [A a].
  void get(PMatrix* P) const { P->set(A, a); }

};

#endif // PMatrixDecompAa_h_
