#ifndef vnl_sparse_symmetric_eigensystem_h_
#define vnl_sparse_symmetric_eigensystem_h_
#ifdef __GNUC__
#pragma interface
#endif
//
// .NAME	vnl_sparse_symmetric_eigensystem
// .LIBRARY	vnl-algo
// .HEADER	vxl Package
// .INCLUDE	vnl/algo/vnl_sparse_symmetric_eigensystem.h
// .FILE	vnl_sparse_symmetric_eigensystem.cxx
//
// .SECTION Description
//    Solve the eigenproblem $A x = \lambda x$, with $A$ symmetric and
//    sparse.  The block Lanczos algorithm is used to allow the
//    recovery of a number of eigenvale/eigenvector pairs from either
//    end of the spectrum, to a required accuracy.
//
//    Uses the dnlaso routine from the LASO package of netlib. 
//
// .SECTION Author
//    Rupert W. Curwen, GE CR&D, 20 Oct 98

#include <vnl/vnl_sparse_matrix.h>
#include <vcl_vector.h>

//: Solve $A x = \lambda x$ using Lanczos algorithm.
class vnl_sparse_symmetric_eigensystem {
public:
  vnl_sparse_symmetric_eigensystem();
  
  // Find n eigenvalue/eigenvectors.  If smallest is true, will
  // calculate the n smallest eigenpairs, else the n largest.
  int CalculateNPairs(vnl_sparse_matrix<double>& M, int n, 
		      bool smallest = true, int nfigures = 10);

  // Recover specified eigenvector after computation.  The argument
  // must be less than the requested number of eigenvectors.
  vnl_vector<double> get_eigenvector(int i) const;
  double get_eigenvalue(int i) const;

  // Used as a callback in solving.
  int CalculateProduct(int n, int m, const double* p, double* q);
  int SaveVectors(int n, int m, const double* q, int base);
  int RestoreVectors(int n, int m, double* q, int base);

protected:
  int nvalues;  // this is the size of the next two arrays.
  vnl_vector<double> * vectors; // eigenvectors
  double * values;              // eigenvalues

  vnl_sparse_matrix<double> * mat;

  vcl_vector<double*> temp_store;
};

#endif // vnl_sparse_symmetric_eigensystem_h_
