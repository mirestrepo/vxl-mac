#ifndef vnl_algo_determinant_h_
#define vnl_algo_determinant_h_
#ifdef __GNUC__
#pragma interface
#endif

//:
// \file
// \brief calculates the determinant of a matrix
// \author fsm@robots.ox.ac.uk
//
//  Evaluation of determinants of any size. For small
//  matrices, will use the direct routines (no netlib)
//  but for larger matrices, a matrix decomposition
//  such as SVD or QR will be used.
//
// \verbatim
// Modifications
//  dac (Manchester) 26/03/2001: tidied up documentation
// \endverbatim

#include <vnl/vnl_matrix.h>


//: direct evaluation for 2x2 matrix
template <class T> T vnl_determinant(T const *row0,
                                     T const *row1);

//: direct evaluation for 3x3 matrix
template <class T> T vnl_determinant(T const *row0,
                                     T const *row1,
                                     T const *row2);

//: direct evaluation for 4x4 matrix
template <class T> T vnl_determinant(T const *row0,
                                     T const *row1,
                                     T const *row2,
                                     T const *row3);

//: evaluation using direct methods for sizes of 2x2, 3x3, and 4x4
// or qr decompostion for other matrices.
template <class T>
T vnl_determinant(T const *const *rows, int size, bool balance = false);

//: evaluation using direct methods for sizes of 2x2, 3x3, and 4x4
// or qr decompostion for other matrices.
template <class T>
T vnl_determinant(vnl_matrix<T> const &M, bool balance = false);

#define VNL_DETERMINANT_INSTANTIATE(T) \
extern "you must include vnl/algo/vnl_determinant.txx first"

#endif // vnl_algo_determinant_h_
