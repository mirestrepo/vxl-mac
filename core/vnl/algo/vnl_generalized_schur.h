#ifndef vnl_generalized_schur_h_
#define vnl_generalized_schur_h_
#ifdef __GNUC__
#pragma interface
#endif

//:
// \file
// \brief  Solves the generalized eigenproblem det(t A - s B) = 0.
// \author fsm, Oxford RRG
// \date   2 Oct 2001
//
// \verbatim
// Modifications:
// \endverbatim

#include <vnl/vnl_matrix.h>

template <class T>
bool vnl_generalized_schur(vnl_matrix<T> *A,
                           vnl_matrix<T> *B,
                           vnl_vector<T> *alphar,
                           vnl_vector<T> *alphai,
                           vnl_vector<T> *beta,
                           vnl_matrix<T> *L,
                           vnl_matrix<T> *R);

VCL_DEFINE_SPECIALIZATION
bool vnl_generalized_schur(vnl_matrix<double> *A,
                           vnl_matrix<double> *B,
                           vnl_vector<double> *alphar,
                           vnl_vector<double> *alphai,
                           vnl_vector<double> *beta,
                           vnl_matrix<double> *L,
                           vnl_matrix<double> *R);

#include <vcl_algorithm.h>

template <class T>
inline bool vnl_generalized_schur(vnl_matrix<T> *A,
                                  vnl_matrix<T> *B,
                                  vnl_vector<T> *alphar,
                                  vnl_vector<T> *alphai,
                                  vnl_vector<T> *beta,
                                  vnl_matrix<T> *L,
                                  vnl_matrix<T> *R)
{
  vnl_matrix<double> A_(A->rows(), A->cols());
  vnl_matrix<double> B_(B->rows(), B->cols());
  vcl_copy(A->begin(), A->end(), A_.begin());
  vcl_copy(B->begin(), B->end(), B_.begin());
  
  vnl_vector<double> alphar_;
  vnl_vector<double> alphai_;
  vnl_vector<double> beta_;
  vnl_matrix<double> L_;
  vnl_matrix<double> R_;
  
  if (! vnl_generalized_schur/*<double>*/(&A_, &B_, &alphar_, &alphai_, &beta_, &L_, &R_))
    return false;
  
  vcl_copy(A_.begin(), A_.end(), A->begin());
  vcl_copy(B_.begin(), B_.end(), B->begin());
  
  alphar->resize(alphar_.size()); vcl_copy(alphar_.begin(), alphar_.end(), alphar->begin());
  alphai->resize(alphai_.size()); vcl_copy(alphai_.begin(), alphai_.end(), alphai->begin());
  beta  ->resize(beta_  .size()); vcl_copy(beta_  .begin(), beta_  .end(), beta  ->begin());
  L->resize(L_.rows(), L_.cols()); vcl_copy(L_.begin(), L_.end(), L->begin());
  R->resize(R_.rows(), R_.cols()); vcl_copy(R_.begin(), R_.end(), R->begin());
  
  return true;
}

#endif
