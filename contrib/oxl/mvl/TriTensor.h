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
//- -*- c++ -*- ----------------------------------------------------------------
#ifndef _TriTensor_h
#define _TriTensor_h
#ifdef __GNUC__
#pragma interface
#endif
//
// .SECTION Description:
//
// A class to hold a Trifocal Tensor and perform common operations, such as
// point and line transfer, coordinate-frame transformation and I/O.
//
// .NAME TriTensor - The trifocal tensor.
// .LIBRARY MViewBasics
// .HEADER MultiView package
// .INCLUDE mvl/TriTensor.h
// .FILE TriTensor.C
//
// .SECTION Author:
//             Paul Beardsley, 29.03.96
//             Oxford University, UK
//
// .SECTION Modifications:
//   AWF - Added composition, transformation, homography generation.
//   Peter Vanroose - 11 Mar 97 - added operator==
//
//------------------------------------------------------------------------------

#include <vcl/vcl_vector.h>

#include <vbl/vbl_array_3d.h>

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_double_3x3.h>

#include <mvl/HomgLine2D.h>    
#include <mvl/HomgLineSeg2D.h>    
#include <mvl/HomgPoint2D.h>

class HMatrix2D;
class FMatrix;
class PMatrix;
class FManifoldProject;

class TriTensor {
  
  // PUBLIC INTERFACE--------------------------------------------------------
  
public:
  
  // Constructors/Initializers/Destructors-----------------------------------
  
  // Construct from 27-element vector
  TriTensor();
  TriTensor(const double *tritensor_array);
  TriTensor(const TriTensor&);
  TriTensor(const PMatrix& P1, const PMatrix& P2, const PMatrix& P3);
  TriTensor(const PMatrix& P2, const PMatrix& P3);
  TriTensor(const vnl_matrix<double>& T1, const vnl_matrix<double>& P2, const vnl_matrix<double>& P3);
 ~TriTensor();
  
  // Data Access-------------------------------------------------------------

  TriTensor& operator=(const TriTensor&);
  bool operator==(TriTensor const& p) const {
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k) if (p(i,j,k)!=T(i,j,k)) return false;
    return true; }
  double& operator() (unsigned int i1, unsigned int i2, unsigned int i3) { return T(i1,i2,i3); }
  double operator() (unsigned int i1, unsigned int i2, unsigned int i3) const { return T(i1,i2,i3); }

  void set (unsigned int i1, unsigned int i2, unsigned int i3, double value);

  void set(const double* vec);
  void set(const vnl_matrix<double>& tvector); // 27x1 matrix
  void convert_to_vector(vnl_matrix<double>* tvector) const; // 27x1 matrix

  void set(const PMatrix& P1, const PMatrix& P2, const PMatrix& P3);
  void set(const PMatrix& P2, const PMatrix& P3);
  void set(const vnl_matrix<double>& T1, const vnl_matrix<double>& T2, const vnl_matrix<double>& T3);
  
  // Data Control------------------------------------------------------------

  HomgPoint2D image1_transfer (const HomgPoint2D& point2, const HomgPoint2D& point3, HomgPoint2D corrected[] = 0) const;
  HomgPoint2D image2_transfer (const HomgPoint2D& point1, const HomgPoint2D& point3, HomgPoint2D corrected[] = 0) const;
  HomgPoint2D image3_transfer (const HomgPoint2D& point1, const HomgPoint2D& point2, HomgPoint2D corrected[] = 0) const;

  HomgPoint2D image1_transfer_qd (const HomgPoint2D& point2, const HomgPoint2D& point3) const;
  HomgPoint2D image2_transfer_qd (const HomgPoint2D& point1, const HomgPoint2D& point3) const;
  HomgPoint2D image3_transfer_qd (const HomgPoint2D& point1, const HomgPoint2D& point2) const;

  HomgPoint2D image1_transfer (const HomgPoint2D& point1, const HomgLine2D& line2) const;
  HomgPoint2D image2_transfer (const HomgPoint2D& point1, const HomgLine2D& line3) const;
  HomgPoint2D image3_transfer (const HomgPoint2D& point2, const HomgLine2D& line3) const;

  HomgLine2D image1_transfer (const HomgLine2D& line2, const HomgLine2D& line3) const;
  HomgLine2D image2_transfer (const HomgLine2D& line2, const HomgLine2D& line3) const;
  HomgLine2D image3_transfer (const HomgLine2D& line2, const HomgLine2D& line3) const;

  //HMatrix2D get_hmatrix_23(const HomgLine2D& line1);
  HMatrix2D get_hmatrix_31(const HomgLine2D& line2) const;
  HMatrix2D get_hmatrix_21(const HomgLine2D& line3) const;

  bool get_epipoles(HomgPoint2D* e2, HomgPoint2D* e3) const;
  bool compute_epipoles();

  HomgPoint2D get_epipole_12() const;
  HomgPoint2D get_epipole_13() const;

  FMatrix get_fmatrix_13() const;
  FMatrix get_fmatrix_12() const;

  // Use these if you've already got the epipoles
  FMatrix get_fmatrix_13(const HomgPoint2D& e2, const HomgPoint2D& e3) const;
  FMatrix get_fmatrix_12(const HomgPoint2D& e2, const HomgPoint2D& e3) const;

  FMatrix compute_fmatrix_23() const;

  FManifoldProject* get_fmp12() const;
  FManifoldProject* get_fmp23() const;
  FManifoldProject* get_fmp13() const;

  void compute_P_matrices(const vnl_vector<double>& x, double alpha, double beta, PMatrix* P2, PMatrix* P3) const;
  void compute_P_matrices(const vnl_vector<double>& x, double alpha, PMatrix* P2, PMatrix* P3) const {
    compute_P_matrices(x,alpha,alpha, P2, P3);
  }
  void compute_P_matrices(const vnl_vector<double>& x, PMatrix* P2, PMatrix* P3) const {
    compute_P_matrices(x, 1, 1, P2, P3);
  }
  void compute_P_matrices(PMatrix* P2, PMatrix* P3) const {
    compute_P_matrices(vnl_double_3(1,1,1), 1, 1, P2, P3);
  }
  
  void compute_caches();
  void clear_caches();
  
  // Utility Methods---------------------------------------------------------
  void get_constraint_lines_image3(const HomgPoint2D& p1, const HomgPoint2D& p2, vcl_vector<HomgLine2D>* lines) const;
  void get_constraint_lines_image2(const HomgPoint2D& p1, const HomgPoint2D& p3, vcl_vector<HomgLine2D>* lines) const;
  void get_constraint_lines_image1(const HomgPoint2D& p2, const HomgPoint2D& p3, vcl_vector<HomgLine2D>* lines) const;

  // -- Contract Tensor axis tensor_axis with first component of Matrix M.
  // That is: 
  // For tensor_axis = 1,  Compute T_ijk = T_pjk M_pi
  // For tensor_axis = 2,  Compute T_ijk = T_ipk M_pj
  // For tensor_axis = 3,  Compute T_ijk = T_ijp M_pk
  TriTensor postmultiply(unsigned tensor_axis, const vnl_matrix<double>& M) const;

  // -- Contract Tensor axis tensor_axis with second component of Matrix M.
  // That is: 
  // For tensor_axis = 1,  Compute T_ijk = M_ip T_pjk
  // For tensor_axis = 2,  Compute T_ijk = M_jp T_ipk
  // For tensor_axis = 3,  Compute T_ijk = M_kp T_ijp
  TriTensor premultiply(unsigned tensor_axis, const vnl_matrix<double>& M) const;

  TriTensor postmultiply1(const vnl_matrix<double>& M) const;
  TriTensor postmultiply2(const vnl_matrix<double>& M) const;
  TriTensor postmultiply3(const vnl_matrix<double>& M) const;

  TriTensor premultiply1(const vnl_matrix<double>& M) const;
  TriTensor premultiply2(const vnl_matrix<double>& M) const;
  TriTensor premultiply3(const vnl_matrix<double>& M) const;

  vnl_double_3x3 dot1(const vnl_vector<double>& v) const;
  vnl_double_3x3 dot2(const vnl_vector<double>& v) const;
  vnl_double_3x3 dot3(const vnl_vector<double>& v) const;
  vnl_double_3x3 dot1t(const vnl_vector<double>& v) const;
  vnl_double_3x3 dot2t(const vnl_vector<double>& v) const;
  vnl_double_3x3 dot3t(const vnl_vector<double>& v) const;

  bool check_equal_up_to_scale(const TriTensor& that) const;
  
  // INTERNALS---------------------------------------------------------------
  
  // -- C123 are line conditioning matrices.
  // If C * l = lhat, and l1 = T l2 l3, then lhat1 = That lhat2 lhat3
  TriTensor condition(const vnl_matrix<double>& line_1_denorm,
		      const vnl_matrix<double>& line_2_norm,
		      const vnl_matrix<double>& line_3_norm) const;
  
  TriTensor decondition(const vnl_matrix<double>& line_1_norm,
			const vnl_matrix<double>& line_2_denorm,
			const vnl_matrix<double>& line_3_denorm) const;

protected:
  
private:
  
  // Data Members------------------------------------------------------------
  
private:
  
  vbl_array_3d<double> T;

  // Caches for various computed quantities
  void init_caches();
  void delete_caches();

  HomgPoint2D* _e12;
  HomgPoint2D* _e13;

  FManifoldProject* _fmp12;
  FManifoldProject* _fmp23;
  FManifoldProject* _fmp13;
};

ostream& operator << (ostream&, const TriTensor& T);
istream& operator >> (istream&, TriTensor& T);

#endif
// _TriTensor_h
