//-*- c++ -*-------------------------------------------------------------------
#ifndef HMatrix2DAffineCompute_h_
#define HMatrix2DAffineCompute_h_
#ifdef __GNUC__
#pragma interface
#endif
//
// Class : HMatrix2DAffineCompute
//
// .SECTION Description:
//
// @{ HMatrix2DAffineCompute contains a linear method to compute
// a 2D affine transformation. The H returned is such that
// \[
//     x_2 \sim H x_1
// \] 
// @}
//
// .NAME HMatrix2DAffineCompute
// .LIBRARY MViewCompute
// .HEADER MultiView package
// .INCLUDE mvl/HMatrix2DAffineCompute.h
// .FILE HMatrix2DAffineCompute.C
// .SECTION Author
//     David Capel, Oxford RRG, 13 May 98
// .SECTION Modifications:
//     FSM 23-08-98 Added constructor so that the class can be used
//                  as a compute object. Removed compute() method
//                  taking PairMatchSet argument. Changed the remaining
//                  compute method to take an HMatrix2D* argument instead
//                  of returning an HMatrix2D.
//

//
#include <mvl/HMatrix2DCompute.h>
#include <mvl/HMatrix2D.h>
class HomgPoint2D;

class HMatrix2DAffineCompute : public HMatrix2DCompute {
protected:
  bool compute_p(const PointArray &,
		 const PointArray &,
		 HMatrix2D *);
public:
  HMatrix2DAffineCompute(void);
  ~HMatrix2DAffineCompute();

  // left in for capes :
  static HMatrix2D compute (const PairMatchSetCorner &matches);
  static HMatrix2D compute (const vcl_vector<HomgPoint2D>&p1, const vcl_vector<HomgPoint2D>&p2);
private:
  static bool tmp_fun(const PointArray&,
		       const PointArray&,
		       HMatrix2D*);
};

//--------------------------------------------------------------------------------

#include <vnl/vnl_matrix.h>
//
// This class takes an array of n HomgPoint2Ds and creates
// an n-by-2 matrix whose ith row contains the inhomogeneous 
// coordinates of the ith homogeneous point.
//
struct NonHomg : public vnl_matrix<double> {
  NonHomg(const vcl_vector<HomgPoint2D> &A);
};

//
// This function computes the mean of the columns of
// an n-by-2 matrix A.
//
vnl_double_2 mean2(const vnl_matrix<double> &A);


//
// This function subtracts the 2-vector a from each row of
// the n-by-2 matrix A.
//
vnl_matrix<double>& sub_rows(vnl_matrix<double> &A, const vnl_double_2 a);

//--------------------------------------------------------------------------------

#endif   // DO NOT ADD CODE AFTER THIS LINE! END OF DEFINITION FOR CLASS HMatrix2DAffineCompute.
