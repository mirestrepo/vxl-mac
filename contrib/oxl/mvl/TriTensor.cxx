#ifdef __GNUG__
#pragma implementation
#endif

#include <vcl_iostream.h>
#include <vcl_cmath.h>
//#include <vcl_memory.h>
#include <vcl_cstdlib.h>
#include <vcl_vector.h>

#include <vul/vul_printf.h>

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_matlab_print.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_double_3x3.h>
#include <vnl/vnl_identity_3x3.h>
#include <vnl/vnl_transpose.h>
#include <vnl/vnl_cross_product_matrix.h>
#include <vnl/algo/vnl_svd.h>

#include <mvl/HomgLine2D.h>
#include <mvl/HomgPoint2D.h>
#include <mvl/HMatrix2D.h>
#include <mvl/FMatrix.h>
#include <mvl/PMatrix.h>
#include <mvl/TriTensor.h>
#include <mvl/PMatrixDecompAa.h>
#include <mvl/HMatrix3D.h>
#include <mvl/HomgOperator2D.h>
#include <mvl/FManifoldProject.h>

struct OuterProduct3x3 : public vnl_double_3x3 {
public:
  // Constructors/Destructors--------------------------------------------------

  OuterProduct3x3(const vnl_vector<double>& a, const vnl_vector<double>& b);
};

inline OuterProduct3x3::OuterProduct3x3(const vnl_vector<double>& a, const vnl_vector<double>& b)
{
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      put(i,j, a[i] * b[j]);
}


static bool tt_verbose = false;

#define MUTABLE_CAST(slot) (((TriTensor*)this)->slot) // const violation ...

//--------------------------------------------------------------


//: Default constructor.
TriTensor::TriTensor ():
  T (3, 3, 3)
{
  init_caches();
}

//
//: Construct a TriTensor from a linear array of doubles.
// The doubles are stored in ``matrix'' order, with the last index
// increasing fastest.
TriTensor::TriTensor (const double *tritensor_array)
  : T(3, 3, 3, tritensor_array)
{
  init_caches();
}

//: Copy constructor.
TriTensor::TriTensor (const TriTensor& that):
  T(that.T)
{
  init_caches();
}

//: Construct from 3 projection matrices.
TriTensor::TriTensor (const PMatrix& P1, const PMatrix& P2, const PMatrix& P3):
  T (3, 3, 3)
{
  init_caches();
  set(P1, P2, P3);
}

//: Construct from 2 projection matrices, as described in set.
TriTensor::TriTensor (const PMatrix& P2, const PMatrix& P3):
  T (3, 3, 3)
{
  init_caches();
  set(P2, P3);
}

//: Construct from 3 matrices.
TriTensor::TriTensor (const vnl_matrix<double>& T1, const vnl_matrix<double>& T2, const vnl_matrix<double>& T3):
  T (3, 3, 3)
{
  init_caches();
  set(T1, T2, T3);
}

//: Assignment
TriTensor& TriTensor::operator = (const TriTensor& that)
{
  T = that.T;
  delete_caches();

  return *this;
}

//: Destructor
TriTensor::~TriTensor()
{
  delete_caches();
}

// - Zero epipole and manifold projector classes
void TriTensor::init_caches()
{
  _e12 = 0;
  _e13 = 0;

  _fmp12 = 0;
  _fmp13 = 0;
  _fmp23 = 0;
}

// - Delete and zero epipole and manifold projector classes.
void TriTensor::delete_caches()
{
  delete _e12;
  delete _e13;
  delete _fmp12;
  delete _fmp13;
  delete _fmp23;

  init_caches();
}

//: Convert T to 27x1 matrix. Out is assumed to have been appropriately resized.
void TriTensor::convert_to_vector(vnl_matrix<double> * out) const
{
  // tr_convert_tensor_to_vector
  assert (out-> rows() == 27 && out-> columns() == 1);
  T.get(out->data_block());
}

//: Convert from 27x1 matrix.
void TriTensor::set(const vnl_matrix<double>& in)
{
  assert (in.rows() == 27 && in.columns() == 1);
  T.set(in.data_block());
  delete_caches();
}

// @{ BUILDING @}

//: Construct from 3 projection matrices.
void
TriTensor::set(const PMatrix& P1, const PMatrix& P2, const PMatrix& P3)
{
  PMatrixDecompAa p2;
  PMatrixDecompAa p3;

  bool canon1 = P1.is_canonical(1e-12);
  if (canon1) {
    p2.set(P2);
    p3.set(P3);
  } else {
    HMatrix3D H = P1.get_canonical_H();
    p2.set(P2 * H);
    p3.set(P3 * H);
  }

  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      for(int k = 0; k < 3; ++k)
        T(i,j,k) = p2.A(j,i) * p3.a[k] - p3.A(k,i) * p2.a[j];
  delete_caches();
}

//: Construct from 2 projection matrices, P2 and P3.  The first is assumed to
// be the canonical [I | 0].
void
TriTensor::set(const PMatrix& P2, const PMatrix& P3)
{
  PMatrixDecompAa p2;
  PMatrixDecompAa p3;

  p2.set(P2);
  p3.set(P3);

  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      for(int k = 0; k < 3; ++k)
        T(i,j,k) = p2.A(j,i) * p3.a[k] - p3.A(k,i) * p2.a[j];

  delete_caches();
}

//: Construct from 3 T matrices.
void
TriTensor::set(const vnl_matrix<double>& T1, const vnl_matrix<double>& T2, const vnl_matrix<double>& T3)
{
  vcl_cerr << "Unimplemented\n";
  vnl_matrix<double> const* Ts[3];
  Ts[0] = &T1;
  Ts[1] = &T2;
  Ts[2] = &T3;

  for(int i = 0; i < 3; ++i) {
    const vnl_matrix<double>& Ti = *Ts[i];
    for(int j = 0; j < 3; ++j)
      for(int k = 0; k < 3; ++k)
        T(i,j,k) = Ti(j,k);
  }
  delete_caches();
}

// @{ TRANSFER @}

//-----------------------------------------------------------------------------
//
//: For the specified points in image 1/2, return the transferred point in image 3.
// Transfer is via optimal backprojection.
//

HomgPoint2D
TriTensor::image3_transfer (const HomgPoint2D& point1, const HomgPoint2D& point2, HomgPoint2D corrected[]) const
{
  HomgPoint2D corr[2];
  if (!corrected) corrected = corr;
  get_fmp12()->correct(point1, point2, &corrected[0], &corrected[1]);

  vcl_vector<HomgLine2D> constraint_lines(9);
  get_constraint_lines_image3(corrected[0], corrected[1], &constraint_lines);
  return HomgOperator2D::lines_to_point(constraint_lines);
}

//-----------------------------------------------------------------------------
//
//: For the specified points in image 1/3, return the transferred point in image 2.
//

HomgPoint2D
TriTensor::image2_transfer (const HomgPoint2D& point1, const HomgPoint2D& point3, HomgPoint2D corrected[]) const
{
  HomgPoint2D corr[2];
  if (!corrected) corrected = corr;
  get_fmp13()->correct(point1, point3, &corrected[0], &corrected[1]);

  vcl_vector<HomgLine2D> constraint_lines(9);
  get_constraint_lines_image2(corrected[0], corrected[1], &constraint_lines);
  return HomgOperator2D::lines_to_point(constraint_lines);
}

//-----------------------------------------------------------------------------
//
//: For the specified points in image 2/3, return the transferred point in image 1.
//

HomgPoint2D
TriTensor::image1_transfer (const HomgPoint2D& point2, const HomgPoint2D& point3, HomgPoint2D corrected[]) const
{
  HomgPoint2D corr[2];
  if (!corrected) corrected = corr;
  get_fmp23()->correct(point2, point3, &corrected[0], &corrected[1]);

  vcl_vector<HomgLine2D> constraint_lines(9);
  get_constraint_lines_image1(corrected[0], corrected[1], &constraint_lines);
  return HomgOperator2D::lines_to_point(constraint_lines);
}

// @{ TRANSFER: ``Quick and Dirty'' @}

//-----------------------------------------------------------------------------
//
//: For the specified points in image 1/2, return the transferred point in image 3.
//

HomgPoint2D
TriTensor::image3_transfer_qd (const HomgPoint2D& point1, const HomgPoint2D& point2) const
{
  // tr_transfer_point_12_to_3
  vcl_vector<HomgLine2D> constraint_lines(9);
  get_constraint_lines_image3(point1, point2, &constraint_lines);
  return HomgOperator2D::lines_to_point(constraint_lines);
}

//-----------------------------------------------------------------------------
//
//: For the specified points in image 1/3, return the transferred point in image 2.
//

HomgPoint2D
TriTensor::image2_transfer_qd (const HomgPoint2D& point1, const HomgPoint2D& point3) const
{
  vcl_vector<HomgLine2D> constraint_lines(9);
  get_constraint_lines_image2(point1, point3, &constraint_lines);
  return HomgOperator2D::lines_to_point(constraint_lines);
}

//-----------------------------------------------------------------------------
//
//: For the specified points in image 2/3, return the transferred point in image 1.
//

HomgPoint2D
TriTensor::image1_transfer_qd (const HomgPoint2D& point2, const HomgPoint2D& point3) const
{
  vcl_vector<HomgLine2D> constraint_lines(9);
  get_constraint_lines_image1(point2, point3, &constraint_lines);
  return HomgOperator2D::lines_to_point(constraint_lines);
}

//-----------------------------------------------------------------------------
//
//: For the specified lines in image 2/3, return the transferred line in image 1.
//

HomgLine2D
TriTensor::image1_transfer (const HomgLine2D& line2, const HomgLine2D& line3) const
{
  vnl_double_3 l1(0,0,0);
  const vnl_vector<double>& l2 = line2.get_vector();
  const vnl_vector<double>& l3 = line3.get_vector();

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        l1 [i] +=  T(i,j,k) * l2 [j] * l3 [k];

  return HomgLine2D(l1);
}

//-----------------------------------------------------------------------------
//
//: For the specified lines in image 2/3, return the transferred line in image 1.
//

HomgLine2D
TriTensor::image2_transfer (const HomgLine2D& line1, const HomgLine2D& line3) const
{
  return HomgLine2D(vnl_svd<double>(dot3(line3.get_vector())).inverse() * line1.get_vector());
}

//-----------------------------------------------------------------------------
//
//: For the specified lines in image 1/2, return the transferred line in image 3.
//

HomgLine2D
TriTensor::image3_transfer (const HomgLine2D& line1, const HomgLine2D& line2) const
{
  return HomgLine2D(vnl_svd<double>(dot2(line2.get_vector())).inverse() * line1.get_vector());
}

// @{ HOMOGRAPHIES FROM LINES @}

//HMatrix2D TriTensor::get_hmatrix_23(const HomgLine2D& line1)
//{
//  assert(!"implemented");
//  return HMatrix2D(dot1(line1.get_vector()));
//}

//: Return the planar homography between views 3 and 1 induced by line 2
HMatrix2D TriTensor::get_hmatrix_31(const HomgLine2D& line2) const
{
  return HMatrix2D(dot2(line2.get_vector()));
}

//: Return the planar homography between views 2 and 1 induced by line 3
HMatrix2D TriTensor::get_hmatrix_21(const HomgLine2D& line3) const
{
  return HMatrix2D(dot3(line3.get_vector()));
}

// @{ CONTRACTION WITH VECTORS @}

//: @{ Compute ${\tt M}_{jk} = T_{ijk} v_i$. @}
vnl_double_3x3 TriTensor::dot1(const vnl_vector<double>& v) const
{
  vnl_double_3x3 answer; answer.fill(0.0);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        answer(j,k) += v[i] * T(i,j,k);
  return answer;
}

//: @{ Compute ${\tt M}_{ik} = T_{ijk} v_j$. @}
vnl_double_3x3 TriTensor::dot2(const vnl_vector<double>& v) const
{
  vnl_double_3x3 answer; answer.fill(0.0);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        answer(i,k) += v[j] * T(i,j,k);
  return answer;
}

//: @{ Compute ${\tt M}_{ij} = T_{ijk} v_k$. @}
vnl_double_3x3 TriTensor::dot3(const vnl_vector<double>& v) const
{
  vnl_double_3x3 answer; answer.fill(0.0);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        answer(i,j) += v[k] * T(i,j,k);
  return answer;
}

//: @{ Compute ${\tt M}_{kj} = T_{ijk} v_i$. (The transpose of dot1). @}
vnl_double_3x3 TriTensor::dot1t(const vnl_vector<double>& v) const
{
  vnl_double_3x3 answer; answer.fill(0.0);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        answer(k,j) += v[i] * T(i,j,k);
  return answer;
}

//: @{ Compute ${\tt M}_{ki} = T_{ijk} v_j$. @}
vnl_double_3x3 TriTensor::dot2t(const vnl_vector<double>& v) const
{
  vnl_double_3x3 answer; answer.fill(0.0);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        answer(k,i) += v[j] * T(i,j,k);
  return answer;
}

//: @{ Compute ${\tt M}_{ji} = T_{ijk} v_k$. @}
vnl_double_3x3 TriTensor::dot3t(const vnl_vector<double>& v) const
{
  vnl_double_3x3 answer; answer.fill(0.0);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        answer(j,i) += v[k] * T(i,j,k);
  return answer;
}

// @{ COMPOSITION WITH MATRICES @}

// - @{ Contract Tensor axis tensor\_axis with first component of Matrix {\tt M}.
// That is, where $S$ is the result of the operation: \cr
// For tensor\_axis = 1,  Compute $S_{ijk} = T_{pjk} M_{pi}$ \cr
// For tensor\_axis = 2,  Compute $S_{ijk} = T_{ipk} M_{pj}$ \cr
// For tensor\_axis = 3,  Compute $S_{ijk} = T_{ijp} M_{pk}$ @}
TriTensor TriTensor::postmultiply(unsigned tensor_axis, const vnl_matrix<double>& M) const
{
  switch (tensor_axis) {
  case 1: return postmultiply1(M);
  case 2: return postmultiply2(M);
  case 3: return postmultiply3(M);
  default:
    assert(tensor_axis <= 3);
    return *this; // Shut up compiler
  }
}

// - @{Contract Tensor axis tensor\_axis with second component of Matrix {\tt M}.
// That is, where $S$ is the result of the operation: \cr
// For tensor\_axis = 1,  Compute $S_{ijk} = M_{ip} T_{pjk}$ \cr
// For tensor\_axis = 2,  Compute $S_{ijk} = M_{jp} T_{ipk}$ \cr
// For tensor\_axis = 3,  Compute $S_{ijk} = M_{kp} T_{ijp}$ @}
TriTensor TriTensor::premultiply(unsigned tensor_axis, const vnl_matrix<double>& M) const
{
  switch (tensor_axis) {
  case 1: return premultiply1(M);
  case 2: return premultiply2(M);
  case 3: return premultiply3(M);
  default:
    assert(tensor_axis <= 3);
    return *this; // Shut up compiler
  }
}

//: Compute @{$ S_{ijk} = T_{pjk} M_{pi} $@}
TriTensor TriTensor::postmultiply1(const vnl_matrix<double>& M) const
{
  TriTensor S;
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      for(int k = 0; k < 3; ++k) {
        double v = 0;
        for(int p = 0; p < 3; ++p)
          v += T(p,j,k) * M(p,i);
        S(i,j,k) = v;
      }
  return S;
}

//: Compute @{$ S_{ijk} = T_{ipk} M_{pj} $@}
TriTensor TriTensor::postmultiply2(const vnl_matrix<double>& M) const
{
  TriTensor S;
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      for(int k = 0; k < 3; ++k) {
        double v = 0;
        for(int p = 0; p < 3; ++p)
          v += T(i,p,k) * M(p,j);
        S(i,j,k) = v;
      }
  return S;
}

//: Compute @{$ S_{ijk} = T_{ijp} M_{pk} $@}
TriTensor TriTensor::postmultiply3(const vnl_matrix<double>& M) const
{
  TriTensor S;
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      for(int k = 0; k < 3; ++k) {
        double v = 0;
        for(int p = 0; p < 3; ++p)
          v += T(i,j,p) * M(p,k);
        S(i,j,k) = v;
      }
  return S;
}

//: Compute @{$ S_{ijk} = M_{ip} T_{pjk} $@}
TriTensor TriTensor::premultiply1(const vnl_matrix<double>& M) const
{
  TriTensor S;
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      for(int k = 0; k < 3; ++k) {
        double v = 0;
        for(int p = 0; p < 3; ++p)
          v += M(i,p) * T(p,j,k);
        S(i,j,k) = v;
      }
  return S;
}

//: Compute @{$ S_{ijk} = M_{jp} T_{ipk} $@}
TriTensor TriTensor::premultiply2(const vnl_matrix<double>& M) const
{
  TriTensor S;
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      for(int k = 0; k < 3; ++k) {
        double v = 0;
        for(int p = 0; p < 3; ++p)
          v += M(j,p) * T(i,p,k);
        S(i,j,k) = v;
      }
  return S;
}

//: Compute @{$ S_{ijk} = M_{kp} T_{ijp} $@}
TriTensor TriTensor::premultiply3(const vnl_matrix<double>& M) const
{
  TriTensor S;
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      for(int k = 0; k < 3; ++k) {
        double v = 0;
        for(int p = 0; p < 3; ++p)
          v += M(k,p) * T(i,j,p);
        S(i,j,k) = v;
      }
  return S;
}

// INTERNALS---------------------------------------------------------------

// @{ TRANSFORMATION @}

//: @{ The ${\tt C}_{123}$ are line transformation matrices.  If
// $ {\tt C}_v l_v = \hat l_v$ describes the transformation of each image plane under
// a planar homography, and $l_1 = T l_2 l_3$ describes the action of this TriTensor, then
// this routine computes $\hat T$ such that $\hat l_1 = \hat T \hat l_2 \hat l_3$.
// \par Specifically $\hat T = T.\mbox{decondition}(C_1^{-1}, C_2, C_3)$ is the
// transformed tensor.  Note that unless transfer is via Hartley-Sturm, the deconditioned
// tensor will not be covariant with the conditioned one.
// @}
TriTensor TriTensor::condition(const vnl_matrix<double>& line_1_denorm,
                               const vnl_matrix<double>& line_2_norm,
                               const vnl_matrix<double>& line_3_norm) const
{
  return premultiply1(line_1_denorm).postmultiply2(line_2_norm).postmultiply3(line_3_norm);
}

TriTensor TriTensor::decondition(const vnl_matrix<double>& line_1_norm,
                                 const vnl_matrix<double>& line_2_denorm,
                                 const vnl_matrix<double>& line_3_denorm) const
{
  return premultiply1(line_1_norm).postmultiply2(line_2_denorm).postmultiply3(line_3_denorm);
}

//-----------------------------------------------------------------------------
// - Return the 9 lines on which a transferred point ought to lie.
void TriTensor::get_constraint_lines_image3(const HomgPoint2D& p1, const HomgPoint2D& p2, vcl_vector<HomgLine2D>* lines) const
{
  // tr_get_constraint_lines_image3
// Old code assumed points were in image coords and conditioned them,
// deconditioning the lines on return.
  //ma2_static_multiply_3x3_trivec (tritensor_ptr-> corner1_norm_matrix, trivec1_ptr, &mapped_trivec1);
  //ma2_static_multiply_3x3_trivec (tritensor_ptr-> corner2_norm_matrix, trivec2_ptr, &mapped_trivec2);

  // use the same notation as the output of tr_hartley_equation.

  double x1 = p1.get_x();
  double y1 = p1.get_y();
  double z1 = p1.get_w();

  double x2 = p2.get_x();
  double y2 = p2.get_y();
  double z2 = p2.get_w();

  // cout << "CLINES = [" << x1 << " " << y1 << " " << z1 << "; " << x2 << " " << y2 << " " << z2 << "];\n";

  lines->resize(0);


  /* 0 */
  {
    double lx =
      -x1 * x2 * T(0,1,1) + x1 * y2 * T(0,0,1)
      -y1 * x2 * T(1,1,1) + y1 * y2 * T(1,0,1)
      -z1 * x2 * T(2,1,1) + z1 * y2 * T(2,0,1);

    double ly =
      x1 * x2 * T(0,1,0) - x1 * y2 * T(0,0,0) +
      y1 * x2 * T(1,1,0) - y1 * y2 * T(1,0,0) +
      z1 * x2 * T(2,1,0) - z1 * y2 * T(2,0,0);

    double lz = 0;
    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 1 */
  {
    double lx =
      -x1 * x2 * T(0,1,2) + x1 * y2 * T(0,0,2)
      - y1 * x2 * T(1,1,2) + y1 * y2 * T(1,0,2)
      - z1 * x2 * T(2,1,2) + z1 * y2 * T(2,0,2);

    double ly = 0;

    double lz
    = x1 * x2 * T(0,1,0) - x1 * y2 * T(0,0,0)
    + y1 * x2 * T(1,1,0) - y1 * y2 * T(1,0,0)
    + z1 * x2 * T(2,1,0) - z1 * y2 * T(2,0,0);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 2 */
  {
    double lx =
      0;

    double ly =
      -x1 * x2 * T(0,1,2) + x1 * y2 * T(0,0,2)
      -y1 * x2 * T(1,1,2) + y1 * y2 * T(1,0,2)
      -z1 * x2 * T(2,1,2) + z1 * y2 * T(2,0,2);

    double lz =
      x1 * x2 * T(0,1,1) - x1 * y2 * T(0,0,1)
      + y1 * x2 * T(1,1,1) - y1 * y2 * T(1,0,1)
      + z1 * x2 * T(2,1,1) - z1 * y2 * T(2,0,1);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 3 */
  {
    double lx =
      -x1 * x2 * T(0,2,1) + x1 * z2 * T(0,0,1)
      - y1 * x2 * T(1,2,1) + y1 * z2 * T(1,0,1)
      - z1 * x2 * T(2,2,1) + z1 * z2 * T(2,0,1);

    double ly =
      x1 * x2 * T(0,2,0) - x1 * z2 * T(0,0,0)
      + y1 * x2 * T(1,2,0) - y1 * z2 * T(1,0,0)
      + z1 * x2 * T(2,2,0) - z1 * z2 * T(2,0,0);

    double lz = 0;
    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 4 */
  {
    double lx =
      -x1 * x2 * T(0,2,2) + x1 * z2 * T(0,0,2)
      -y1 * x2 * T(1,2,2) + y1 * z2 * T(1,0,2)
      -z1 * x2 * T(2,2,2) + z1 * z2 * T(2,0,2);

    double ly = 0;

    double lz =
      x1 * x2 * T(0,2,0) - x1 * z2 * T(0,0,0) +
      y1 * x2 * T(1,2,0) - y1 * z2 * T(1,0,0) +
      z1 * x2 * T(2,2,0) - z1 * z2 * T(2,0,0);
    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 5 */
  {
    double lx = 0;

    double ly =
      -x1 * x2 * T(0,2,2) + x1 * z2 * T(0,0,2)
      -y1 * x2 * T(1,2,2) + y1 * z2 * T(1,0,2)
      -z1 * x2 * T(2,2,2) + z1 * z2 * T(2,0,2);

    double lz
        = x1 * x2 * T(0,2,1) - x1 * z2 * T(0,0,1)
        + y1 * x2 * T(1,2,1) - y1 * z2 * T(1,0,1)
        + z1 * x2 * T(2,2,1) - z1 * z2 * T(2,0,1);
    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 6 */
  {
    double lx
        = - x1 * y2 * T(0,2,1) + x1 * z2 * T(0,1,1)
        - y1 * y2 * T(1,2,1) + y1 * z2 * T(1,1,1)
        - z1 * y2 * T(2,2,1) + z1 * z2 * T(2,1,1);

    double ly
        = x1 * y2 * T(0,2,0) - x1 * z2 * T(0,1,0)
        + y1 * y2 * T(1,2,0) - y1 * z2 * T(1,1,0)
        + z1 * y2 * T(2,2,0) - z1 * z2 * T(2,1,0);

    double lz = 0;
    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 7 */
  {
    double lx
        = -x1 * y2 * T(0,2,2) + x1 * z2 * T(0,1,2)
        - y1 * y2 * T(1,2,2) + y1 * z2 * T(1,1,2)
        - z1 * y2 * T(2,2,2) + z1 * z2 * T(2,1,2);

    double ly = 0;

    double lz
        = x1 * y2 * T(0,2,0) - x1 * z2 * T(0,1,0)
        + y1 * y2 * T(1,2,0) - y1 * z2 * T(1,1,0)
        + z1 * y2 * T(2,2,0) - z1 * z2 * T(2,1,0);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 8 */
  {
    double lx = 0;

    double ly
        = -x1 * y2 * T(0,2,2) + x1 * z2 * T(0,1,2)
        - y1 * y2 * T(1,2,2) + y1 * z2 * T(1,1,2)
        - z1 * y2 * T(2,2,2) + z1 * z2 * T(2,1,2);

    double lz
        = x1 * y2 * T(0,2,1) - x1 * z2 * T(0,1,1)
        + y1 * y2 * T(1,2,1) - y1 * z2 * T(1,1,1)
        + z1 * y2 * T(2,2,1) - z1 * z2 * T(2,1,1);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  assert(lines->size() == 9);
  if (tt_verbose)
    for(int i = 0; i < 9; ++i)
      vcl_cout << (*lines)[i]<< vcl_endl;

  return;

#if 0
  *trivec3_ptr = ho_trivec_orthog (line_table_ptr);
  ma2_static_multiply_3x3_trivec (_point_invnorm_matrix3, trivec3_ptr, trivec3_ptr);

  // Decondition lines
  if (false)
    for (int line_index = 0; line_index < lines->size(); line_index++)
      ma2_static_multiply_3x3_trivec (_line_invnorm_matrix3, lines[line_index], lines[line_index]);


  ho_triveccam_aspect_lines_to_point (line_table_ptr, trivec3_ptr);
#endif
}

void TriTensor::get_constraint_lines_image2(const HomgPoint2D& p1, const HomgPoint2D& p3, vcl_vector<HomgLine2D>* lines) const
{
  double x1 = p1.get_x();
  double y1 = p1.get_y();
  double z1 = p1.get_w();

  double x3 = p3.get_x();
  double y3 = p3.get_y();
  double z3 = p3.get_w();

  lines->resize(0);

  /* 0 */
  {
    double lx
        = x1 * y3 * T(0,1,0) - x1 * x3 * T(0,1,1)
        + y1 * y3 * T(1,1,0) - y1 * x3 * T(1,1,1)
        + z1 * y3 * T(2,1,0) - z1 * x3 * T(2,1,1);

    double ly
        = - x1 * y3 * T(0,0,0) + x1 * x3 * T(0,0,1)
        - y1 * y3 * T(1,0,0) + y1 * x3 * T(1,0,1)
        - z1 * y3 * T(2,0,0) + z1 * x3 * T(2,0,1);

    double lz = 0;

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 1 */
  {
    double lx
        = x1 * z3 * T(0,1,0) - x1 * x3 * T(0,1,2)
        + y1 * z3 * T(1,1,0) - y1 * x3 * T(1,1,2)
        + z1 * z3 * T(2,1,0) - z1 * x3 * T(2,1,2);

    double ly
        = - x1 * z3 * T(0,0,0) + x1 * x3 * T(0,0,2)
        - y1 * z3 * T(1,0,0) + y1 * x3 * T(1,0,2)
        - z1 * z3 * T(2,0,0) + z1 * x3 * T(2,0,2);

    double lz = 0;

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 2 */
  {
    double lx
        = x1 * z3 * T(0,1,1) - x1 * y3 * T(0,1,2)
        + y1 * z3 * T(1,1,1) - y1 * y3 * T(1,1,2)
        + z1 * z3 * T(2,1,1) - z1 * y3 * T(2,1,2);

    double ly
        = -x1 * z3 * T(0,0,1) + x1 * y3 * T(0,0,2)
        - y1 * z3 * T(1,0,1) + y1 * y3 * T(1,0,2)
        - z1 * z3 * T(2,0,1) + z1 * y3 * T(2,0,2);

    double lz = 0;

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 3 */
  {
    double lx
        = x1 * y3 * T(0,2,0) - x1 * x3 * T(0,2,1)
        + y1 * y3 * T(1,2,0) - y1 * x3 * T(1,2,1)
        + z1 * y3 * T(2,2,0) - z1 * x3 * T(2,2,1);

    double ly = 0;

    double lz
        = -x1 * y3 * T(0,0,0) + x1 * x3 * T(0,0,1)
        - y1 * y3 * T(1,0,0) + y1 * x3 * T(1,0,1)
        - z1 * y3 * T(2,0,0) + z1 * x3 * T(2,0,1);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 4 */
  {
    double lx
        = x1 * z3 * T(0,2,0) - x1 * x3 * T(0,2,2)
        + y1 * z3 * T(1,2,0) - y1 * x3 * T(1,2,2)
        + z1 * z3 * T(2,2,0) - z1 * x3 * T(2,2,2);

    double ly = 0;

    double lz
        = - x1 * z3 * T(0,0,0) + x1 * x3 * T(0,0,2)
        - y1 * z3 * T(1,0,0) + y1 * x3 * T(1,0,2)
        - z1 * z3 * T(2,0,0) + z1 * x3 * T(2,0,2);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 5 */
  {
    double lx
        = x1 * z3 * T(0,2,1) - x1 * y3 * T(0,2,2)
        + y1 * z3 * T(1,2,1) - y1 * y3 * T(1,2,2)
        + z1 * z3 * T(2,2,1) - z1 * y3 * T(2,2,2);

    double ly = 0;

    double lz
        = - x1 * z3 * T(0,0,1) + x1 * y3 * T(0,0,2)
        - y1 * z3 * T(1,0,1) + y1 * y3 * T(1,0,2)
        - z1 * z3 * T(2,0,1) + z1 * y3 * T(2,0,2);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 6 */
  {
    double lx = 0;

    double ly
        = x1 * y3 * T(0,2,0) - x1 * x3 * T(0,2,1)
        + y1 * y3 * T(1,2,0) - y1 * x3 * T(1,2,1)
        + z1 * y3 * T(2,2,0) - z1 * x3 * T(2,2,1);

    double lz
        = -x1 * y3 * T(0,1,0) + x1 * x3 * T(0,1,1)
        - y1 * y3 * T(1,1,0) + y1 * x3 * T(1,1,1)
        - z1 * y3 * T(2,1,0) + z1 * x3 * T(2,1,1);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 7 */
  {
    double lx = 0;

    double ly
        = x1 * z3 * T(0,2,0) - x1 * x3 * T(0,2,2)
        + y1 * z3 * T(1,2,0) - y1 * x3 * T(1,2,2)
        + z1 * z3 * T(2,2,0) - z1 * x3 * T(2,2,2);

    double lz
        = - x1 * z3 * T(0,1,0) + x1 * x3 * T(0,1,2)
        - y1 * z3 * T(1,1,0) + y1 * x3 * T(1,1,2)
        - z1 * z3 * T(2,1,0) + z1 * x3 * T(2,1,2);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 8 */
  {
    double lx = 0;

    double ly
        = x1 * z3 * T(0,2,1) - x1 * y3 * T(0,2,2)
        + y1 * z3 * T(1,2,1) - y1 * y3 * T(1,2,2)
        + z1 * z3 * T(2,2,1) - z1 * y3 * T(2,2,2);

    double lz
        = - x1 * z3 * T(0,1,1) + x1 * y3 * T(0,1,2)
        - y1 * z3 * T(1,1,1) + y1 * y3 * T(1,1,2)
        - z1 * z3 * T(2,1,1) + z1 * y3 * T(2,1,2);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  // awf removed deconditioning
}

void TriTensor::get_constraint_lines_image1(const HomgPoint2D& p2, const HomgPoint2D& p3, vcl_vector<HomgLine2D>* lines) const
{
  // use the same notation as the output of tr_hartley_equation.

  double x2 = p2.get_x();
  double y2 = p2.get_y();
  double z2 = p2.get_w();

  double x3 = p3.get_x();
  double y3 = p3.get_y();
  double z3 = p3.get_w();

  lines->resize(0);

  /* 0 */

  {
    double lx
      = x2 * y3 * T(0,1,0)
      - y2 * y3 * T(0,0,0)
      - x2 * x3 * T(0,1,1)
      + y2 * x3 * T(0,0,1);

    double ly
      = x2 * y3 * T(1,1,0)
      - y2 * y3 * T(1,0,0)
      - x2 * x3 * T(1,1,1)
      + y2 * x3 * T(1,0,1);

    double lz
      = x2 * y3 * T(2,1,0)
      - y2 * y3 * T(2,0,0)
      - x2 * x3 * T(2,1,1)
      + y2 * x3 * T(2,0,1);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 1 */
  {
    double lx
      = x2 * z3 * T(0,1,0)
      - y2 * z3 * T(0,0,0)
      - x2 * x3 * T(0,1,2)
      + y2 * x3 * T(0,0,2);

    double ly
      = x2 * z3 * T(1,1,0)
      - y2 * z3 * T(1,0,0)
      - x2 * x3 * T(1,1,2)
      + y2 * x3 * T(1,0,2);

    double lz
      = x2 * z3 * T(2,1,0)
      - y2 * z3 * T(2,0,0)
      - x2 * x3 * T(2,1,2)
      + y2 * x3 * T(2,0,2);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 2 */
  {
    double lx
      = x2 * z3 * T(0,1,1)
      - y2 * z3 * T(0,0,1)
      - x2 * y3 * T(0,1,2)
      + y2 * y3 * T(0,0,2);

    double ly
      = x2 * z3 * T(1,1,1)
      - y2 * z3 * T(1,0,1)
      - x2 * y3 * T(1,1,2)
      + y2 * y3 * T(1,0,2);

    double lz
      = x2 * z3 * T(2,1,1)
      - y2 * z3 * T(2,0,1)
      - x2 * y3 * T(2,1,2)
      + y2 * y3 * T(2,0,2);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 3 */
  {
    double lx
      = x2 * y3 * T(0,2,0)
      - z2 * y3 * T(0,0,0)
      - x2 * x3 * T(0,2,1)
      + z2 * x3 * T(0,0,1);

    double ly
      = x2 * y3 * T(1,2,0)
      - z2 * y3 * T(1,0,0)
      - x2 * x3 * T(1,2,1)
      + z2 * x3 * T(1,0,1);

    double lz
      = x2 * y3 * T(2,2,0)
      - z2 * y3 * T(2,0,0)
      - x2 * x3 * T(2,2,1)
      + z2 * x3 * T(2,0,1);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 4 */
  {
    double lx
      = x2 * z3 * T(0,2,0)
      - z2 * z3 * T(0,0,0)
      - x2 * x3 * T(0,2,2)
      + z2 * x3 * T(0,0,2);

    double ly
      = x2 * z3 * T(1,2,0)
      - z2 * z3 * T(1,0,0)
      - x2 * x3 * T(1,2,2)
      + z2 * x3 * T(1,0,2);

    double lz
      = x2 * z3 * T(2,2,0)
      - z2 * z3 * T(2,0,0)
      - x2 * x3 * T(2,2,2)
      + z2 * x3 * T(2,0,2);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 5 */
  {
    double lx
      = x2 * z3 * T(0,2,1)
      - z2 * z3 * T(0,0,1)
      - x2 * y3 * T(0,2,2)
      + z2 * y3 * T(0,0,2);

    double ly
      = x2 * z3 * T(1,2,1)
      - z2 * z3 * T(1,0,1)
      - x2 * y3 * T(1,2,2)
      + z2 * y3 * T(1,0,2);

    double lz
      = x2 * z3 * T(2,2,1)
      - z2 * z3 * T(2,0,1)
      - x2 * y3 * T(2,2,2)
      + z2 * y3 * T(2,0,2);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 6 */
  {
    double lx
      = y2 * y3 * T(0,2,0)
      - z2 * y3 * T(0,1,0)
      - y2 * x3 * T(0,2,1)
      + z2 * x3 * T(0,1,1);

    double ly
      = y2 * y3 * T(1,2,0)
      - z2 * y3 * T(1,1,0)
      - y2 * x3 * T(1,2,1)
      + z2 * x3 * T(1,1,1);

    double lz
      = y2 * y3 * T(2,2,0)
      - z2 * y3 * T(2,1,0)
      - y2 * x3 * T(2,2,1)
      + z2 * x3 * T(2,1,1);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 7 */
  {
    double lx
      = y2 * z3 * T(0,2,0)
      - z2 * z3 * T(0,1,0)
      - y2 * x3 * T(0,2,2)
      + z2 * x3 * T(0,1,2);

    double ly
      = y2 * z3 * T(1,2,0)
      - z2 * z3 * T(1,1,0)
      - y2 * x3 * T(1,2,2)
      + z2 * x3 * T(1,1,2);

    double lz
      = y2 * z3 * T(2,2,0)
      - z2 * z3 * T(2,1,0)
      - y2 * x3 * T(2,2,2)
      + z2 * x3 * T(2,1,2);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  /* 8 */
  {
    double lx
      = y2 * z3 * T(0,2,1)
      - z2 * z3 * T(0,1,1)
      - y2 * y3 * T(0,2,2)
      + z2 * y3 * T(0,1,2);

    double ly
      = y2 * z3 * T(1,2,1)
      - z2 * z3 * T(1,1,1)
      - y2 * y3 * T(1,2,2)
      + z2 * y3 * T(1,1,2);

    double lz
      = y2 * z3 * T(2,2,1)
      - z2 * z3 * T(2,1,1)
      - y2 * y3 * T(2,2,2)
      + z2 * y3 * T(2,1,2);

    lines->push_back(HomgLine2D(lx, ly, lz));
  }

  // awf removed deconditioning
}


// @{ INPUT/OUTPUT @}

//-----------------------------------------------------------------------------
//: Read from ASCII vcl_istream
vcl_istream& operator >> (vcl_istream& s, TriTensor& T)
{
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      for(int k = 0; k < 3; ++k)
        s >> T(i,j,k);
  return s;
}

//-----------------------------------------------------------------------------
//: Print in ASCII to vcl_ostream
vcl_ostream& operator << (vcl_ostream& s, const TriTensor& T)
{
  for(int i = 0; i < 3; ++i) {
    for(int j = 0; j < 3; ++j) {
      for(int k = 0; k < 3; ++k)
        vul_printf(s, "%20.16e ", T(i,j,k));
      s << vcl_endl;
    }
    s << vcl_endl;
  }
  return s;
}

#include <vnl/vnl_matops.h> // use vnl_matlab_print.h for pretty printing
struct Column3x3 : public vnl_double_3x3 {
  Column3x3(const vnl_vector<double>& v1, const vnl_vector<double>& v2, const vnl_vector<double>& v3) {
    (*this)(0,0) = v1[0];   (*this)(0,1) = v2[0];  (*this)(0,2) = v3[0];
    (*this)(1,0) = v1[1];   (*this)(1,1) = v2[1];  (*this)(1,2) = v3[1];
    (*this)(2,0) = v1[2];   (*this)(2,1) = v2[2];  (*this)(2,2) = v3[2];
  }
};

// @{ FUNDAMENTAL MATRICES AND EPIPOLES @}

//: Compute and cache the two epipoles from image 1.
bool TriTensor::compute_epipoles()
{
  vnl_double_3x3 T1 = dot1(vnl_double_3(1,0,0));
  vnl_double_3x3 T2 = dot1(vnl_double_3(0,1,0));
  vnl_double_3x3 T3 = dot1(vnl_double_3(0,0,1));

  vnl_svd<double> svd1(T1);
  vnl_double_3 u1 = svd1.nullvector();
  vnl_double_3 v1 = svd1.left_nullvector();

  vnl_svd<double> svd2(T2);
  vnl_double_3 u2 = svd2.nullvector();
  vnl_double_3 v2 = svd2.left_nullvector();

  vnl_svd<double> svd3(T3);
  vnl_double_3 u3 = svd3.nullvector();
  vnl_double_3 v3 = svd3.left_nullvector();

  vnl_double_3x3 V;
  V(0,0) = v1[0];   V(0,1) = v2[0];  V(0,2) = v3[0];
  V(1,0) = v1[1];   V(1,1) = v2[1];  V(1,2) = v3[1];
  V(2,0) = v1[2];   V(2,1) = v2[2];  V(2,2) = v3[2];

  vnl_svd<double> svdv(V);

  delete _e12;
  _e12 = new HomgPoint2D(svdv.left_nullvector());

  vnl_double_3x3 U;
  U(0,0) = u1[0];   U(0,1) = u2[0];  U(0,2) = u3[0];
  U(1,0) = u1[1];   U(1,1) = u2[1];  U(1,2) = u3[1];
  U(2,0) = u1[2];   U(2,1) = u2[2];  U(2,2) = u3[2];

  vnl_svd<double> svdu(U);

  delete _e13;
  _e13 = new HomgPoint2D(svdu.left_nullvector());

  return _e12 && _e13;
}

//: Return epipoles e2 and e3, from image 1 into images 2 and 3 respectively.
// There is no longer any need to use this routine as they are cached in the TriTensor.
bool TriTensor::get_epipoles(HomgPoint2D* e2, HomgPoint2D* e3) const
{
  // Check if cached.
  if (!_e12)
    MUTABLE_CAST(compute_epipoles());

  if (e2) *e2 = *_e12;
  if (e3) *e3 = *_e13;
  return true;
}

//: Return epipole12.
HomgPoint2D TriTensor::get_epipole_12() const
{
  get_epipoles(0,0);
  return *_e12;
}

//: Return epipole13.
HomgPoint2D TriTensor::get_epipole_13() const
{
  get_epipoles(0,0);
  return *_e13;
}

//: Return F12, the fundamental matrix between images 1 and 2
FMatrix TriTensor::get_fmatrix_12() const
{
  get_epipoles(0,0);
  return get_fmatrix_12(*_e12, *_e13);
}

//: Return F13, the fundamental matrix between images 1 and 3
FMatrix TriTensor::get_fmatrix_13() const
{
  get_epipoles(0,0);
  return get_fmatrix_13(*_e12, *_e13);
}

// next two routines retired as caching introduced.  awf, aug97
// This empty doc comment is to make gentex place these two together.
//  @{ \vspace{-1.5\baselineskip} @}
FMatrix TriTensor::get_fmatrix_13(const HomgPoint2D& e2, const HomgPoint2D& e3) const
{
  return vnl_cross_product_matrix(e3.get_vector()) * dot2(e2.get_vector()).transpose();
}

//  More efficient than above if both F matrices are required.  The epipoles ought
// to be computed once and the passed twice to get_fmatrix_12 and get_fmatrix_13.
FMatrix TriTensor::get_fmatrix_12(const HomgPoint2D& e2, const HomgPoint2D& e3) const
{
  return vnl_cross_product_matrix(e2.get_vector()) * dot3(e3.get_vector()).transpose();
}

FMatrix TriTensor::compute_fmatrix_23() const
{
  PMatrix P2, P3;
  compute_P_matrices(&P2, &P3);
  return FMatrix(P2, P3);
}

//: Return a manifold-projector for the Fundamental matrix between 1 and 2.
// The projector is cached until the next time T is changed.
FManifoldProject* TriTensor::get_fmp12() const
{
  // If not cached, compute it.
  if (!_fmp12) MUTABLE_CAST(_fmp12) = new FManifoldProject(get_fmatrix_12());
  return _fmp12;
}

//: Return a manifold-projector as above, between 1 and 3.
FManifoldProject* TriTensor::get_fmp13() const
{
  // If not cached, compute it.
  if (!_fmp13) MUTABLE_CAST(_fmp13) = new FManifoldProject(get_fmatrix_13());
  return _fmp13;
}

//: Return a manifold-projector as above, between 2 and 3.
FManifoldProject* TriTensor::get_fmp23() const
{
  // If not cached, compute it.
  if (!_fmp23) {
    // Need to get FMatrix 23
    PMatrix P2;
    PMatrix P3;
    compute_P_matrices(&P2, &P3);
    FMatrix f23(P2,P3);

    MUTABLE_CAST(_fmp23) = new FManifoldProject(f23);
  }
  return _fmp23;
}

//: Compute one of the family of P matrix triplets consistent with this T
void TriTensor::compute_P_matrices(const vnl_vector<double>& x, double alpha, double beta, PMatrix* P2, PMatrix* P3) const
{
  HomgPoint2D e2 = get_epipole_12();
  HomgPoint2D e3 = get_epipole_13();

  vnl_double_3x3 Te3 = dot3t(e3);
  vnl_double_3x3 TTe2 = dot2t(e2);

  MATLABPRINT((vnl_matrix<double> const&/*2.7*/)Te3);
  MATLABPRINT((vnl_matrix<double> const&/*2.7*/)TTe2);

  vnl_double_3x3 M = vnl_identity_3x3() - OuterProduct3x3(e3,e3);

  vnl_double_3x3 B0 = -M * TTe2;

  vnl_double_3x3 DIFF = B0 + TTe2 - OuterProduct3x3(e3, vnl_transpose(TTe2)*e3);
  double diffmag = DIFF.fro_norm();
  if (diffmag > 1e-12) {
    vcl_cerr << "TriTensor::compute_P_matrices: DIFF = " << DIFF << vcl_endl;
  }

  vnl_double_3x3& A0 = Te3;

  // P1 = [I O];
  //P2 = [A0 + e2 * x(:)' , beta*e2];
  //P3 = [B0 + e3 * x(:)' , alpha*e3 ];

  P2->set(A0 + OuterProduct3x3(e2, x), beta*e2);
  P3->set(B0 + OuterProduct3x3(e3, x), alpha*e3);

  vcl_cerr << *P2 << vcl_endl;
  vcl_cerr << *P3 << vcl_endl;

  // Check
  this->check_equal_up_to_scale(TriTensor(*P2, *P3));
}

struct maxabs {
  int i, j, k;
  double maxval;
  maxabs(const TriTensor& T);
};

maxabs::maxabs(const TriTensor& T)
{
  maxval = 0;
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      for(int k = 0; k < 3; ++k) {
        double v = vcl_fabs(T(i,j,k));
        if (v >= maxval) {
          maxval = v;
          this->i = i;
          this->j = j;
          this->k = k;
        }
      }
}

static bool check_same(const TriTensor& T1, const TriTensor& T2) {
  maxabs m1(T1);
  double scale1 = 1/m1.maxval;
  double scale2 = 1/vcl_fabs(T2(m1.i,m1.j,m1.k));

  double rms = 0;
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      for(int k = 0; k < 3; ++k) {
        double d = T1(i,j,k)*scale1 - T2(i,j,k) * scale2;
        rms += d*d;
      }

  rms /= 27;

  if (rms > 1e-15) {
    vcl_cerr << "check_same: different TriTensors\n";
    vcl_cerr << "T1 = " << vcl_endl << T1;
    vcl_cerr << "T2 = " << vcl_endl << T2;
    return false;
  }

  return true;
}

//: Check that another trifocal tensor is equal to this one up to scale.
// Finds largest component of this, scales both tritensors so that this
// component is one, and checks that fronorm of difference is small.
// Prints a message if not.
bool TriTensor::check_equal_up_to_scale(const TriTensor& that) const
{
  return check_same(*this, that);
}
