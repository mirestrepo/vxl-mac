// This is gel/vmal/vmal_rectifier.cxx
#include "vmal_rectifier.h"
//:
//  \file

#include <vmal/vmal_convert.h>
#include <vcl_cmath.h> // atan2()
#include <vnl/algo/vnl_svd.h>

#include <vnl/algo/vnl_determinant.h>
//#include <vcl_cmath.h>
#include <mvl/FMatrix.h>
#include <mvl/FMatrixComputeLinear.h>


vmal_rectifier::vmal_rectifier()
{
  lines0_p_=NULL;
  lines0_q_=NULL;
  lines1_p_=NULL;
  lines1_q_=NULL;
  points0_=NULL;
  points1_=NULL;
}

vmal_rectifier::vmal_rectifier(vmal_multi_view_data_vertex_sptr mvd_vertex,
                               vmal_multi_view_data_edge_sptr mvd_edge,
                               int ima_height, int ima_width) :
  is_f_compute_(false)
{
  lines0_p_=NULL;
  lines0_q_=NULL;
  lines1_p_=NULL;
  lines1_q_=NULL;
  points0_=NULL;
  points1_=NULL;

  if ((mvd_vertex->get_nb_views()>1) && (mvd_edge->get_nb_views()>1))
  {
    // Prescale the points
    vcl_vector<vtol_edge_2d_sptr> tmp_lines0;
    vcl_vector<vtol_edge_2d_sptr> tmp_lines1;

    mvd_edge->get(0,1,tmp_lines0,tmp_lines1);
    numpoints_=tmp_lines0.size();

    convert_lines_double_3(tmp_lines0, lines0_p_, lines0_q_);
    convert_lines_double_3(tmp_lines1, lines1_p_, lines1_q_);


    vcl_vector<vtol_vertex_2d_sptr> tmp_points0;
    vcl_vector<vtol_vertex_2d_sptr> tmp_points1;

    mvd_vertex->get(0,1,tmp_points0,tmp_points1);

    convert_points_double_3(tmp_points0, points0_);
    convert_points_double_3(tmp_points1, points1_);
    height_=ima_height;
    width_=ima_width;
  }
}

// Constructor for dealing with just matched points

vmal_rectifier::vmal_rectifier(vcl_vector< vnl_vector<double> >* pts0,
                               vcl_vector< vnl_vector<double> >* pts1,
                               int ima_height, int ima_width) :
  is_f_compute_(false)
{
  lines0_p_=NULL;
  lines0_q_=NULL;
  lines1_p_=NULL;
  lines1_q_=NULL;
  points0_=NULL;
  points1_=NULL;

  height_=ima_height;
  width_=ima_width;
  numpoints_ = pts0->size();

  // put the points in the proper buffers...
  points0_ = new vnl_double_3[numpoints_];
  points1_ = new vnl_double_3[numpoints_];
  vcl_vector< vnl_vector<double> >::iterator vit0 = pts0->begin();
  vcl_vector< vnl_vector<double> >::iterator vit1 = pts1->begin();
  for (int i=0; i<numpoints_; i++) {
    points0_[i][0] = (*vit0).x();
    points0_[i][1] = (*vit0).y();
    points0_[i][2] = 1;
    points1_[i][0] = (*vit1).x();
    points1_[i][1] = (*vit1).y();
    points1_[i][2] = 1;
    vit0++;
    vit1++;
  }
  
}



vmal_rectifier::~vmal_rectifier()
{
  if (points0_!=NULL) delete [] points0_;
  if (points1_!=NULL) delete [] points1_;
  if (lines0_p_!=NULL) delete [] lines0_p_;
  if (lines0_q_!=NULL) delete [] lines0_q_;
  if (lines1_p_!=NULL) delete [] lines1_p_;
  if (lines1_q_!=NULL) delete [] lines1_q_;
}

void vmal_rectifier::rectification_matrix(vnl_double_3x3& H0,
                                          vnl_double_3x3& H1)
{
  if (!is_f_compute_)
  {
    is_f_compute_=true;
    vcl_vector<HomgPoint2D> v_points0;
    vcl_vector<HomgPoint2D> v_points1;
    for (int i=0;i<numpoints_;i++)
    {
      HomgPoint2D tmp_point0(points0_[i][0],points0_[i][1]);
      HomgPoint2D tmp_point1(points1_[i][0],points1_[i][1]);
      v_points0.push_back(tmp_point0);
      v_points1.push_back(tmp_point1);
    }

    FMatrixComputeLinear tmp_fcom;
    FMatrix tmp_f;
    tmp_fcom.compute(v_points0,v_points1,& tmp_f);
    tmp_f.get(&F12_.as_ref().non_const());

    HomgPoint2D epi1;
    HomgPoint2D epi2;

    tmp_f.get_epipoles (&epi1, &epi2);

    double x1,y1;
    double x2,y2;
    epi1.get_nonhomogeneous(x1,y1);
    epi2.get_nonhomogeneous(x2,y2);

    vnl_double_3 tmp_epi1;
    tmp_epi1[0]=x1;
    tmp_epi1[1]=y1;
    tmp_epi1[2]=1.0;

    vnl_double_3 tmp_epi2;
    tmp_epi2[0]=x2;
    tmp_epi2[1]=y2;
    tmp_epi2[2]=1.0;

    epipoles_.push_back(tmp_epi1);
    epipoles_.push_back(tmp_epi2);
  }
  bool affine=false;
  int out_height;
  int out_width;
  int sweeti=(int)(width_/2);
  int sweetj=(int)(height_/2);

  compute_joint_epipolar_transform_new(
    points0_,       // Points in one view
    points1_,       // Points in the other view
    numpoints_,     // Number of matched points
    H0_, H1_,       // The matrices to be returned
    height_, width_,// Dimensions of the input images
    out_height, out_width,
    sweeti, sweetj, // Sweet spot in the first image
    affine);
  H0=H0_;
  H1=H1_;
  // vnl_double_3 p1=H0*epipoles_[0], p2=H1*epipoles_[1], p3=H0*points0_[0], p4=H1*points1_[0];
}

//In this case (3 cameras), we assume that the first camera matrix is equal
//to P=[I|0]. So epi1 corresponds to e', i.e. the epipole of the second image and
//epi2 corresponds to e", i.e. the epipole of the third image in relation to the
//first image.
void vmal_rectifier::set_tritensor(TriTensor &tri)
{
  //this method is helpful when using the vmal_projective_reconstruction class
  //that compute the tritensor.
  is_f_compute_=true;
  tritensor_=tri;

  HomgPoint2D epi12=tri.get_epipole_12();
  FMatrix F12(tri.get_fmatrix_12());

  double x,y;
  epi12.get_nonhomogeneous(x,y);

  vnl_double_3 tmp_epi;
  tmp_epi[0]=x;
  tmp_epi[1]=y;
  tmp_epi[2]=1.0;

  epipoles_.push_back(tmp_epi);
  F12.get(&F12_.as_ref().non_const());
}

void vmal_rectifier::compute_joint_epipolar_transform_new (
  vnl_double_3* points0,  // Points in one view
  vnl_double_3* points1,  // Points in the other view
  int numpoints,          // Number of matched points
  vnl_double_3x3 &H0, vnl_double_3x3 &H1,  // The matrices to be returned
  int in_height, int in_width, // Dimensions of the input images
  int &out_height, int &out_width,
  double sweeti, double sweetj,// Sweet spot in the first image
  bool affine // = FALSE
)
{
   // Compute the pair of epipolar transforms to rectify matched points
   compute_initial_joint_epipolar_transforms (
  points0, points1, numpoints, H0, H1, sweeti, sweetj, affine);

   // Apply the affine correction
   apply_affine_correction (points0, points1, numpoints, H0, H1);
#if 0
   // Next set the range so that the overlap is properly placed
   center_overlap_region (H0, im1_roi, H1, im2_roi, output_roi);
#endif
   // Rotate through 90 degrees
   rectify_rotate90 (out_height,out_width, H0, H1);
#if 0
   // Also, rotate a further 180 degrees if necessary
   conditional_rectify_rotate180 (output_roi, H0, H1);
#endif
   vcl_cerr << "vmal_rectifier::compute_joint_epipolar_transform_new() not yet fully implemented\n";
}

void vmal_rectifier::compute_initial_joint_epipolar_transforms (
   vnl_double_3* points0,       // Points in one view
   vnl_double_3* points1,       // Points in the other view
   int numpoints,               // Number of matched points
   vnl_double_3x3 &H0, vnl_double_3x3 &H1,  // The matrices to be returned
   double sweeti, double sweetj,// Sweet spot in the first image
   bool affine // = FALSE
   )
{
   // Compute the pair of epipolar transforms to rectify matched points
   // This does a minimally distorting correction to H0 and matches it with H1.

  if (is_f_compute_)
  {
    //the epipoles are already set, we don't have to compute the
    //the fundamental matrix.
    if ( !compute_initial_joint_epipolar_transforms (F12_,sweeti, sweetj, H0, H1))
    {
      // Error message and exit
      vcl_cerr<<"Computation of epipolar transform failed\n";
    }
  }
  else // TODO
  {
    vcl_cerr << "vmal_rectifier::compute_initial_joint_epipolar_transforms() not yet fully implemented\n";
#if 0
    // First of all compute the Q matrix
    rhMatrix Q(3, 3);

    // Compute and refine the estimate of the Q matrix
    if (affine)
    {
    // We do not call affine_optimum_solveQmatrix right now,
    // since it is faulty, because the triangulation routine is bad.
      affine_solveQmatrix (u1, v1, u2, v2, numpoints, Q);
      checkQmatrix (u1, v1, u2, v2, numpoints, Q);
    }
    else
    {
      solveQmatrix (u1, v1, u2, v2, numpoints, Q);
      refine_Q_matrix (u1, v1, u2, v2, numpoints, Q);
      checkQmatrix (u1, v1, u2, v2, numpoints, Q);
    }


    // Compute the epipolar transforms.
    if ( !compute_initial_joint_epipolar_transforms (Q, sweeti, sweetj, H0, H1))
    {
      // Error message and exit
      error_message ("Computation of epipolar transform failed\n");
      bail_out (2);
    }
#endif
  }
}

//: Computes a pair of transformation matrices for an image pair that will define the joint epipolar projection.
// This does a minimally distortion-free correction to H0 and then
// gets a matching H0.

int vmal_rectifier::compute_initial_joint_epipolar_transforms (
  const vnl_double_3x3 &Q,
  double ci, double cj,   // Position of reference point in first image
  vnl_double_3x3 &H0,     // The first transformation matrix computed
  vnl_double_3x3 &H1      // second transfrmtion matrix to be computed
  )
{
  // First get the epipole e'
  vnl_double_3 p1 = epipoles_[0];

  // Next, compute the mapping H' for the second image

  // First of all, translate the centre pixel to 0, 0
  H0[0][0] = 1.0; H0[0][1] = 0.0; H0[0][2] = -ci;
  H0[1][0] = 0.0; H0[1][1] = 1.0; H0[1][2] = -cj;
  H0[2][0] = 0.0; H0[2][1] = 0.0; H0[2][2] = 1.0;

  // Translate the epipole as well
  p1 = H0 * p1;

  // Make sure that the epipole is not at the origin
  if (p1[0] == 0.0 && p1[1] == 0.0)
  {
    vcl_cerr<<"Error : Epipole is at image center\n";
    return 0;
  }

  // Next determine a rotation that will send the epipole to (1, 0, x)
  double theta = vcl_atan2 (p1[1], p1[0]);
  double c = vcl_cos (theta);
  double s = vcl_sin (theta);

  vnl_double_3x3 T;

  T[0][0] =   c; T[0][1] =   s; T[0][2] = 0.0;
  T[1][0] =  -s; T[1][1] =   c; T[1][2] = 0.0;
  T[2][0] = 0.0; T[2][1] = 0.0; T[2][2] = 1.0;

    // Multiply things out
  H0 =  T * H0;
  p1 = T * p1;
  vnl_double_3 ep1=H0*epipoles_[0];

  // Now send the epipole to infinity
  double x = p1[2]/p1[0];

  vnl_double_3x3 E;
  E[0][0] = 1.0; E[0][1] = 0.0; E[0][2] = 0.0;
  E[1][0] = 0.0; E[1][1] = 1.0; E[1][2] = 0.0;
  E[2][0] =  -x; E[2][1] = 0.0; E[2][2] = 1.0;

  // Multiply things out.  Put the result in H0
  H0 = E * H0;
  ep1=H0*epipoles_[0];
  // Next compute the initial 2x
  H1 = matching_transform (Q.transpose(), H0);

  // Return 1 value
  return 1;
}

vnl_double_3x3 vmal_rectifier::matching_transform (
  const vnl_double_3x3 &Q,
  const vnl_double_3x3 &H1
  )
   {
   // Computes a transform compatible with H1.  It is assumed that
   // H1 maps the epipole p2 to infinity (1, 0, 0)
   vnl_double_3x3 S, M;
   factor_Q_matrix_SR (Q, M, S);

   // Compute the matching transform
   vnl_double_3x3 H0 = H1 * M;
   return H0;
}

//: This routine comes up with a factorization of Q into R S
void vmal_rectifier::factor_Q_matrix_SR (
        const vnl_double_3x3 &Q,// 3 x 3 matrix
        vnl_double_3x3 &R,      // non-singular matrix
        vnl_double_3x3 &S)      // Skew-symmetric part
{
  double r, s;            // The two singular values

  // First do the singular value decomposition of Q

  vnl_svd<double> SVD(Q);
  vnl_double_3x3 U(SVD.U());
  vnl_double_3x3 V(SVD.V());

  vnl_double_3x3 E(0.0);
  vnl_double_3x3 Z(0.0);
  E[1][0]=E[2][2]=1;
  E[0][1]=-1;
  Z[0][1]=1;Z[1][0]=-1;

  r = SVD.W(0);
  s = SVD.W(1);

  // Transpose V, since we will need to multiply by it
  vnl_double_3x3 Vt = V.transpose();
  vnl_double_3x3 Ut = U.transpose();

  // Define RS
  vnl_double_3x3 RS;
  RS[0][0] =  r; RS[0][1] =  0; RS[0][2] =  0;
  RS[1][0] =  0; RS[1][1] =  s; RS[1][2] =  0;
  RS[2][0] =  0; RS[2][1] =  0; RS[2][2] =  s;

  // Construct the result
  // First of all the R matrix
  R=(U * E * RS * Vt);

  // Next the S matrix
  S=(U * Z * Ut);
}

vnl_double_3x3 vmal_rectifier::affine_correction (
   vnl_double_3* points0,
   vnl_double_3* points1,
   int numpoints,
   const vnl_double_3x3 &H0,
   const vnl_double_3x3 &H1)
  {
   // Correct according to an affine transformation
   // Finds the correction to H1 that would make it closest to H0

  // Matrices for a linear least-squares problem
  vnl_matrix<double> E(numpoints, 3);
  vnl_vector<double> x(numpoints);
   // Fill out the equations
   for (int i=0; i<numpoints; i++)
   {
      // Compute the transformed points
      vnl_double_3 u2hat = H1 * points1[i];
      double uu2 = u2hat[0]/u2hat[2];
      double vv2 = u2hat[1]/u2hat[2];

      vnl_double_3 u1hat = H0 * points0[i];
      double uu1 = u1hat[0]/u1hat[2];

      // Fill out the equation
      E[i][0] = uu2;
      E[i][1] = vv2;
      E[i][2] = 1.0;
      x[i] = uu1;
   }

   // Now solve the equations
   vnl_svd<double> SVD(E);
   vnl_double_3 a=SVD.solve(x);
  // rhVector a = lin_solve(E, x);

   // Now, make up a matrix A to do the transformation
   vnl_double_3x3 A;
   A.set_identity();
   A[0][0] = a[0];  A[0][1] = a[1];  A[0][2] = a[2];

   // This is the affine correction matrix to be returned
   return A;
}

void vmal_rectifier::apply_affine_correction (
   vnl_double_3* points0,       // Points in one view
   vnl_double_3* points1,       // Points in the other view
   int numpoints,               // Number of matched points
   vnl_double_3x3 &H0, vnl_double_3x3 &H1   // The matrices to be returned
   )
{
   // Correct H0 so that it matches with H1 best on the points given
   vnl_double_3x3 A = affine_correction (points0, points1, numpoints, H0, H1);

   // Now correct H0
   H1 = A * H1;

   // Make both H0 and H1 have positive determinant)
   if (vnl_determinant (H1) < 0.0) H1 = -H1;
}

void vmal_rectifier::rectify_rotate90 (
  int &height, int &width,
  vnl_double_3x3 &H0,
  vnl_double_3x3 &H1
  )
{
   // Make a correction so that the horizontal lines come out horizontal
   // instead of vertical

   // Define a matrix for rotating the images
   vnl_double_3x3 R;
   R[0][0] = 0.0;  R[0][1] = -1.0; R[0][2] = height;
   R[1][0] = 1.0;  R[1][1] =  0.0; R[1][2] = 0.0;
   R[2][0] = 0.0;  R[2][1] =  0.0; R[2][2] = 1.0;

   // Multiply the matrices
   H0 = R * H0;
   H1 = R * H1;

   // Now return the output image dimensions
   int t = width; width = height; height = t;
}
