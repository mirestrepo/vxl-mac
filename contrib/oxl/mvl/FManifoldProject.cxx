#ifdef __GNUC__
#pragma implementation
#endif

#include "FManifoldProject.h"

#include <vcl/vcl_cstdlib.h>

#include <vnl/vnl_matlab_print.h>
#include <vnl/vnl_double_2x2.h>
#include <vnl/vnl_double_4x4.h>
#include <vnl/vnl_double_4.h>
#include <vnl/algo/vnl_cholesky.h>
#include <vnl/algo/vnl_symmetric_eigensystem.h>
#include <vnl/algo/vnl_rpoly_roots.h>
#include <vnl/vnl_real_polynomial.h>
#include <vnl/vnl_transpose.h>
#include <vnl/algo/vnl_svd.h>
#include <vnl/vnl_diag_matrix.h>

#include <mvl/HomgPoint2D.h>
#include <mvl/FMatrix.h>
#include <mvl/HomgOperator2D.h>

// -- Construct an FManifoldProject object which will
// use the given F to correct point pairs.
FManifoldProject::FManifoldProject(const FMatrix& Fobj):
  _coeffs(7)
{
  set_F(Fobj);
}

// -- Construct an FManifoldProject object with the intention
// of later setting its F.
FManifoldProject::FManifoldProject():
  _coeffs(7)
{
}

// -- Use the given F to correct point pairs.
void FManifoldProject::set_F(const FMatrix& Fobj)
{
  F_ = Fobj.get_matrix();

  F_.assert_finite();
  
  // 2x2 zero
  vnl_double_2x2 O(0.0);

  // Top left corner of F
  vnl_double_2x2 f22 = F_.extract(2,2);

  // A = 0.5*[O f22'; f22 O];
  A_.fill(0.0);
  A_.update(0.5*f22.transpose(), 0, 2);
  A_.update(0.5*f22, 2, 0);

  vnl_double_4 b(F_(2,0), F_(2,1), F_(0,2), F_(1,2));

  double c = F_(2,2);

  // Compute eig(A) to translate and rotate the quadric
  vnl_symmetric_eigensystem<double>  eig(A_);

  //cerr << vnl_svd<double>(F_);
  //MATLABPRINT(F_);
  //MATLABPRINT(eig.D);
  
  // If all eigs are 0, had an affine F
  _affine_F = eig.D(3,3) < 1e-6;
  if (_affine_F) {
    ///cerr << "FManifoldProject: Affine F = " << F_ << endl;
    double s = 1.0 / b.magnitude();
    t_ = b * s;
    d_[0] = c * s;
  }
  else {

    // Translate Quadric so that b = 0. (Translates to the epipoles)
    t_ = -0.5 * eig.solve(b);

    vnl_double_4x4 Aprime = A_;
    vnl_double_4 At = A_*t_;
    vnl_double_4 Bprime = 2.0*At + b;
    double tAt = dot_product(t_, At);
    double Cprime = tAt + dot_product(t_, b) + c;

    // Now C is zero cos F is rank 2
    if (vnl_math_abs(Cprime) > 1e-6) {
      cerr << "FManifoldProject: ** HartleySturm: F = " << F_ << endl;
      cerr << "FManifoldProject: ** HartleySturm: B = " << Bprime << endl;
      cerr << "FManifoldProject: ** HartleySturm: Cerror = " << Cprime << endl;
      cerr << "FManifoldProject: ** HartleySturm: F not rank 2 ?\n";
      cerr << "FManifoldProject: singular values are "  << vnl_svd<double>(F_).W() << endl;
    }
    // **** Now have quadric x'*A*x = 0 ****
    
    // Rotate A
    
    // Group the sign-conjugates
    // Re-sort the eigensystem so that it is -a a -b b
    {
      int I[] = { 0, 3, 1, 2 };
      for(int col = 0; col < 4; ++col) {
	int from_col = I[col];
	d_[col] = eig.D(from_col,from_col);
	for(int r=0;r<4;++r)
	  V_(r,col) = eig.V(r, from_col);
      }
    }
  }
}

// -- Find the points out1, out2 which minimize d(out1,p1) + d(out2,p2) subject to
// out1'*F*out2 = 0.  Returns the minimum distance squared: ||x[1..4] - p[1..4]||^2.
double FManifoldProject::correct(const HomgPoint2D& p1, const HomgPoint2D& p2, HomgPoint2D* out1, HomgPoint2D* out2) const
{
  // Make the query point
  vnl_double_4 p;
  if (!p1.get_nonhomogeneous(p[0], p[1]) ||
      !p2.get_nonhomogeneous(p[2], p[3])) {
    cerr << "FManifoldProject: p1 or p2 at infinity\n";
    *out1 = p1;
    *out2 = p2;
    return 1e30;
  }

  if (_affine_F) {
    // Easy case for affine F, F is a plane.
    // t_ = n;
    // d_[0] = d;
    // pout = x - n (n p + d)
    const vnl_double_4& n = t_;
    double d = d_[0];

    double distance = (dot_product(n, p) + d);
    p -= n * distance;

    /// p -= n * (dot_product(n, p) + d);

    out1->set(p[0], p[1], 1.0);
    out2->set(p[2], p[3], 1.0);

    double EPIDIST = dot_product(out2->get_vector(), F_*out1->get_vector());
    if (EPIDIST > 1e-4) {
      cerr << "FManifoldProject: Affine F: EPIDIST = " << EPIDIST << endl;
      cerr << "FManifoldProject: Affine F: p = " << (dot_product(p,n) + d) << endl;

      double EPI1 = dot_product(out2->get_vector(), F_*out1->get_vector());
      double EPI2 = dot_product(p, n) + d;
      cerr << "t = " << n << " " << d << endl;
      cerr << "F_ = " << F_ << endl;
      cerr << "FManifoldProject: Affine F: E = " << (EPI1 - EPI2) << endl;
      //abort();
    }
    
    return distance * distance;
  }

  // Transform the query point
  vnl_double_4 pprime = vnl_transpose(V_) * (p - t_);

  // Solve p' (I - lambda D)^-1 D (I - lambda D)^-1 p = 0
  double b1 = 1./d_[0]; double a1 = vnl_math_sqr(pprime[0])*b1;
  double b2 = 1./d_[1]; double a2 = vnl_math_sqr(pprime[1])*b2;
  double b3 = 1./d_[2]; double a3 = vnl_math_sqr(pprime[2])*b3;
  double b4 = 1./d_[3]; double a4 = vnl_math_sqr(pprime[3])*b4;

  if (vnl_math_max(vnl_math_abs(b1 + b2), vnl_math_abs(b3 + b4)) > 1e-7) {
    cerr << "FManifoldProject: B = [" <<b1<< " " <<b2<< " " <<b3<< " " <<b4<< "];\n";
    cerr << "FManifoldProject: b1 != -b2 or b3 != -b4\n";
  }
  
  // a11 ../ (b1 - x).^2 + a12 ../ (b1 + x).^2 + a21 ../ (b2 - x).^2 + a22 ../ (b2 + x).^2
  // a11 = p1^2*b1
  //		   2         2              2         2              2         2          2              2         2         2
  //     (a3*(x - b1)  (x - b2)  + a2*(x - b1)  (x - b3)  + a1*(x - b2)  (x - b3) ) (x - b4)  + a4*(x - b1)  (x - b2)  (x - b3)
  // Coeffs from mma, assuming /. { b4 -> -b3, b2 -> -b1 }
  static vnl_vector<double> _coeffs(7);
  double b12 = b1*b1;
  double b32 = b3*b3;
  double b14 = b12*b12;
  double b34 = b32*b32;
  
  _coeffs[6] = a3*b14*b32 + a4*b14*b32 + a1*b12*b34 + a2*b12*b34;
  _coeffs[5] = (2*a3*b14*b3 - 2*a4*b14*b3 + 2*a1*b1*b34 - 2*a2*b1*b34);
  _coeffs[4] = (a3*b14 + a4*b14 - 2*(a1 +a2 + a3 + a4)*b12*b32 + a1*b34 + a2*b34);
  _coeffs[3] = (-4*a3*b12*b3 + 4*a4*b12*b3 - 4*a1*b1*b32 + 4*a2*b1*b32);
  _coeffs[2] = (a1*b12 + a2*b12 - 2*a3*b12 - 2*a4*b12 - 2*a1*b32 - 2*a2*b32 + a3*b32 + a4*b32);
  _coeffs[1] = 2*(b3*(a3 - a4) + b1*(a1 - a2));
  _coeffs[0] = (a1 + a2 + a3 + a4);

  // Don't try this: c = c ./ [1e0 1e2 1e4 1e6 1e8 1e10 1e12]
  _coeffs /= _coeffs.magnitude();

  vnl_real_polynomial poly(_coeffs);
  vnl_rpoly_roots roots(_coeffs);
  double dmin = 1e30;
  vnl_double_4 Xmin;
  vnl_vector<double> realroots = roots.realroots(1e-8);
  int errs = 0;
  bool got_one = false;
  for(unsigned i = 0; i < realroots.size(); ++i) {
    double lambda = realroots[i];
    
    // Some roots to the multiplied out poly are not roots to the rational polynomial.
    double RATPOLY_RESIDUAL = (a1/vnl_math_sqr(b1 - lambda) +
			       a2/vnl_math_sqr(b2 - lambda) +
			       a3/vnl_math_sqr(b3 - lambda) +
			       a4/vnl_math_sqr(b4 - lambda));

    if (fabs(RATPOLY_RESIDUAL) > 1e-8)
      continue;

    vnl_diag_matrix<double> Dinv(1.0 - lambda * d_);
    Dinv.invert_in_place();
    vnl_double_4 Xp = Dinv * pprime;
    vnl_double_4 X = V_ * Xp + t_;
    
    // Paranoia check
    {
      HomgPoint2D x1(X[0], X[1]);
      HomgPoint2D x2(X[2], X[3]);
      double EPIDIST = HomgOperator2D::perp_dist_squared(x2, HomgLine2D(F_*x1.get_vector()));
      if (0 && EPIDIST > 1e-12) {
	// This can happen in reasonable circumstances -- notably when one
	// epipole is at infinity.
	cout << "FManifoldProject: A root has epidist = " << sqrt(EPIDIST) << endl;
	cout << "  coeffs: " << _coeffs << endl;
	cout << "  root = " << lambda << endl;
	cout << "  poly residual = " << poly.evaluate(lambda) << endl;
	cout << "  rational poly residual = " << RATPOLY_RESIDUAL << endl;
	++ errs;
	break;
      }
    }
    
    double dist = (X - p).squared_magnitude();
    if (!got_one || dist < dmin) {
      dmin = dist;
      Xmin = X;
      got_one = true;
    }
  }

  if (!got_one) {
    cerr << "FManifoldProject: AROOGAH. Final epipolar distance =  " << dmin << ", errs = " << errs << endl;
    *out1 = p1;
    *out2 = p2;
  } else {
    out1->set(Xmin[0], Xmin[1], 1);
    out2->set(Xmin[2], Xmin[3], 1);
  }

  return dmin;
}
