#include "HMatrix1DComputeOptimize1.h"
#include "HMatrix1DComputeDesign.h"

#include <vcl/vcl_cassert.h>

#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <mvl/HMatrix1D.h>

//********************************************************************************
//
//
//
//********************************************************************************



class XXX : public vnl_least_squares_function 
{ 
private:
  unsigned N;
  const vcl_vector<double> &z1,z2;
public: 
  XXX(const vcl_vector<double> &z1_,const vcl_vector<double> &z2_)
    : vnl_least_squares_function(3, z1_.size(), no_gradient) 
    , N(z1_.size())
    , z1(z1_) , z2(z2_)
    {
      assert(N == z1.size());
      assert(N == z2.size());
      //      cout << "N=" << N << endl;
    }
  ~XXX() { N=0; }

  void boo(const vnl_vector<double> &x) {
    assert(x.size()==3);
    int f=cout.flags();
    cout.flags(ios::fixed | ios::showpos);
    double z,y;
    for (unsigned i=0;i<N;i++) {
      z=z1[i];
      y=(z+x[0])/(x[1]*z+1+x[2]);
      cout << z << ' ' << y << '[' << z2[i] << ']' << endl;
    }
    cout.flags(f);
  }
  
  //	the matrix is	[ 1.0    x[0] ]
  //			[ x[1] 1+x[2] ]
  void f(const vnl_vector<double>& x, vnl_vector<double>& fx) {
    assert(x.size()==3);
    assert(fx.size()==N);
    double z,y;
    for (unsigned k=0;k<N;k++) {
      z=z1[k];
      y=(z+x[0])/(x[1]*z+1+x[2]);
      fx[k]=z2[k]-y;
    }
  } 
}; 

static
void foo(const vcl_vector<double> &z1,const vcl_vector<double> &z2,HMatrix1D &M) 
{
  //
  // **** minimise over the set of 2x2 matrices of the form [  1     x[0] ] ****
  // ****                                                   [ x[1] 1+x[2] ] ****
  //
  
  // Set up a compute object :
  XXX f(z1,z2);
  
  // Set up the initial guess :
  vnl_vector<double> x(3);
  x.fill(0);
  
  // Make a Levenberg Marquardt minimizer and attach f to it :
  vnl_levenberg_marquardt LM(f);
  
  // Run minimiser :
  //	f.boo(x);
  LM.minimize(x);
  //	f.boo(x);
  
  // convert back to matrix format.
  vnl_double_2x2 T;
  T(0,0)=1; T(0,1)=x[0];
  T(1,0)=x[1]; T(1,1)=1+x[2];
  M.set(T);
}

HMatrix1DComputeOptimize1::HMatrix1DComputeOptimize1(void) : HMatrix1DCompute() { }
HMatrix1DComputeOptimize1::~HMatrix1DComputeOptimize1() { }

bool
HMatrix1DComputeOptimize1::compute_cool_homg(const vcl_vector<HomgPoint1D>&p1,
					     const vcl_vector<HomgPoint1D>&p2,
					     HMatrix1D *M)
{
  unsigned N=p1.size();
  assert(N==p2.size());
  if (N<3) return false;

  vcl_vector<double> z1(N,0.0),z2(N,0.0);
  HMatrix1DComputeDesign C;
  C.compute(p1,p2,M);

  // map the points in p1 under M so that we are looking for a correction near the identity :
  for (unsigned i=0;i<N;i++) {
    if (!M->transform_to_plane2(p1[i]).get_nonhomogeneous(z1[i])) return false;
    if (!p2[i].get_nonhomogeneous(z2[i])) return false;
  }

//	cout << "z1  = " << z1 << endl;
//	cout << "z2  = " << z2 << endl;
//	cout << "M   = " << M << endl;
//	cout << "Mz1 = " << Mz1 << endl;

  HMatrix1D K;
  foo(z1,z2,K);
  *M=HMatrix1D(K,*M);      // refine M using the correction K.
  return true;
}

