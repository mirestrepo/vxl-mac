//:
// \file

#include "vdgl_digital_region.h"
#include <vcl_cmath.h> // for fabs and sqrt
#include <vcl_cassert.h>
#include <vnl/algo/vnl_svd.h>

#if 0
#include <Basics/Histogram.h>
#include <SpatialBasics/tolerance.h>
#include <DigitalGeometry/IntensityCoef.h>
#include <DigitalGeometry/DigitalRegion.h>
#include <vnl/vnl_numeric_traits.h>
#endif

#ifndef MAX_ROUNDOFF
#define MAX_ROUNDOFF .000025
#endif

#define near_zero(a) (vcl_fabs (a) < MAX_ROUNDOFF)

vsol_spatial_object_2d_sptr vdgl_digital_region::clone() const
{
  return new vdgl_digital_region(*this);
}

//-----------------------------------------------------------
//: Initialize region data
void vdgl_digital_region::init()
{
  npts_ = 0;
  pixel_size_=1.0;
  max_ = 0;
  min_ = (unsigned short)(-1);
  xp_ = NULL; yp_ = NULL; zp_ = NULL; pix_ = NULL;
  xo_ = 0; yo_ = 0; zo_ = 0; io_ = 0;
  X2_ = 0;  Y2_ = 0;  I2_ = 0;
  XY_ = 0;  XI_ = 0;  YI_ = 0;
  error_ = 0; sigma_sq_ = 0;
  fit_valid_ = false;
  scatter_matrix_valid_ = false;
  Si_ = new vnl_matrix<double>(3, 3, 0.0);
  pi_ = new vnl_matrix<double>(3, 1, 0.0);
#if 0
  hist_ = NULL;
#endif
}
//-------------------------------------------------------------------------
// Constructors
//
vdgl_digital_region::vdgl_digital_region()
{
  this->init();
}
vdgl_digital_region::vdgl_digital_region(vdgl_digital_region const& r)
{
  this->init();
  for (int i = 0; i<r.Npix(); ++i)
    this->IncrementMeans(r.Xj()[i], r.Yj()[i], r.Zj()[i], r.Ij()[i]);
  this->InitPixelArrays();
  for (int i = 0; i<r.Npix(); i++)
    this->InsertInPixelArrays(r.Xj()[i], r.Yj()[i], r.Zj()[i], r.Ij()[i]);
}
vdgl_digital_region::vdgl_digital_region(int npts, const float* xp, const float* yp,
                                         const unsigned short *pix)
{
  assert(npts > 0);
  this->init();
  for (int i = 0; i<npts; i++)
    this->IncrementMeans(xp[i], yp[i], pix[i]);
  this->InitPixelArrays();
  for (int i = 0; i<npts_; i++)
    this->InsertInPixelArrays(xp[i], yp[i], pix[i]);
}
vdgl_digital_region::vdgl_digital_region(int npts, const float* xp, const float* yp,
                                         const float* zp, const unsigned short *pix)
{
  assert(npts > 0);
  this->init();
  for (int i = 0; i<npts; i++)
    this->IncrementMeans(xp[i], yp[i], zp[i], pix[i]);
  this->InitPixelArrays();
  for (int i = 0; i<npts_; i++)
    this->InsertInPixelArrays(xp[i], yp[i], zp[i], pix[i]);
}
//-------------------------------------------------------------------------
//
// Destructor.
//
vdgl_digital_region::~vdgl_digital_region()
{
  delete [] xp_;
  delete [] yp_;
  delete [] zp_;
  delete [] pix_;
  delete pi_;
  delete Si_;
}

#if 0
//--------------------------------------------------------
//: The Histogram is lazily evaluated.
//  If a Histogram is present, it is not recomputed unless force is true;
Histogram_ref vdgl_digital_region::GetHistogram(bool force)
{
  if (!npts_) return NULL;
  int res = int(max_-min_);
  if (hist_&&!force)
    return hist_;
  Histogram* h;
  if (res)
    h = new Histogram(res, min_, max_);
  else
    h = new Histogram(1, min_, max_);
  for (int i =0; i<npts_; i++)
    h->UpCount(pix_[i]);
  hist_ = h;
  return h;
}
#endif

//------------------------------------------------------------
//: reset the iterator for accessing region pixels
//
void vdgl_digital_region::reset() const
{
  pix_index_ = -1; // mutable const
}

//------------------------------------------------------------
//: check if iterator is finished
//  - colin found a major bug here
//    we were skipping pixel[0] for the entire existence of this class
//
bool vdgl_digital_region::next() const
{
  ++pix_index_; // mutable const
  return pix_index_<npts_;
}

//-------------------------------------------------------
//: The X pixel coordinate
float vdgl_digital_region::X() const
{
  if (pix_index_<0)
    return 0.0f;
  return xp_[pix_index_];
}

//-------------------------------------------------------
//: The Y pixel coordinate
float vdgl_digital_region::Y() const
{
  if (pix_index_<0)
    return 0.0f;
  return yp_[pix_index_];
}

//-------------------------------------------------------
//: The Z pixel coordinate
float vdgl_digital_region::Z() const
{
  if (pix_index_<0)
    return 0.0f;
  return zp_[pix_index_];
}

//-------------------------------------------------------
//: The pixel Intensity
unsigned short vdgl_digital_region::I() const
{
  if (pix_index_<0)
    return 0;
  return pix_[pix_index_];
}
//--------------------------------------------------------
//:
//  Initialize the region for accepting a stream of pixels
//  through repeated calls to ::IncrementMeans(..)
void vdgl_digital_region::ResetPixelData()
{
  npts_ = 0;
  delete [] xp_; xp_ = NULL;
  delete [] yp_; yp_ = NULL;
  delete [] zp_; zp_ = NULL;
  delete [] pix_; pix_ = NULL;
  xo_ = 0; yo_ = 0; zo_ = 0; io_ = 0;
}
//---------------------------------------------------------
//: The 2-d version
void vdgl_digital_region::IncrementMeans(float x, float y, unsigned short pix)
{
  this->IncrementMeans(x, y, 0, pix);
}
//-----------------------------------------------------------------
//:
//    Used to scan through a set of pixels and acquire mean values
//    for coordinates and intensity.  This approach is neccessary to
//    accumulate scatter matrices which have zero mean. One scans
//    the region pixels twice. First to get the means and and number of
//    region pixels and then to insert the pixel data into fixed arrays
void vdgl_digital_region::IncrementMeans(float x, float y, float z,
                                         unsigned short pix)
{
  ++npts_;
  xo_ += x;
  yo_ += y;
  zo_ += z;
  io_ += pix;
}
//-----------------------------------------------------------------
//:
// Now we have the number of pixels so we can create the storage arrays.
void vdgl_digital_region::InitPixelArrays()
{
  assert(npts_ > 0);
  delete [] xp_;  xp_ = new float[npts_];
  delete [] yp_;  yp_ = new float[npts_];
  delete [] zp_;  zp_ = new float[npts_];
  delete [] pix_; pix_ = new unsigned short[npts_];
  pix_index_ = 0;
  min_ = (unsigned short)(-1);
  max_ = 0;
}
//--------------------------------------------------------------------
//: the 2-d version
void vdgl_digital_region::InsertInPixelArrays(float x, float y,
                                              unsigned short pix)
{
  this->InsertInPixelArrays(x, y, 0, pix);
}

//------------------------------------------------------------------------
//: Insert pixel data into the face arrays.
void vdgl_digital_region::InsertInPixelArrays(float x, float y, float z,
                                              unsigned short pix)
{
  assert(npts_ > 0);
  if (pix_index_<0||pix_index_>npts_) return;
  xp_[pix_index_] = x; yp_[pix_index_] = y;
  zp_[pix_index_] = z; pix_[pix_index_] = pix;
  if (pix<min_) min_ = pix;
  if (pix>max_) max_ = pix;
  ++pix_index_;
}

float vdgl_digital_region::Xo() const
{
  assert(npts_ > 0);
  return xo_/npts_;
}

float vdgl_digital_region::Yo() const
{
  assert(npts_ > 0);
  return yo_/npts_;
}

float vdgl_digital_region::Zo() const
{
  assert(npts_ > 0);
  return zo_/npts_;
}

float vdgl_digital_region::Io() const
{
  assert(npts_ > 0);
  return io_/npts_;
}
//: Individual scatter matrix elements
double vdgl_digital_region::X2() const
{
  if (!scatter_matrix_valid_)
    this->ComputeScatterMatrix();
  return X2_;
}
double vdgl_digital_region::Y2() const
{
  if (!scatter_matrix_valid_)
    this->ComputeScatterMatrix();
  return Y2_;
}
double vdgl_digital_region::XY() const
{
  if (!scatter_matrix_valid_)
    this->ComputeScatterMatrix();
  return XY_;
}
double vdgl_digital_region::I2() const
{
  if (!scatter_matrix_valid_)
    this->ComputeScatterMatrix();
  return I2_;
}
double vdgl_digital_region::XI() const
{
  if (!scatter_matrix_valid_)
    this->ComputeScatterMatrix();
  return XI_;
}
double vdgl_digital_region::YI() const
{
  if (!scatter_matrix_valid_)
    this->ComputeScatterMatrix();
  return YI_;
}
//------------------------------------------------------------
//: Compute the diameter from the scatter matrix
float vdgl_digital_region::Diameter() const
{
  // make sure the scatter matrix is valid
  if (!scatter_matrix_valid_)
    this->ComputeScatterMatrix();
  if (this->Npix()<4)
    return 1.0;
  // construct the lower right 2x2 matrix of S, s.
  vnl_matrix<double> s(2, 2, 0.0);
  for (int r = 1; r<=2; r++)
    for (int c = 1; c<=2; c++)
      s(r-1,c-1) = (*Si_)(r,c);
  //Compute SVD of s to get diameter
  vnl_svd<double> svd(s);
  if (svd.rank()!=2)
    return float(vcl_sqrt(this->area()));
  //The factor of two is to estimate the extreme limit of the distribution
  double r = 2*vcl_sqrt(vcl_fabs(svd.W(0))+ vcl_fabs(svd.W(1)));
  return float(2*r);//diameter
}

//------------------------------------------------------------
//: Compute the aspect ratio from the scatter matrix
float vdgl_digital_region::AspectRatio() const
{
  // make sure the scatter matrix is valid
  if (!scatter_matrix_valid_)
    this->ComputeScatterMatrix();
  if (this->Npix()<4)
    return 1.0;
  // construct the lower right 2x2 matrix of S, s.
  vnl_matrix<double> s(2, 2, 0.0);
  for (int r = 1; r<=2; r++)
    for (int c = 1; c<=2; c++)
      s(r-1,c-1) = (*Si_)(r,c);
  //Compute SVD of s to get aspect ratio
  vnl_svd<double> svd(s);
  if (svd.rank()!=2)
    return 1.0;
  return (float)vcl_sqrt(svd.W(0)/svd.W(1));
}

#if 0
//------------------------------------------------------------
//: Compute the principal orientation of the region.
//   major_axis is a 2-d vector representing the orientation.
void vdgl_digital_region::PrincipalOrientation(vcl_vector<float>& major_axis)
{
  // make sure the scatter matrix is valid
  if (!scatter_matrix_valid_)
    this->ComputeScatterMatrix();
  if (this->Npix()<4)
    {
      vcl_cout << "In vdgl_digital_region::PrincipalOrientation(..) Npts<4\n";
      major_axis.x()=1.0; major_axis.y()=0.0;
      return;
    }
  // construct the lower right 2x2 matrix of S, s.
  vnl_matrix<double> s(2, 2, 0.0);
  for (int r = 1; r<=2; r++)
    for (int c = 1; c<=2; c++)
      s(r-1,c-1) = (*Si_)(r,c);
  //Compute SVD of s to get aspect ratio
  SVD svd(s);
  if (svd.rank()!=2)
    {
      vcl_cout << "In vdgl_digital_region::PrincipalOrientation(..) Insufficient rank\n";
      major_axis.x()=1.0; major_axis.y()=0.0;
      return;
    }
  vnl_matrix<double> v = svd.V();
  //2 sigma gives a good estimate of axis length (sigma = principal eigenvalue)
  double r = 2*vcl_sqrt(vcl_fabs(svd.W(0)));
  major_axis.x()=float(v(0,0)*r);
  major_axis.y()=float(v(1,0)*r);
}

#endif

double vdgl_digital_region::Ix() const
{
  if (!fit_valid_)
    this->DoPlaneFit();
  return Ix_;
}
double vdgl_digital_region::Iy() const
{
  if (!fit_valid_)
    this->DoPlaneFit();
  return Iy_;
}


//--------------------------------------------------------------
//: Update the scatter matrix elements.
// The means are subtracted from the input values before incrementing.
// Thus the scatter matrix is formed with a coordinate system at the origin.
void vdgl_digital_region::IncrByXYI(double x, double y, int intens) const
{
  fit_valid_=false;
  scatter_matrix_valid_=false;
  double dbx = x - this->Xo(), dby = y - this->Yo();
  double dint = (double)intens- this->Io();
  X2_ += dbx*dbx;  Y2_ += dby*dby;  I2_ += dint*dint;
  XY_ += dbx*dby;  XI_ += dbx*dint;  YI_ += dby*dint;
}

//-------------------------------------------
//: The scatter matrix (upper 3x3) is transformed to the origin by subtracting off the means.
void vdgl_digital_region::ComputeScatterMatrix() const
{
  if (!npts_)
    {
      vcl_cout << "In vdgl_digital_region::ComputeScatterMatrix() - no pixels\n";
      return;
    }
  X2_ = 0;  Y2_ = 0;  I2_ = 0;
  XY_ = 0;  XI_ = 0;  YI_ = 0;

  for (this->reset(); this->next();)
    this->IncrByXYI(this->X(), this->Y(), this->I());


  //The Scatter Matrix
  //
  //   -             -
  //  | I^2  XI   YI  |
  //  |               |
  //  | XI   X^2  XY  |
  //  |               |
  //  | YI   XY   Y^2 |
  //   -             -
  //
  Si_->put(0,0, I2_/npts_);
  Si_->put(0,1, XI_/npts_);
  Si_->put(0,2, YI_/npts_);
  Si_->put(1,1, X2_/npts_);
  Si_->put(1,2, XY_/npts_);
  Si_->put(2,2, Y2_/npts_);
  Si_->put(1,0, Si_->get(0,1));
  Si_->put(2,0, Si_->get(0,2));
  Si_->put(2,1, Si_->get(1,2));
  scatter_matrix_valid_ = true;
}

//--------------------------------------------------------------
//: Solve for the fitted plane.
//  Closed form solution in terms of the scatter matrix.
void vdgl_digital_region::SolveForPlane() const
{
  assert(npts_ >= 4);
  fit_valid_ = true;
  double den = Si_->get(1,1)*Si_->get(2,2)
             - Si_->get(1,2)*Si_->get(1,2);
  if (near_zero(den))
    {
      vcl_cout << "In vdgl_digital_region::SolveForPlane(..) determinant near zero\n";
      Ix_ = 0;
      Iy_ = 0;
      pi_->put(0,0, 1.0);   pi_->put(1,0, -Ix_);   pi_->put(2,0, -Iy_);
      return;
    }
  double adet = Si_->get(0,1)*Si_->get(2,2)
              - Si_->get(0,2)*Si_->get(1,2);
  double bdet = Si_->get(1,1)*Si_->get(0,2)
              - Si_->get(0,1)*Si_->get(1,2);
  Ix_ = adet/den;
  Iy_ = bdet/den;
  pi_->put(0,0, 1.0);   pi_->put(1,0, -Ix_);   pi_->put(2,0, -Iy_);
}

//---------------------------------------------------------------
//: Computes the square residual error using the scatter matrix
//
double vdgl_digital_region::ComputeResidual(vnl_matrix<double> const& pvect) const
{
  if (!fit_valid_)
    this->DoPlaneFit();
  vnl_matrix<double> residual = pvect.transpose() * (*Si_) * pvect;
  return residual.get(0,0);
}

//---------------------------------------------------------------
//: Computes the squared deviation from the sample mean
//
double vdgl_digital_region::ComputeSampleResidual() const
{
  if (!fit_valid_)
    this->DoPlaneFit();
  return Si_->get(0,0) + Si_->get(1,1) + Si_->get(2,2);
}

void vdgl_digital_region::DoPlaneFit() const
{
  if (!scatter_matrix_valid_)
    this->ComputeScatterMatrix();

  this->SolveForPlane();
  error_ =  this->ComputeResidual(*pi_);
  sigma_sq_ =  this->ComputeSampleResidual();
}


void vdgl_digital_region::PrintFit() const
{
  if (!fit_valid_)
    this->DoPlaneFit();
  vcl_cout << "IntensityFit(In Plane Coordinates): "
       << "Number of Points =" <<  npts_ << vcl_endl
       << "Scatter Matrix: \n"
       << "X2 Y2 I2 " << this->X2() << " " << this->Y2()
       << " " << this->I2() << vcl_endl
       << "XY XI YI " << this->XY() << " " << this->XI()
       << " " << this->YI() << vcl_endl
       << "Xo Yo Io " << this->Xo() << " " << this->Yo()
       << " " << this->Io() << vcl_endl
       << "fitted Plane: \n"
       << "di/dx " << this->Ix() << vcl_endl
       << "di/dy " << this->Iy() << vcl_endl
       << "sample variance: " << this->Var()<< vcl_endl
       << "squared cost: " << error_ << vcl_endl
       << "average cost: " << vcl_sqrt(error_) << vcl_endl << vcl_endl;
}

#if 0
//-----------------------------------------------------------
//: Return a package of fitted coeficients
//
IntensityCoef_ref vdgl_digital_region::GetIntCoef() const
{
  if (!fit_valid_)
    this->DoPlaneFit();
  return new IntensityCoef(this->Npix(), float(this->Var()),
                           float(this->Io()), float(this->Ix()),
                           float(this->Iy()));
}
#endif

//-------------------------------------------------------
//: The Residual Intensity
float vdgl_digital_region::Ir() const
{
  if (pix_index_<0)
    return 0.0;

  if (!fit_valid_)
    {
      int initial_pix_index = pix_index_;//Save the current pix_index_ state
      this->DoPlaneFit();
      pix_index_ = initial_pix_index;//Restore the pix_index_ state
    }
  float val = float(this->I());
  float io = this->Io(),
        ix = float(this->Ix()), x = this->X(), xo = this->Xo(),
        iy = float(this->Iy()), y = this->Y(), yo = this->Yo();
  float plane_int = io + ix*(x - xo) + iy*(y-yo);
  return val - plane_int;
}

#if 0
Histogram_ref vdgl_digital_region::GetResidualHistogram()
{
  if (!npts_) return NULL;
  float min = vnl_numeric_traits<float>::maxval,
        max = -vnl_numeric_traits<float>::maxval;
  for (this->reset(); this->next();)
    {
      float ir = (float)this->Ir();
      if (ir<min)
        min = ir;
      if (ir>max)
        max = ir;
    }
  int res = int(max-min);
  Histogram* h = new Histogram(res, min, max);
  for (this->reset(); this->next();)
    h->UpCount(this->Ir());
  return h;
}

bool vdgl_digital_region::Transform(CoolTransform const& t)
{
  //Transform the Pixels
  int n = npts_;
  if (!npts_)
    return false;
  for (int i=0; i<n; i++)
    {
      vcl_vector<float> p(xp_[i], yp_[i], zp_[i]);
      vcl_vector<float>pt = p*t;
      xp_[i]=pt.x();
      yp_[i]=pt.y();
      zp_[i]=pt.z();
    }
  //Transform the mean pixel position
  vcl_vector<float> m(this->Xo(), this->Yo(), this->Zo());
  vcl_vector<float>mt = m*t;
  xo_ = mt.x()*n;
  yo_ = mt.y()*n;
  zo_ = mt.z()*n;
  fit_valid_ = false;
  scatter_matrix_valid_=false;
  return true;
}
#endif
