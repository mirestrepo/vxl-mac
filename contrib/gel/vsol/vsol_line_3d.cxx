#include <vsol/vsol_line_3d.h>

//*****************************************************************************
// External declarations for implementation
//*****************************************************************************
#include <vcl_cassert.h>
#include <vnl/vnl_math.h>
#include <vsol/vsol_point_3d.h>
#include <vgl/vgl_homg_point_3d.h>

//***************************************************************************
// Initialization
//***************************************************************************

//---------------------------------------------------------------------------
//: Constructor from the direction and the middle point
//---------------------------------------------------------------------------
vsol_line_3d::vsol_line_3d(const vnl_vector_fixed<double,3> &new_direction,
                           const vsol_point_3d_sptr &new_middle)
{
  p0_=new vsol_point_3d(*(new_middle->plus_vector(-(new_direction)/2)));
  p1_=new vsol_point_3d(*(new_middle->plus_vector((new_direction)/2)));
}

//---------------------------------------------------------------------------
//: Constructor
//---------------------------------------------------------------------------
vsol_line_3d::vsol_line_3d(const vsol_point_3d_sptr &new_p0,
                           const vsol_point_3d_sptr &new_p1)
{
  p0_=new_p0;
  p1_=new_p1;
}

//---------------------------------------------------------------------------
//: Copy constructor
// Description: no duplication of the points
//---------------------------------------------------------------------------
vsol_line_3d::vsol_line_3d(const vsol_line_3d &other)
{
  p0_=other.p0_;
  p1_=other.p1_;
}

//---------------------------------------------------------------------------
// Destructor
//---------------------------------------------------------------------------
vsol_line_3d::~vsol_line_3d()
{
}

//---------------------------------------------------------------------------
//: Clone `this': creation of a new object and initialization
// See Prototype pattern
//---------------------------------------------------------------------------
vsol_spatial_object_3d_sptr vsol_line_3d::clone(void) const
{
  return new vsol_line_3d(*this);
}

//***************************************************************************
// Access
//***************************************************************************

//---------------------------------------------------------------------------
//: Middle point of the straight line segment
//---------------------------------------------------------------------------
vsol_point_3d_sptr vsol_line_3d::middle(void) const
{
  return p0_->middle(*p1_);
}

//---------------------------------------------------------------------------
//: direction of the straight line segment. Has to be deleted manually
//---------------------------------------------------------------------------
vnl_vector_fixed<double,3> *vsol_line_3d::direction(void) const
{
  vnl_vector_fixed<double,3> *result;

  result=p0_->to_vector(*p1_);

  return result;
}

//---------------------------------------------------------------------------
//: First point of the straight line segment
//---------------------------------------------------------------------------
vsol_point_3d_sptr vsol_line_3d::p0(void) const
{
  return p0_;
}

//---------------------------------------------------------------------------
//: Last point of the straight line segment
//---------------------------------------------------------------------------
vsol_point_3d_sptr vsol_line_3d::p1(void) const
{
  return p1_;
}

//***************************************************************************
// Comparison
//***************************************************************************

//---------------------------------------------------------------------------
//: Has `this' the same points than `other' ?
//---------------------------------------------------------------------------
bool vsol_line_3d::operator==(const vsol_line_3d &other) const
{
  bool result;

  result=this==&other;
  if(!result)
    result=(((*p0_)==(*(other.p0_)))&&((*p1_)==(*(other.p1_))))
      ||(((*p0_)==(*(other.p1_)))&&((*p1_)==(*(other.p0_))));
  return result;
}

//: spatial object equality

bool vsol_line_3d::operator==(const vsol_spatial_object_3d& obj) const
{
  return
   obj.spatial_type() == vsol_spatial_object_3d::CURVE &&
   ((vsol_curve_3d const&)obj).curve_type() == vsol_curve_3d::LINE
  ? *this == (vsol_line_3d const&) (vsol_curve_3d const&) obj
  : false;
}

//---------------------------------------------------------------------------
//: Has `this' not the same points than `other' ?
//---------------------------------------------------------------------------
bool vsol_line_3d::operator!=(const vsol_line_3d &other) const
{
  return !operator==(other);
}

//***************************************************************************
// Status report
//***************************************************************************

//---------------------------------------------------------------------------
//: Return the real type of a line. It is a CURVE
//---------------------------------------------------------------------------
vsol_spatial_object_3d::vsol_spatial_object_3d_type
vsol_line_3d::spatial_type(void) const
{
  return CURVE;
}

//---------------------------------------------------------------------------
//: Compute the bounding box of `this'
//---------------------------------------------------------------------------
void vsol_line_3d::compute_bounding_box(void)
{
  double xmin;
  double xmax;
  double ymin;
  double ymax;
  double zmin;
  double zmax;

  xmin=p0_->x();
  ymin=p0_->y();
  zmin=p0_->z();
  xmax=xmin;
  ymax=ymin;
  zmax=zmin;

  if(p1_->x()<xmin)
    xmin=p1_->x();
  else if(p1_->x()>xmax)
    xmax=p1_->x();
  if(p1_->y()<ymin)
    ymin=p1_->y();
  else if(p1_->y()>ymax)
    ymax=p1_->y();
  if(p1_->z()<zmin)
    zmin=p1_->z();
  else if(p1_->z()>zmax)
    zmax=p1_->z();

  if(_bounding_box==0)
    _bounding_box=new vsol_box_3d();
  _bounding_box->set_min_x(xmin);
  _bounding_box->set_max_x(xmax);
  _bounding_box->set_min_y(ymin);
  _bounding_box->set_max_y(ymax);
  _bounding_box->set_min_y(zmin);
  _bounding_box->set_max_y(zmax);
}

//---------------------------------------------------------------------------
//: Return the length of `this'
//---------------------------------------------------------------------------
double vsol_line_3d::length(void) const
{
  return p0_->distance(p1_);
}

//***************************************************************************
// Status setting
//***************************************************************************

//---------------------------------------------------------------------------
//: Set the first point of the straight line segment
//---------------------------------------------------------------------------
void vsol_line_3d::set_p0(const vsol_point_3d_sptr &new_p0)
{
  p0_=new_p0;
}

//---------------------------------------------------------------------------
//: Set the last point of the straight line segment
//---------------------------------------------------------------------------
void vsol_line_3d::set_p1(const vsol_point_3d_sptr &new_p1)
{
  p1_=new_p1;
}

//---------------------------------------------------------------------------
//: Set the length of `this'. Doesn't change middle point and orientation.
//    If p0 and p1 are equal then the direction is set to (1,0,0)
// Require: new_length>=0
//---------------------------------------------------------------------------
void vsol_line_3d::set_length(const double new_length)
{
  // require
  assert(new_length>=0);

  vsol_point_3d_sptr m=middle();
  vnl_vector_fixed<double,3> *d=direction();

  if((*p0_)==(*p1_)) // ie. d=0 then d is set to (1,0,0)
    {
      (*d)[0]=1;
      (*d)[1]=0;
      (*d)[2]=0;
    }
  else
    d->normalize();

  (*d)*=new_length;

  p0_=new vsol_point_3d(*(m->plus_vector(-(*d)/2)));
  p1_=new vsol_point_3d(*(m->plus_vector((*d)/2)));

  delete d;
}

//***************************************************************************
// Basic operations
//***************************************************************************

//---------------------------------------------------------------------------
//: Is `p' in `this' ?
//---------------------------------------------------------------------------
bool vsol_line_3d::in(const vsol_point_3d_sptr &p) const
{
  bool result;
  double dot_product;
  double ax;
  double ay;
  double az;
  double bx;
  double by;
  double bz;
  // `p' belongs to the straight line
  ax=p1_->x()-p0_->x();
  ay=p1_->y()-p0_->y();
  az=p1_->z()-p0_->z();
  bx=p->x()-p0_->x();
  by=p->y()-p0_->y();
  bz=p->z()-p0_->z();

  result=(ay*bz-az*by==0)&&(az*bx-ax*bz==0)&&(ax*by-ay*bx==0);
  if(result) // `p' belongs to the segment
    {
      dot_product=(p->x()-p0_->x())*(p1_->x()-p0_->x())
        +(p->y()-p0_->y())*(p1_->y()-p0_->y())
        +(p->z()-p0_->z())*(p1_->z()-p0_->z());
      result=(dot_product>=0)&&
        (dot_product<(vnl_math_sqr(p1_->x()-p0_->x())
                      +vnl_math_sqr(p1_->y()-p0_->y())
                      +vnl_math_sqr(p1_->z()-p0_->z())));
    }
  return result;
}

//---------------------------------------------------------------------------
//: Return the tangent to `this' at `p'.  Has to be deleted manually
// Require: in(p)
//---------------------------------------------------------------------------
vgl_homg_line_3d_2_points<double> *
vsol_line_3d::tangent_at_point(const vsol_point_3d_sptr &p) const
{
  assert(false); // TO DO
  // require
  // assert(in(p));

  vgl_homg_line_3d_2_points<double> *result;
  vgl_homg_point_3d<double> a(p0_->x(),p0_->y(),p0_->z());
  vgl_homg_point_3d<double> b(p1_->x(),p1_->y(),p1_->z());

  result=new vgl_homg_line_3d_2_points<double>(a,b);

  return result;
}
