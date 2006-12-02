#include "rgrl_mask.h"
//:
// \file
#include <vcl_cassert.h>
#include <vcl_limits.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_transpose.h>

rgrl_mask_box
rgrl_mask::
bounding_box() const
{
  rgrl_mask_box box( x0_, x1_ );
  return box;
}

//************* mask using a binary 2D image ********************
rgrl_mask_2d_image::
rgrl_mask_2d_image( const vil_image_view<vxl_byte>& in_mask,
                    int org_x, int org_y)
  : rgrl_mask( 2 ), mask_image_( in_mask ), 
    org_x_( org_x ), org_y_( org_y )
{
  update_bounding_box();
}

bool
rgrl_mask_2d_image::
inside( vnl_vector<double> const& pt ) const
{
  double x = pt[0]-double(org_x_);
  double y = pt[1]-double(org_y_);

  // As the bounding box is tighter than image dim,
  // check w/ bounding box is sufficient
  //
  bool in_range = ( x0_[0] <= x ) && ( x <= x1_[0] ) && 
                  ( x0_[1] <= y ) && ( y <= x1_[1] );
  
  return in_range && mask_image_( vnl_math_rnd(x), vnl_math_rnd(y) )>0;
}

void
rgrl_mask_2d_image::
update_bounding_box()
{
  // reset bounding box
  x0_[0] = double( mask_image_.ni() );
  x0_[1] = double( mask_image_.nj() );
  x1_[0] = 0.0;
  x1_[1] = 0.0;

  bool non_zero_pixel = false;
  
  for( unsigned j=0; j<mask_image_.nj(); ++j )
    for( unsigned i=0; i<mask_image_.ni(); ++i )
      if( mask_image_(i,j) ) {
        
        if( x0_[0] > double(i) )        x0_[0] = double(i);
        if( x0_[1] > double(j) )        x0_[1] = double(j);
        if( x1_[0] < double(i) )        x1_[0] = double(i);
        if( x1_[1] < double(j) )        x1_[1] = double(j);
  
        non_zero_pixel = true;      
      }
  
  // special case: no pixel is true
  if( !non_zero_pixel ) {
    x0_.fill( 0.0 );
    x1_.fill( 0.0 );
  }
}

//******************* mask using a sphere *****************

rgrl_mask_sphere::
rgrl_mask_sphere( unsigned dim )
  : rgrl_mask( dim ), center_( dim, 0.0 ), radius_sqr_( 0 )
{
}

rgrl_mask_sphere::
rgrl_mask_sphere( const vnl_vector<double>& in_center,
                  double in_radius )
  : rgrl_mask( in_center.size() ),
    center_(in_center), 
    radius_sqr_(in_radius*in_radius)
{
  update_bounding_box();
}

bool
rgrl_mask_sphere::
inside( vnl_vector<double> const& pt ) const
{
  assert( pt.size() == center_.size() );

  double sqr_sum = 0;
  for (unsigned int i = 0; i<pt.size(); ++i) {
    sqr_sum +=  vnl_math_sqr( pt[i] - center_[i] );
  }
  return sqr_sum < radius_sqr_ ;
}

void
rgrl_mask_sphere::
set_center( vnl_vector<double> const& pt )
{
  assert( pt.size() == center_.size() );
  center_ = pt;
  
  update_bounding_box();
}

void
rgrl_mask_sphere::
set_radius( double radius )
{
  radius_sqr_ = radius * radius;
  
  update_bounding_box();
}

void
rgrl_mask_sphere::
update_bounding_box()
{
  // if ceter or radius not yet set
  if( !center_.size() || !radius_sqr_ )
    return;
 
  const unsigned m = center_.size();
  x0_.set_size( m );
  x1_.set_size( m );
  double r = vcl_sqrt( radius_sqr_ );
  for( unsigned i=0; i<m; ++i ) {
    x0_[i] = center_[i] - r;
    x1_[i] = center_[i] + r;
  }
}

//******************** mask using an oriented box ***********************

rgrl_mask_oriented_box::
rgrl_mask_oriented_box( unsigned dim )
  : rgrl_mask( dim ),
    origin_(dim),
    axes_(dim, dim),
    len_(dim)
{
}

rgrl_mask_oriented_box::
rgrl_mask_oriented_box( vnl_vector<double> const& x0, 
                        vnl_matrix<double> const& axes,
                        vnl_vector<double> const& len )
  : rgrl_mask( x0.size() ),
    origin_( x0 ),
    axes_( axes ),
    len_( len )
{
  assert( x0.size() == len.size() );
  assert( x0.size() == axes.rows());
  assert( x0.size() == axes.cols());
  
  update_bounding_box();
}

bool
rgrl_mask_oriented_box::
inside( vnl_vector<double> const& pt ) const
{
  assert( pt.size() == origin_.size() );
 
  vnl_vector<double> mapped = axes_ * ( pt - origin_ );
  
  // len_[i] >=0 is gurranteed in update_bounding_box function
  // 
  bool inside = true;
  for( unsigned i=0; i<origin_.size()&&inside; ++i )
    inside = mapped[i] >= 0 && mapped[i] <= len_[i];
  
  return inside;
}

void
rgrl_mask_oriented_box::
set_origin( vnl_vector<double> const& v )
{
  assert( v.size() == origin_.size() || !origin_.size() );
  origin_ = v;
  
  update_bounding_box();
}


void
rgrl_mask_oriented_box::
set_len( vnl_vector<double> const& len )
{
  assert( len.size() == len_.size() || !len_.size() );
  len_ = len;
  
  update_bounding_box();
}

void
rgrl_mask_oriented_box::
set_axes( vnl_matrix<double> const& axes )
{
  // square matrix
  assert( axes.rows() == axes.cols() );

  axes_ = axes;
  
  update_bounding_box();
}

void
rgrl_mask_oriented_box::
update_bounding_box()
{
  assert( origin_.size() == len_.size() );
  assert( origin_.size() == axes_.rows());
  assert( origin_.size() == axes_.cols());

  const unsigned int dim = origin_.size();

  // Extra step:
  // make sure len_[i] >=0 
  // 
  for( unsigned i=0; i<dim; ++i )
    if( len_[i] <= 0 ) {
    
      len_[i] = -len_[i];
      // invert the column vector
      for( unsigned j=0; j<dim; ++j )
        axes_(j, i) = -axes_(j,i);
    }
          
  // use bit pattern to generate all corners
  const unsigned num_corners = 2<<dim;
  
  vnl_vector<double> xmin( origin_ ), xmax( origin_ );
  vnl_vector<double> corner;
  vnl_vector<double> mapped_len = axes_ * len_;
  for( unsigned i=0; i<num_corners; ++i ) {
    
    corner = origin_; 
    // going through exes
    for( unsigned j=0; j<dim; ++j ) {
      
      // multiplication using each bit 0/1 
      corner[j] += ((i>>j)&0x1)?mapped_len[j]:0;
      
      if( corner[j] < xmin[j] )   xmin[j] = corner[j];
      if( corner[j] > xmax[j] )   xmax[j] = corner[j];
    }
  }
  
  x0_ = xmin;
  x1_ = xmax;
}

//: get average distance of corresponding vertices between two oriented box
double 
rgrl_mask_oriented_box::
average_vertices_dist( const rgrl_mask_oriented_box& other ) const
{
  if( origin_.size() != other.origin_.size() )
    return vcl_numeric_limits<double>::infinity();
  
  const unsigned int dim = origin_.size();
  double cum_dist = 0.0;
  
  // use bit pattern to generate all corners
  const unsigned num_corners = 2<<dim;
  vnl_vector<double> corner, other_corner;
  vnl_vector<double> mapped_len = axes_ * len_;
  vnl_vector<double> other_mapped_len = other.axes_ * len_;
  for( unsigned i=0; i<num_corners; ++i ) {
    
    corner = origin_; 
    other_corner = other.origin_;
    
    // going through exes
    for( unsigned j=0; j<dim; ++j ) {
      
      // multiplication using each bit 0/1 
      corner[j] += ((i>>j)&0x1)?mapped_len[j]:0;
      other_corner[j] += ((i>>j)&0x1)?other_mapped_len[j]:0;
    }
    
    cum_dist += (corner-other_corner).two_norm();
  }
  return cum_dist/num_corners;
}

bool
rgrl_mask_oriented_box::
operator==( const rgrl_mask_oriented_box& other ) const
{
  // check the axes first 
  // axes are othogonal matrix
  // therefore the product should be identity matrix
  vnl_matrix<double> prod = vnl_transpose( this->axes_ ) * other.axes_;
  vnl_matrix<double> eye( origin_.size(), origin_.size() );
  eye.set_identity();
  if( (prod - eye).fro_norm() > 1e-8 ) 
    return false;
    
  // now check origin_ and len_
  // 
  return origin_ == other.origin_  &&
         len_ == other.len_;
}

bool
rgrl_mask_oriented_box::
operator!=( const rgrl_mask_oriented_box& other ) const
{
  return !( *this == other );
}

//******************** mask using a box ***********************

rgrl_mask_box::
rgrl_mask_box( unsigned dim )
  : rgrl_mask( dim )
{
}

rgrl_mask_box::
rgrl_mask_box( vnl_vector<double> const& x0, vnl_vector<double> const& x1 )
  : rgrl_mask( x0.size() )
{
  assert( x0.size() == x1.size() );
  
  //check
  for( unsigned i=0; i<x0.size(); ++i )
    assert( x0[i] <= x1[i] );
    
  x0_ = x0;
  x1_ = x1;
}

bool
rgrl_mask_box::
inside( vnl_vector<double> const& pt ) const
{
  assert( pt.size() == x1_.size() );

  bool inside = true;
  for (unsigned i =0; i<pt.size()&&inside; ++i) {
    inside = (x0_[i] <= pt[i] && pt[i] <= x1_[i]);
  }
  return inside;
}

void
rgrl_mask_box::
set_x0( vnl_vector<double> const& v )
{
  assert( v.size() == x0_.size() || !x0_.size() );
  x0_ = v;
}


void
rgrl_mask_box::
set_x1( vnl_vector<double> const& v )
{
  assert( v.size() == x1_.size() || !x1_.size() );
  x1_ = v;
}

bool
rgrl_mask_box::
operator==( const rgrl_mask_box& other ) const
{
  return x0_ == other.x0_  &&
         x1_ == other.x1_;
}

bool
rgrl_mask_box::
operator!=( const rgrl_mask_box& other ) const
{
  return !( *this == other );
}

vcl_ostream& operator<<(vcl_ostream& os, const rgrl_mask_box& box)
{
  os<< box.x0().size() << "  ";
  if( box.x0().size() )
    os << box.x0()<<"  "<<box.x1();
  return os;
}

vcl_istream& operator>>(vcl_istream& is, rgrl_mask_box& box)
{
  int m = -1;
  is >> m;
  
  if( m <= 0 ) return is;
    
  vnl_vector<double> x0(m), x1(m);
  
  is >> x0 >> x1;
  rgrl_mask_box temp_box( x0, x1 );
  box = temp_box;
  return is;
}

//--------------------------------
//               Utility functions

//: Intersection Box A with Box B (make it within the range of B). Then Box A has the result
rgrl_mask_box
rgrl_mask_box_intersection( rgrl_mask_box const& a, rgrl_mask_box const& b )
{
  assert( a.x0().size() == b.x0().size() );
  assert( a.x1().size() == b.x1().size() );

  const unsigned m = a.x0().size();
  vnl_vector<double> new_x0=a.x0();
  vnl_vector<double> new_x1=a.x1();
  const vnl_vector<double>& b_x0=b.x0();
  const vnl_vector<double>& b_x1=b.x1();

  for ( unsigned d=0; d < m; ++d )
  {
    if ( new_x0[d] < b_x0[d] )  new_x0[d] = b_x0[d];
    if ( new_x0[d] > b_x1[d] )  new_x0[d] = b_x1[d];

    if ( new_x1[d] > b_x1[d] )  new_x1[d] = b_x1[d];
    if ( new_x1[d] < b_x0[d] )  new_x1[d] = b_x0[d];
  }

  return rgrl_mask_box( new_x0, new_x1 );
}
