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
//--------------------------------------------------------------
//
// Class : bsol_hough_line_index
//
// Modifications : see bsol_hough_line_index.h
//
//-----------------------------------------------------------------------------
#include <vcl_cmath.h>
#include <vcl_algorithm.h> // vcl_find()
#include <vnl/vnl_math.h>
#include <vsol/vsol_point_2d.h>
#include <vsol/vsol_point_2d_sptr.h>
#include <bsol/bsol_hough_line_index.h>
#ifndef DEGTORAD
#define DEGTORAD vnl_math::pi/180
#endif

//--------------------------------------------------------------
//
// Constructor and Destructor Functions
//
//--------------------------------------------------------------

//---------------------------------------------------------------
//: Simple Constructor
//
bsol_hough_line_index::bsol_hough_line_index(const int r_dimension,
                                             const int theta_dimension)
{
  xo_ = 0; yo_ = 0;
  xsize_ = (int)ceil(r_dimension/vnl_math::sqrt2);
  ysize_ = xsize_;
  angle_range_ = theta_dimension;
  angle_increment_ = 1.0;
  
  this->init(r_dimension, theta_dimension);
}
//--------------------------------------------------------------
//:  A Useful Constructor
//
bsol_hough_line_index::bsol_hough_line_index(const float x0,
                                             const float y0,
                                             const float xsize,
                                             const float ysize,
                                             const float angle_range,
                                             const float angle_increment)
{
  xo_ = x0;  yo_ = y0;
  xsize_ = xsize;
  ysize_ = ysize;
  angle_range_ = angle_range;
  angle_increment_ = angle_increment;

  int theta_dimension = (int)vcl_ceil(angle_range_/angle_increment_);
  theta_dimension++; //Include both 0 and angle_range_
  float diag = vcl_sqrt(xsize*xsize + ysize*ysize);
  int rmax = int(diag);
  rmax++; //Round off.
  this->init(rmax, theta_dimension);
}

// -- Destructor
bsol_hough_line_index::~bsol_hough_line_index()
{
  for(int r=0;r<r_dim_;r++)
    for(int th=0;th<th_dim_;th++)
      delete index_[r][th];
}

//-----------------------------------------------------------------------------
//
//: Compute the bsol_hough_line_index array locations corresponding to a line
//
void bsol_hough_line_index::array_loc(vsol_line_2d_sptr const& line,
                                        float& r, float& theta)
{
  //Compute angle index
  float angle = (float)line->tangent_angle();
 if(angle>180.0)
    angle -= 180.0;
 if(angle>angle_range_)
   {
     vcl_cout << "The line angle was outside the range of bsol_hough_line_index"
          << " space!\n";
      return;
    }

 theta = angle;

 float angrad = DEGTORAD*angle;

 //Compute distance indices
 vsol_point_2d_sptr mid = line->middle();
 float midx = mid->x()-xo_;
 float midy = mid->y()-yo_;
 float xs2 = xsize_/2.0; 
 float ys2 = ysize_/2.0;

 float cx = -(midx-xs2)*vcl_sin(angrad);
 float cy = (midy-ys2)*vcl_cos(angrad);

 //We use the middle of the ranges as the origin to insure minium error
 //Also, the distance index is guaranteed to be positive
 r = cx + cy + vcl_sqrt(xs2*xs2 + ys2*ys2);
}
//-----------------------------------------------------------------------------
//
// -- Compute the bsol_hough_line_index array locations corresponding to a line
//
void bsol_hough_line_index::array_loc(vsol_line_2d_sptr const& line,
                                         int& r, int& theta)
{
  float angle = 0, radius = 0;
  this->array_loc(line, radius, angle);
  theta = (int)floor(angle/angle_increment_);
  r = int(radius);
}

//-----------------------------------------------------------------------------
//
// -- Modify bsol_hough_line_index array R location under translation
//
int bsol_hough_line_index::trans_loc(const int transx, const int transy,
                                        const int r, const int theta)
{
  float angle = angle_increment_*theta;
  float angrad = DEGTORAD*angle;
  int new_cx =  -int(float(transx)*vcl_sin(angrad));
  int new_cy = int(float(transy)*vcl_cos(angrad));
  int newr = new_cx + new_cy;
  newr += r;
  if(newr< 0)
    return 0;
  if(newr > r_dim_)
    return r_dim_;
  return newr;
}

//-----------------------------------------------------------------------------
//
// -- Return the count at a given r and theta
// 
//-----------------------------------------------------------------------------
int bsol_hough_line_index::count(const int r, const int theta)
{
  if(r<0||theta<0||r>=r_dim_||theta>=th_dim_)
    {
      vcl_cout << "Warning - bsol_hough_line_index index outside of range!\n";
      return 0;
    }

  return index_[r][theta]->size();
}

//-----------------------------------------------------------------------------
//
// -- Method to index into bsol_hough_line_index index given an vsol_line_2d_sptr.
//    The timestamp is not updated to facilitate lazy insertion of
//    multiple items.
bool bsol_hough_line_index::index(vsol_line_2d_sptr const& line)
{
 if(!line)
    {
      vcl_cout << "In bsol_hough_line_index::index(..) NULL line\n";
      return false;
    }
  int r, theta;
  this->array_loc(line, r, theta);
  if(!(r < r_dim_)||!(theta < th_dim_))
    return false;

  index_[r][theta]->push_back(line);
  return true;
}

//-----------------------------------------------------------------------------
//
// -- Method to index into bsol_hough_line_index index given an vsol_line_2d_sptr.
//    Only new vsol_line_2d_sptr pointers are added to the bin.
bool bsol_hough_line_index::index_new(vsol_line_2d_sptr const& line)
{
 if(!line)
    {
      vcl_cout << "In bsol_hough_line_index::index_new(..) NULL line\n";
      return false;
    }
 int r, theta;
 //Check array bounds and uniqueness of line
 this->array_loc(line, r, theta);
 if(!(r < r_dim_)||!(theta < th_dim_))
   return false;

 vcl_vector<vsol_line_2d_sptr>* lines = index_[r][theta];
 if (!(vcl_find(lines->begin(), lines->end(), line) == lines->end()))
   return false;

 index_[r][theta]->push_back(line);
 return true;
}
//-----------------------------------------------------------------------------
//
// -- find if a line is in the array
bool bsol_hough_line_index::find(vsol_line_2d_sptr const& line)
{
 int r, theta;
 this->array_loc(line, r, theta);
 vcl_vector<vsol_line_2d_sptr>* lines = index_[r][theta];
 return !(vcl_find(lines->begin(), lines->end(), line) == lines->end());
}

//-----------------------------------------------------------------------------
//
// -- remove a line from the Hough index
//    The timestamp is not updated to facilitate lazy insertion of
//    multiple items.  See vsol_line_2d_sptrGroup as an example.
bool bsol_hough_line_index::remove(vsol_line_2d_sptr const& line)
{
 int r, theta;
 this->array_loc(line, r, theta);
 if(!(r < r_dim_)||!(theta < th_dim_))
   return false;
 vcl_vector<vsol_line_2d_sptr>* lines = index_[r][theta];

 vcl_vector<vsol_line_2d_sptr>::iterator lit = 
   vcl_find(lines->begin(), lines->end(), line);
 if(lit == lines->end())
   return false;
 lines->erase(lit);
 return true;
}

//-----------------------------------------------------------------------------
//
// -- Fill a vector of vsol_line_2d_sptr(s) which are at the index location
// 
//-----------------------------------------------------------------------------
void 
bsol_hough_line_index::lines_at_index(const int r, const int theta,
                                      vcl_vector<vsol_line_2d_sptr>& lines)
{
  lines.clear();
  if((theta<0)||(theta>=th_dim_)||(r<0)||(r>=r_dim_))
    return;

  int count = this->count(r, theta);
  if(count==0)
    return;

  for(int i = 0; i<count; i++)
    lines.push_back((*index_[r][theta])[i]);
}
//-----------------------------------------------------------------------------
//
// -- Return a list of vsol_line_2d_sptr(s) which are at the index location
// 
//-----------------------------------------------------------------------------
vcl_vector<vsol_line_2d_sptr> bsol_hough_line_index::
lines_at_index(const int r, const int theta)
{
  vcl_vector<vsol_line_2d_sptr> out;
  this->lines_at_index(r, theta, out);
  return out;  
}

//-----------------------------------------------------------------------------
//
// -- Fill a list of vsol_line_2d_sptr(s) which are within a distance(radius) 
//    of the r, theta values of a given line.
// 
//-----------------------------------------------------------------------------

void bsol_hough_line_index::
lines_in_interval(vsol_line_2d_sptr const & l,
                  const float r_dist,
                  const float theta_dist,
                  vcl_vector<vsol_line_2d_sptr>& lines)
{
  lines.clear();
  if(!l)
    {
      vcl_cout << "In bsol_hough_line_index::lines_in_interval(..) NULL line\n";
      return;
    }
  int r, theta;
  this->array_loc(l, r, theta);
  int angle_radius = (int)vcl_ceil(theta_dist/angle_increment_);
  int r_radius = (int)vcl_ceil(r_dist);
  int th_dim_m1 = th_dim_ - 1;

  for(int i = -angle_radius; i<=angle_radius; i++)
    {
      //The angle space is circular
      int t_indx = (theta + i) % (th_dim_m1);
      if(t_indx<0)
        t_indx += th_dim_m1;
      for(int j = -r_radius; j<=r_radius; j++)
	{
	  int r_indx = r + j;
	  if((r_indx<0)||(r_indx>=r_dim_))
	    continue;
	  vcl_vector<vsol_line_2d_sptr> temp;
	  this->lines_at_index(r_indx, t_indx,temp);
	  for(vcl_vector<vsol_line_2d_sptr>::iterator lit = temp.begin();
        lit != temp.end(); lit++)
	    {
	      //Note, these tests should eventually be more
	      //sophisticated - JLM
	      vsol_line_2d_sptr line = *lit;
	      float l_angle, line_angle;
	      float l_ndist, line_ndist;
	      this->array_loc(l, l_ndist, l_angle);
	      this->array_loc(line, line_ndist, line_angle);

	      //Test error in normal distance
	      bool within_r_radius = fabs(l_ndist - line_ndist) < r_dist;
	      if(!within_r_radius)
		continue;

	      //Test angular error
	      bool within_angle_radius = fabs(l_angle - line_angle) < theta_dist;
	      if(!within_angle_radius)
		continue;

	      //line, passed both tests
	      lines.push_back(line);
	    }
	}
    }
}

//-----------------------------------------------------------------------------
//
// -- Return a list of vsol_line_2d_sptr(s) which are within a distance(radius) 
//    of the r, theta values of a given line.
// 
//-----------------------------------------------------------------------------

vcl_vector<vsol_line_2d_sptr>
bsol_hough_line_index::lines_in_interval(vsol_line_2d_sptr const & l,
                                            const float r_dist,
                                            const float theta_dist)
{
  vcl_vector<vsol_line_2d_sptr> out;
 if(!l)
    {
      vcl_cout << "In bsol_hough_line_index::lines_in_interval(..) NULL line\n";
      return out;
    }

  this->lines_in_interval(l, r_dist, theta_dist, out);
  return out;
}
//-----------------------------------------------------------------------------
//
// -- Fill a list of vsol_line_2d_sptr(s) which are within angle_dist of
//    of a given angle
// 
//-----------------------------------------------------------------------------

void 
bsol_hough_line_index::parallel_lines(const float angle, 
                                         const float angle_dist,
                                         vcl_vector<vsol_line_2d_sptr>& lines)
{
  lines.clear();

  //Compute angle index and tolerance
  float ang = angle;
  if(ang>180.0)
    ang -= 180.0;
  if(ang>angle_range_)
    {
      vcl_cout << "The line angle was outside the range "
           << " of bsol_hough_line_index Space!\n";
      return;
    }
  int theta = (int)vcl_floor(ang/angle_increment_);
  int angle_radius = (int)vcl_ceil(angle_dist/angle_increment_);
  int th_dim_m1 = th_dim_ - 1;

  for(int i = -angle_radius; i<=angle_radius; i++)
    {
      //The angle space is circular
      int t_indx = (theta + i) % (th_dim_m1);
      if(t_indx<0)
        t_indx += th_dim_m1;
      for(int j = 0; j<r_dim_; j++)
        {
          if(!(this->count(j, t_indx)>0))
            continue;
          vcl_vector<vsol_line_2d_sptr> temp;
          this->lines_at_index(j, t_indx, temp);
	  for(vcl_vector<vsol_line_2d_sptr>::iterator lit = temp.begin();
        lit != temp.end(); lit++)
	    {
	      vsol_line_2d_sptr line = *lit;
	      //Test angular error
	      float line_angle = (float)line->tangent_angle();
	      if(line_angle>180.0)
          line_angle -= 180.0;
	      float ang_error = fabs(ang - line_angle);
	      if(ang_error<angle_dist)
          lines.push_back(line);
	    }
        }
    }
}

//-----------------------------------------------------------------------------
//
// -- Return a list of vsol_line_2d_sptr(s) which are within angle_dist of
//    of a given angle
// 
//-----------------------------------------------------------------------------

vcl_vector<vsol_line_2d_sptr >
 bsol_hough_line_index::parallel_lines(const float angle, 
                                          const float angle_dist)
{
  vcl_vector<vsol_line_2d_sptr> out;
  this->parallel_lines(angle, angle_dist, out);
  return out;
}

//-----------------------------------------------------------------------------
//
// -- Fill a list of vsol_line_2d_sptr(s) which are oriented at an angle with
//    respect to a given line given a specified angular tolerance.
// 
//-----------------------------------------------------------------------------

void 
bsol_hough_line_index::lines_at_angle(vsol_line_2d_sptr const &l, 
                                         const float angle,
                                         const float angle_dist,
                                         vcl_vector<vsol_line_2d_sptr >& lines)
{
  lines.clear();
  if(!l)
    {
      vcl_cout << "In bsol_hough_line_index::lines_at_angle(..) NULL line\n";
      return;
    }
  float line_angle = (float)l->tangent_angle();
  if(line_angle>180.0)
    line_angle -= 180.0;
  float ang = line_angle + angle;
  this->parallel_lines(ang, angle_dist, lines);
}

//-----------------------------------------------------------------------------
//
// -- Return a new list of vsol_line_2d_sptr(s) which are oriented at an angle with
//    respect to a given line given a specified angular tolerance.
//-----------------------------------------------------------------------------

vcl_vector<vsol_line_2d_sptr>
bsol_hough_line_index::lines_at_angle(vsol_line_2d_sptr const &l, 
                                         const float angle,
                                         const float angle_dist)
{
  vcl_vector<vsol_line_2d_sptr> out;
  this->lines_at_angle(l, angle, angle_dist, out);
  return out;
}

//-----------------------------------------------------------------------------
//
// -- Fill a list of vsol_line_2d_sptr(s) which are within angle_dist of
//    of the orientation of a given line.
// 
//-----------------------------------------------------------------------------

void 
bsol_hough_line_index::parallel_lines(vsol_line_2d_sptr const &l, 
                                         const float angle_dist,
                                         vcl_vector<vsol_line_2d_sptr>& lines)
{
  lines.clear();
 if(!l)
    {
      vcl_cout << "In bsol_hough_line_index::parallel_lines(..) NULL line\n";
      return;
    }
 float angle = (float)l->tangent_angle();
 this->parallel_lines(angle, angle_dist, lines);
}

//-----------------------------------------------------------------------------
//
// -- Return a list of vsol_line_2d_sptr(s) which are within angle_dist of
//    of the orientation of a given line.
// 
//-----------------------------------------------------------------------------

vcl_vector<vsol_line_2d_sptr>
bsol_hough_line_index::parallel_lines(vsol_line_2d_sptr const &l,
                                         const float angle_dist)
{
  vcl_vector<vsol_line_2d_sptr> out;
  if(!l)
    {
      vcl_cout << "In bsol_hough_line_index::parallel_lines(..) NULL line\n";
      return out;
    }
  this->parallel_lines(l, angle_dist, out);
  return out;
}

//-----------------------------------------------------------------------------
//
// -- Clear the bsol_hough_line_index index space
// 
//-----------------------------------------------------------------------------
void bsol_hough_line_index::clear_index()
{
  for(int r=0;r<r_dim_;r++)
    for(int th=0;th<th_dim_;th++)
      index_[r][th]->clear();
}

//-----------------------------------------------------------------
//: Constructor Utility
//
void bsol_hough_line_index::init(const int r_dimension, 
                                    const int theta_dimension)
{
  r_dim_ = r_dimension;
  th_dim_ = theta_dimension;
  index_.resize(r_dim_, th_dim_);
  for(int r=0;r<r_dim_;r++)
    for(int th=0;th<th_dim_;th++)
      index_.put(r,th, new vcl_vector<vsol_line_2d_sptr>);
}

//-----------------------------------------------------------------
//: An image of the hough index, useful for debugging applications
//  that use this class
vbl_array_2d<unsigned char> bsol_hough_line_index::get_hough_image()
{
  vbl_array_2d<unsigned char> out(r_dim_, th_dim_);
  float nmax = 0;
  for(int r=0;r<r_dim_;r++)
    for(int th=0;th<th_dim_;th++)
      {
        int n_lines = index_[r][th]->size();
        if(n_lines>nmax)
          nmax = n_lines;
      }
  float scale = 1;
  if(nmax)
    scale = 1/nmax;
  for(int r=0;r<r_dim_;r++)
    for(int th=0;th<th_dim_;th++)
      {
        unsigned char val = 0;
        int n_lines = index_[r][th]->size();
        float v = 255*n_lines/scale;
        val = (unsigned char)v;
        out.put(r,th,val);
      }
  return out;
}
