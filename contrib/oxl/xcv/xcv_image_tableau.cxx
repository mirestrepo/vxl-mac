/*
  u97mb@robots.ox.ac.uk
*/
#include "xcv_image_tableau.h"

#include <vcl_string.h>
#include <vcl_cmath.h>

#include <vil/vil_crop.h>
#include <vil/vil_image.h>

#include <vgui/vgui_event.h>
#include <vgui/vgui_matrix_state.h>
#include <vgui/vgui_gl.h>

//--------------------------------------------------------------------------------
  
xcv_image_tableau::xcv_image_tableau() 
  : defined_(false) { }

xcv_image_tableau::xcv_image_tableau(vil_image const &I) 
  : base(I), defined_(false) { }

xcv_image_tableau::xcv_image_tableau(char const *f)
  : base(f), defined_(false) { }

vcl_string xcv_image_tableau::type_name() const 
{
  return "xcv_image_tableau"; 
}

//--------------------------------------------------------------------------------

vil_image xcv_image_tableau::get_image() const 
{
  if(!defined_)
    return base::get_image();
  else 
    return vil_crop(base::get_image(),
		    int(roi_.x+0.5),int(roi_.y+0.5),
		    int(roi_.width),int(roi_.height));
}

void xcv_image_tableau::set_roi(float x,float y,float w,float h)
{
  defined_ = true;
  roi_.x = x;
  roi_.y = y;
  roi_.width = w;
  roi_.height = h;
}
void xcv_image_tableau::unset_roi()
{
  defined_ = false;
}
//--------------------------------------------------------------------------------

unsigned xcv_image_tableau::width() const 
{
  if(!defined_)
    return base::width();
  else
    return int(roi_.width);
}

unsigned xcv_image_tableau::height() const
{
  if(!defined_)
    return base::height();
  else
    return int(roi_.height);
}

bool xcv_image_tableau::get_bounding_box(float low[3], float high[3]) const 
{
  if(defined_) {
    low[0] = roi_.x;
    low[1] = roi_.y;
    low[2] = 0; 
    high[0] = roi_.x+roi_.width;
    high[1] = roi_.y+roi_.height;
    high[2] = 0;
    return true;
  }
  else
    return base::get_bounding_box(low, high);
}

//--------------------------------------------------------------------------------

bool xcv_image_tableau::handle(vgui_event const &e) 
{
  //
  if (e.type == vgui_DRAW) {
    base::handle(e);
    if(defined_) {
      // -- Draw a region of interest
      glLineWidth(1);
	  glColor3f(0,1,0);
	  glBegin(GL_LINE_LOOP);
      glVertex2f(roi_.x,roi_.y);
      glVertex2f(roi_.x+roi_.width,roi_.y);
      glVertex2f(roi_.x+roi_.width,roi_.y+roi_.height);
      glVertex2f(roi_.x,roi_.y+roi_.height);
      glEnd();
    }
    return true;
  }
  
  else 
    return base::handle(e);
}
vgui_roi_tableau_make_roi::vgui_roi_tableau_make_roi(xcv_image_tableau_ref const& imt)
{
  image_tableau_ = imt;
  done_ = false;
}
void vgui_roi_tableau_make_roi::add_box(float x0,float y0,float x1,float y1)
{
  float sx = x0>x1 ? x1:x0;
  float sy = y0>y1 ? y1:y0;
  float w = vcl_fabs(x1-x0),h = vcl_fabs(y1-y0); 

  image_tableau_->set_roi(sx,sy,w,h);
  done_ = true;
}
//--------------------------------------------------------------------------------
