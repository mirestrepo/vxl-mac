// This is brl/bbas/bgui/bgui_displaylist2D_tableau.cxx
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma implementation
#endif
//:
// \file
// \author Philip C. Pritchett, RRG, University of Oxford
// \date   14 Sep 99
// \brief  See bgui_displaylist2D_tableau.h for a description of this file.

#include "bgui_displaylist2D_tableau.h"

#include <vcl_iostream.h>
#include <vcl_vector.h>

#include <vgui/vgui_gl.h>
#include <vgui/vgui_glu.h>

#include <vgui/vgui.h>
#include <vgui/vgui_utils.h>
#include <vgui/vgui_event.h>
#include <vgui/vgui_projection_inspector.h>
#include <vgui/vgui_soview2D.h>
#include <vgui/vgui_style.h>

bgui_displaylist2D_tableau::bgui_displaylist2D_tableau() :
  posted_redraw_(false)
{
}

bgui_displaylist2D_tableau::~bgui_displaylist2D_tableau()
{
}

bool bgui_displaylist2D_tableau::handle(const vgui_event& e) {

  // Send to motion/drag functions
  if (vgui_tableau::handle(e))
    return true;

  // if mouse leaves the context then unhighlight
  // the highlit object
  if (e.type == vgui_LEAVE)
  {
#if 0 // commented out
    vgui_utils::begin_sw_overlay();

    unsigned highlighted = this->get_highlighted();

    if (highlighted)
    {
      //vcl_cerr << "unhighlighting : " << highlighted << vcl_endl;
      vgui_soview* so = vgui_soview::id_to_object(highlighted);
      vgui_style* style = so->get_style();
      glPointSize(style->point_size);
      glLineWidth(style->line_width);

      if (this->is_selected(highlighted))
        glColor3f(1.0, 0.0, 0.0);
      else
        glColor3f(style->rgba[0],style->rgba[1],style->rgba[2]);

      so->draw();
    }

    vgui_utils::end_sw_overlay();
#endif // 0
    this->highlight(0);
    this->post_redraw();
    posted_redraw_ = true;
    return true;
  }

  return bgui_displaybase_tableau::handle(e);
}

//: Return indices of my elements which are near (x,y)
void bgui_displaylist2D_tableau::get_hits(float x, float y, vcl_vector<unsigned>& my_hits) {
  GLuint *ptr = vgui_utils::enter_pick_mode(x,y,100);

  this->gl_mode = GL_SELECT;
  this->handle(vgui_event(vgui_DRAW));
  this->gl_mode = GL_RENDER;

  int num_hits = vgui_utils::leave_pick_mode();

  // get all hits
  vcl_vector<vcl_vector<unsigned> > hits;
  vgui_utils::process_hits(num_hits, ptr, hits);

  // for each hit get the name of the soview if it is
  // being managed by this vcl_list

  for (vcl_vector<vcl_vector<unsigned> >::iterator i=hits.begin();
       i != hits.end(); ++i) {
    vcl_vector<unsigned> names = *i;

    for (vcl_vector<unsigned>::iterator n_iter = names.begin();
         n_iter != names.end(); ++n_iter) {
      unsigned name = *n_iter;

      for (vcl_vector<vgui_soview*>::iterator so_iter = this->objects.begin();
           so_iter != this->objects.end(); ++so_iter) {
        if ((*so_iter)->get_id() == name) {
          my_hits.push_back(name);
          break;
        }
      }
    }
  }
}

unsigned bgui_displaylist2D_tableau::find_closest(float x, float y, vcl_vector<unsigned>& hits) {
  unsigned closest = 0;
  float closest_dist = -1; // vnl_numeric_traits<float>::maxval;

  for (vcl_vector<unsigned>::iterator h_iter = hits.begin();
       h_iter != hits.end(); ++h_iter) {

    vgui_soview2D* so = static_cast<vgui_soview2D*>(vgui_soview::id_to_object(*h_iter));

    float dist = so->distance_squared(x,y);
    //vcl_cerr << " distance to " << (void*)so << " = " << dist << vcl_endl;

    if (closest_dist<0 || dist<closest_dist) {
      closest_dist = dist;
      closest = *h_iter;
    }
  }// end for

  return closest;
}

bool bgui_displaylist2D_tableau::motion(int x, int y) {

  vgui_projection_inspector pi;
  float ix, iy;
  pi.window_to_image_coordinates(x,y, ix,iy);

  vcl_vector<unsigned> hits;
  get_hits(x,y,hits);
  unsigned closest_id = find_closest(ix,iy,hits);

  if (closest_id == this->get_highlighted() && !posted_redraw_) {
    return false;
  }
#if 0 // commented out
  posted_redraw_ = false;

  vgui_utils::begin_sw_overlay();

  unsigned highlighted = this->get_highlighted();

  if (highlighted) {
    //vcl_cerr << "unhighlighting : " << highlighted << vcl_endl;
    vgui_soview* so = vgui_soview::id_to_object(highlighted);
    vgui_style* style = so->get_style();
    glPointSize(style->point_size);
    glLineWidth(style->line_width);
    //    vgui::out << vcl_endl << *so;
    if (this->is_selected(highlighted))
      glColor3f(1.0, 0.0, 0.0);
    else {
      glColor3f(style->rgba[0],style->rgba[1],style->rgba[2]);
    }
    so->draw();
  }

  if (closest_id) {
    //vcl_cerr << "highlighting : " << closest_id << vcl_endl;

    vgui_soview* so = vgui_soview::id_to_object(closest_id);
    vgui_style* style = so->get_style();
    glPointSize(style->point_size);
    glLineWidth(style->line_width);
    glColor3f(0.0,0.0,1.0);
    so->draw();
    //    vgui::out<<vcl_endl<<*so;
  }
  //  else
  //    vgui::out<<vcl_endl;
  vgui_utils::end_sw_overlay();
#endif // 0

  if ( highlighted != closest_id ) {
    this->highlight(closest_id);
    this->post_redraw();
    posted_redraw_ = true;
  }

  return false;
}

bool bgui_displaylist2D_tableau::mouse_down(int x, int y, vgui_button button, vgui_modifier modifier)
{
  float ix, iy;
  vgui_projection_inspector().window_to_image_coordinates(x,y, ix,iy);

  // selecting
  if (button == vgui_LEFT && modifier == 0) {
#ifdef DEBUG
    vcl_cerr << "selecting at " << x << ' ' << y << vcl_endl;
#endif

    vcl_vector<unsigned> hits;
    get_hits(x,y,hits);
    unsigned closest_id = find_closest(ix,iy,hits);
    if (closest_id) {
      this->select(closest_id);
      this->post_redraw();
      posted_redraw_ = true;
      return true;
    }

    return false;
  }// end selecting

  // deselecting
  else if (button == vgui_MIDDLE) {

    if (modifier & vgui_SHIFT) {
#ifdef DEBUG
      vcl_cerr << "deselecting all\n";
#endif
      this->deselect_all();
      this->post_redraw();
      posted_redraw_ = true;
      return false;
    }

#ifdef DEBUG
    vcl_cerr << "deselecting at " << x << ' ' << y << vcl_endl;
#endif

    vcl_vector<unsigned> hits;
    get_hits(x,y,hits);
    unsigned closest_id = find_closest(ix,iy,hits);
    if (closest_id) {
      this->deselect(closest_id);
      this->post_redraw();
      posted_redraw_ = true;
      return true;
    }

    return false;
  }// end deselecting
  return false;
}// end mouse_down
