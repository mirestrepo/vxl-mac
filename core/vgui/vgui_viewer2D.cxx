//-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation
#endif
//
// .NAME vgui_viewer2D
// Author: Philip C. Pritchett, RRG, University of Oxford
// Created: 14 Sep 99
//
//-----------------------------------------------------------------------------

#include "vgui_viewer2D.h"

#include <vcl_cmath.h>
#include <vbl/vbl_bool_ostream.h>

#include <vgui/vgui_gl.h>
#include <vgui/vgui_glu.h>
#include <vgui/vgui.h>
#include <vgui/vgui_event.h>
#include <vgui/vgui_event_condition.h>
#include <vgui/vgui_drag_mixin.h>
#include <vgui/vgui_matrix_state.h>
#include <vgui/vgui_find.h>
#include <vgui/vgui_utils.h>
#include <vgui/vgui_projection_inspector.h>
#include <vgui/vgui_image_tableau.h> // for centering

/*

to transform from window coords (x,y) to image coords (ix,iy)

  ix = (x - token.offsetX) / token.scaleX;
  iy = (y - token.offsetY) / token.scaleY;

*/

/*
  h = w * (H/W)
  w = h * (W/H)

 */


static bool debug=false;

const void * const vgui_viewer2D::CENTER_EVENT="x";

// this is what it always was. please leave it. -- fsm.
vgui_event_condition c_pan(vgui_MIDDLE, vgui_CTRL);
//vgui_event_condition c_pan(vgui_LEFT, vgui_modifier(vgui_CTRL + vgui_SHIFT));

vgui_viewer2D::vgui_viewer2D(vgui_tableau_sptr const& s) :
  vgui_wrapper_tableau(s),
  nice_points(true),
  nice_lines(true),
  zoom_type(normal_zoom),
  panning(false),
  smooth_zooming(false),
  sweep_zooming(false),
  sweep_next(false),
  prev_x(0), prev_y(0),
  zoom_x(0), zoom_y(0),
  zoom_factor(1.5)
{
}

vgui_viewer2D::~vgui_viewer2D()
{
}

void vgui_viewer2D::setup_gl_matrices()
{
  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);
  int width  = vp[2];
  int height = vp[3];

  // the projection matrix sets up GL for
  // rendering in window coordinates.
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, width,  // left, right
          height, 0, // bottom, top
          -1,+1);    // near, far

  // the modelview matrix applies a tranformation
  // to incoming coordinates before they reach the
  // projection matrix. in this case, it consists
  // of anisotropic scaling about (0,0) followed
  // by a translation.
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(token.offsetX, token.offsetY, 0);
  glScalef(token.scaleX, token.scaleY, 1);
}



// this routine will modify the token in such a way as to
// effect a zoom about the point (x, y) by the given factor.
// (x, y) are in viewport coords.
void vgui_viewer2D::zoomin(float zoom_factor, int x, int y) {
  // this bit is easy.
  token.scaleX *= zoom_factor;
  token.scaleY *= zoom_factor;

  // this bit is tricky.
  GLint vp[4]; glGetIntegerv(GL_VIEWPORT,vp);
  float dx = (        (x-vp[0])) - token.offsetX;
  float dy = (vp[3]-1-(y-vp[1])) - token.offsetY;

  float tmpx = zoom_factor*dx - dx;
  float tmpy = zoom_factor*dy - dy;

  token.offsetX -= tmpx;
  token.offsetY -= tmpy;
}


void vgui_viewer2D::zoomout(float zoom_factor, int x, int y) {
  zoomin(1.0 / zoom_factor, x, y);
}

void vgui_viewer2D::center_image(int w, int h) {

  GLfloat vp[4];
  glGetFloatv(GL_VIEWPORT, vp);
  float width = vp[2];
  float height = vp[3];

  token.offsetX =  width/2 - token.scaleX*(float(w)/2.0);
  token.offsetY = height/2 - token.scaleY*(float(h)/2.0);

  post_redraw();
}


vcl_string vgui_viewer2D::type_name() const {return "vgui_viewer2D";}

static void draw_rect(float x0, float y0, float x1, float y1) {
  glColor3f(1,0,0);

  glLineWidth(2);
  glBegin(GL_LINE_LOOP);
  glVertex2f(x0,y0);
  glVertex2f(x0,y1);
  glVertex2f(x1,y1);
  glVertex2f(x1,y0);
  glEnd();
}

bool vgui_viewer2D::handle(const vgui_event& e) {

  if (e.type == vgui_DRAW) {

    // Setup OpenGL for 2D
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glShadeModel(GL_FLAT);


    if (nice_points)
      glEnable(GL_POINT_SMOOTH);
    else
      glDisable(GL_POINT_SMOOTH);

    if (nice_lines) {
      glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
      glEnable(GL_LINE_SMOOTH);
      glLineWidth (1.5);
    }
    else
      glDisable(GL_LINE_SMOOTH);

    if (nice_points || nice_lines) {
      glEnable (GL_BLEND);
      glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      // glBlendFunc (GL_SRC_ALPHA,  GL_ONE);
    } else {
      glDisable(GL_BLEND);
    }

    setup_gl_matrices();

    return child->handle(e);
  }
  // -- This deals with horizontal scroll message
  if(e.type == vgui_HSCROLL)
  {
        static int npos = 0;
        this->token.offsetX -= *((int *)e.data)-npos;
    this->post_redraw();
        npos = *((int *)e.data);

  }
  // -- This deals with vertical scroll message
  if(e.type == vgui_VSCROLL)
  {
        static int npos = 0;
        this->token.offsetY -= *((int *)e.data)-npos;
        this->post_redraw();
        npos = *((int *)e.data);
  }
  setup_gl_matrices();

  if (vgui_drag_mixin::handle(e)) {
    return true;
  }

  if (vgui_tableau::handle(e)) {
    return true;
  }

  return child->handle(e);
}

bool vgui_viewer2D::mouse_down(int x, int y, vgui_button button, vgui_modifier modifier) {

  // Middle mouse button press.  Update last seen mouse position. And set
  //  "panning" true since button is pressed.
  if (c_pan(button, modifier)) {
    if (debug) vcl_cerr << "vgui_viewer2D_handler::middle " << vcl_endl;

    prev_x = x;
    prev_y = y;
    panning = true;
    return true;
  }
  if (this->zoom_type == vgui_viewer2D::normal_zoom && !sweep_next) {
    if (button == vgui_LEFT && (modifier & vgui_CTRL)) {
      if (debug) vcl_cerr << "vgui_viewer2D_handler::left" << vcl_endl;
      this->zoomin(zoom_factor, int(x), int(y));
      this->post_redraw();
      return true;
    }
    else if (button == vgui_RIGHT && (modifier & vgui_CTRL)) {
      if (debug) vcl_cerr << "vgui_viewer2D_handler::right" << vcl_endl;
      this->zoomout(zoom_factor, int(x), int(y));
      this->post_redraw();
      return true;
    }
  }
  else if (this->zoom_type == vgui_viewer2D::smooth_zoom && !sweep_next) {// if this->smooth_zoom
    if (button == vgui_LEFT && (modifier & vgui_CTRL)) {
      if (debug) vcl_cerr << "vgui_viewer2D_handler::left" << vcl_endl;
      prev_x = x;
      prev_y = y;
      zoom_x = x;
      zoom_y = y;
      smooth_zooming = true;

      return true;
    }
  }
  else {
    if (button == vgui_LEFT && (sweep_next || (modifier & vgui_CTRL))) {
      // this is the beginning of the sweep zoom operation. we have to (a) set the
      // state flag 'sweep_zooming', (b) remember the position of the pointer and
      // (c) save the front buffer to the back buffer.
      prev_x = x;
      prev_y = y;
      zoom_x = x;
      zoom_y = y;
      sweep_zooming = true;
      //cerr << "copy_back_to_front..." << flush;
      vgui_utils::copy_back_to_front();
      //cerr << "done" << endl;

      return true;
    }
  }


  return false;
}

bool vgui_viewer2D::mouse_drag(int x, int y,  vgui_button /*button*/, vgui_modifier /*modifier*/) {
  if (debug) vcl_cerr << __FILE__ ": vgui_viewer2D_handler::mouse_drag" << vcl_endl;

  if (!panning && !smooth_zooming && !sweep_zooming)
    return false;

  if (panning) {
    // the mouse events come in viewport coordinates, so the relevant
    // translation in window coordinates is (dx,-dy).
    this->token.offsetX += (x-prev_x);
    this->token.offsetY -= (y-prev_y);
    this->post_redraw();
  }

  if (smooth_zooming) {
    GLdouble vp[4];
    glGetDoublev(GL_VIEWPORT, vp);
    double height = vp[3];

    double newscale = 1 - (1.5*(prev_y - y)/height);

    this->zoomin(newscale, int(zoom_x), int(zoom_y));
    this->post_redraw();
  }

  if (sweep_zooming) {
    // this is called during the sweep zoom operation. we have
    // to (a) repair the front buffer, (b) draw the new rectangle
    // and (c) remember where the mouse pointer was (in new_x, new_y).
    vgui_matrix_state gl_state;

    //cerr << "begin_sw_overlay..." << flush;
    vgui_utils::begin_sw_overlay();
    //cerr << "done" << endl;

    //cerr << "copy_back_to_front..." << flush;
    vgui_utils::copy_back_to_front();
    //cerr << "done" << endl;

    // set projection*modelview matrices to render in **viewport** coordinates :
    GLdouble vp[4];
    glGetDoublev(GL_VIEWPORT, vp);
    double width = vp[2];
    double height = vp[3];

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, width, 0, height);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // get ratio of viewport width to viewport height :
    double W_H_ratio = width/height;

    // new method - draws rectangular box of aspect ratio W/H
    new_x = x;
    new_y = y;

    // compute the size of the sweep area in pixels on the screen :
    float xdiff = fabs(zoom_x - new_x);
    float ydiff = fabs(zoom_y - new_y);

    // this bit here makes sure the swept out region has the
    // same shape as the viewport :
    if (xdiff > ydiff*W_H_ratio) {
      if ((zoom_y - y) > 0)
        new_y = zoom_y - xdiff/W_H_ratio;
      else
        new_y = zoom_y + xdiff/W_H_ratio;
    }
    else {
      if ((zoom_x - x) > 0)
        new_x = zoom_x - ydiff*W_H_ratio;
      else
        new_x = zoom_x + ydiff*W_H_ratio;
    }

    // useful line as it documents the meaning of
    // zoom_? and new_?
    draw_rect(zoom_x - vp[0], zoom_y - vp[1], new_x - vp[0], new_y - vp[1]);

    vgui_utils::end_sw_overlay();
  }

  // Update last seen mouse position.
  prev_x = x;
  prev_y = y;
  return true;
}

bool vgui_viewer2D::mouse_up(int /*x*/, int /*y*/,  vgui_button button, vgui_modifier /*modifier*/) {
  if (debug) vcl_cerr << "vgui_viewer2D_handler::mouse_up" << vcl_endl;

  if (sweep_zooming && button == vgui_LEFT) {
    // this is the end of the sweepzoom operation.
    // we have to deduce and set the new parameters in the token
    // from the viewport coordinates (zoom_x, zoom_y) and (new_x, new_y).
    // the safe way to do this is convert the sweep region into pre-modelview
    // coordinates and then deduce the token parameters from that.
    //
    // we modify the token to make the sweep region fill the whole viewport.
    sweep_zooming = false;
    sweep_next = false;

    // get size of viewport
    GLfloat vp[4];
    glGetFloatv(GL_VIEWPORT, vp);

    // compute pre-modelview coordinates of corners of sweep region :
    float x1 = (        (zoom_x-vp[0]) - this->token.offsetX) / this->token.scaleX;
    float y1 = (vp[3]-1-(zoom_y-vp[1]) - this->token.offsetY) / this->token.scaleY;
    float x2 = (        ( new_x-vp[0]) - this->token.offsetX) / this->token.scaleX;
    float y2 = (vp[3]-1-( new_y-vp[1]) - this->token.offsetY) / this->token.scaleY;

    // set the new parameters in the token :
    this->token.scaleX = vp[2]/(x2-x1);
    this->token.scaleY = vp[3]/(y2-y1);
    this->token.offsetX = - this->token.scaleX*x1;
    this->token.offsetY = - this->token.scaleY*y1;

    this->post_redraw(); // we probably need one now
  }


  if (smooth_zooming && button == vgui_LEFT) {
    smooth_zooming = false;
  }

  if (panning && button == c_pan.button) {
    panning = false;
  }

  return false;
}

bool vgui_viewer2D::help() {
  vcl_cerr << vcl_endl;
  vcl_cerr << "-- vgui_viewer2D ----------" << vcl_endl;
  vcl_cerr << "|     mouse               |"  << vcl_endl;
  vcl_cerr << "| ctrl+left       zoom in |"  << vcl_endl;
  vcl_cerr << "| ctrlt+middle        pan |"  << vcl_endl;
  vcl_cerr << "| ctrl+right     zoom out |"  << vcl_endl;
  vcl_cerr << "|                         |" << vcl_endl;
  vcl_cerr << "|     keys                |"  << vcl_endl;
  vcl_cerr << "| ctrl+`c'   center image |"  << vcl_endl;
  vcl_cerr << "| ctrl+`x'   resize image |"  << vcl_endl;
  vcl_cerr << "| `-'   lower zoom factor |"  << vcl_endl;
  vcl_cerr << "| `='   raise zoom factor |"  << vcl_endl;
  vcl_cerr << "| `n'     toggle aliasing |"  << vcl_endl;
  vcl_cerr << "| `z'    toggle zoom type |"  << vcl_endl;
  vcl_cerr << "| `d'          sweep zoom |"  << vcl_endl;
  vcl_cerr << "--------------------------" << vcl_endl;
  vcl_cerr << vcl_endl;
  return false;
}

void vgui_viewer2D::center_event() {
#if 0
  vgui_event eprime;
  eprime.user = (void*) &vgui_viewer2D::CENTER_EVENT;
  eprime.data = (void*) viewer;
  this->handle(eprime);
#else
  vgui_tableau_sptr t = vgui_find_below_by_type_name(this, "vgui_image_tableau");
  if (!t)
    t = vgui_find_below_by_type_name(this, "xcv_image_tableau");
  if (t) {
    vgui_image_tableau_sptr im; im.vertical_cast(t);
    this->center_image(im->width(), im->height());
  }
  else
    vcl_cerr << __FILE__ " : no image found" << vcl_endl;
#endif
}

bool vgui_viewer2D::key_press(int /*x*/, int /*y*/, vgui_key key, vgui_modifier modifier) {
  if (debug) vcl_cerr << "vgui_viewer2D_handler::key_press " << key << vcl_endl;

  switch(key) {
  case 'x':
    /*if ((modifier & vgui_CTRL))*/ {
      vgui::out << "viewer2D : resizing image" << vcl_endl;
      this->token.scaleX = 1;
      this->token.scaleY = 1;
      center_event();
      this->post_redraw();
      return true;
    }
    /* else return false; */
  case 'c':
    /*if ((modifier & vgui_CTRL))*/ {
      vgui::out << "viewer2D : centering image" << vcl_endl;
      center_event();
      return true;
    }
    /* else return false; */
  case '-':
    zoom_factor -= 0.1f;
    vgui::out << "viewer2D : zoom_factor = " << zoom_factor << vcl_endl;
    return true;
  case '=':
    zoom_factor += 0.1f;
    vgui::out << "viewer2D : zoom_factor = " << zoom_factor << vcl_endl;
    return true;
  case 'n':
    this->nice_points = !this->nice_points;
    this->nice_lines = !this->nice_lines;
    vgui::out << "viewer2D : antialiased points & lines " << vbl_bool_ostream::on_off(this->nice_points) << vcl_endl;
    this->post_redraw();
    return true;
  case 'd':
    sweep_next = true;;
    return true;
  case 'z':
    if (this->zoom_type == vgui_viewer2D::normal_zoom) {
      this->zoom_type = vgui_viewer2D::smooth_zoom;
      vgui::out << "viewer2D : smooth" << vcl_endl;
    }
    else {
      this->zoom_type = vgui_viewer2D::normal_zoom;
      vgui::out << "viewer2D : normal" << vcl_endl;
    }
    return true;
  default:
    return false;
  }
}
