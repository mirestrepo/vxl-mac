#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma implementation
#endif
//:
// \file
// \author Philip C. Pritchett, RRG, University of Oxford
// \date   24 Mar 99
// \brief  See vgui_soview2D.h for a description of this file.

#include "vgui_soview2D.h"

#include <vcl_cmath.h>
#include <vcl_iostream.h>

#include <vgl/vgl_distance.h>
#include <vnl/vnl_math.h>

#include <vgui/vgui_gl.h>
#include <vgui/vgui_style.h>
#include <vgui/vgui_projection_inspector.h>
#include <vgui/internals/vgui_draw_line.h>

vgui_soview2D::vgui_soview2D() {}

//--------------------------------------------------------------------------//

vcl_ostream& vgui_soview2D_point::print(vcl_ostream& s) const
{
  s << "[vgui_soview2D_point " << x << "," << y << " ";
  return vgui_soview2D::print(s) << "]";
}

void vgui_soview2D_point::draw() const
{
  glPointSize(style->point_size);
  glBegin(GL_POINTS);
  glVertex2f(x,y);
  glEnd();
}

float vgui_soview2D_point::distance_squared(float x, float y) const
{
  float dx = this->x - x;
  float dy = this->y - y;
  return dx*dx + dy*dy;
}

void vgui_soview2D_point::get_centroid(float* x, float* y) const
{
  *x = this->x;
  *y = this->y;
}

void vgui_soview2D_point::translate(float tx, float ty)
{
  x += tx;
  y += ty;
}

//--------------------------------------------------------------------------//

vcl_ostream& vgui_soview2D_lineseg::print(vcl_ostream& s) const
{
  s << "[vgui_soview2D_lineseg " << x0 << "," << y0 << " -- " << x1 << "," << y1;
  s << " "; return vgui_soview2D::print(s) << "]";
}

void vgui_soview2D_lineseg::draw() const
{
  //vcl_cerr << "line id : " << id << vcl_endl;

  //glLineWidth(style->line_width);
  glBegin(GL_LINES);
  glVertex2f(x0,y0);
  glVertex2f(x1,y1);
  glEnd();
}

float vgui_soview2D_lineseg::distance_squared(float x, float y) const
{
  return vgl_distance2_to_linesegment(x0, y0, x1, y1, x, y);
}

void vgui_soview2D_lineseg::get_centroid(float* x, float* y) const
{
  *x = (x0 + x1) / 2;
  *y = (y0 + y1) / 2;
}

void vgui_soview2D_lineseg::translate(float tx, float ty)
{
  x0 += tx;
  y0 += ty;
  x1 += tx;
  y1 += ty;
}

//--------------------------------------------------------------------------//

void vgui_soview2D_group::set_style(vgui_style *s)
{
  for (unsigned int i=0; i< ls.size(); i++)
    if (!ls[i]->get_style())
      ls[i]->set_style(s);

  vgui_soview::set_style( s);
}

vcl_ostream& vgui_soview2D_group::print(vcl_ostream& s) const
{
  s << "[vgui_soview2D_group ";

  for (unsigned int i=0; i< ls.size(); i++)
    ls[i]->print(s);

  return vgui_soview2D::print(s) << "]";
}

void vgui_soview2D_group::draw() const
{
  for (unsigned int i=0; i< ls.size(); i++)
    ls[i]->draw();
}

float vgui_soview2D_group::distance_squared(float x, float y) const
{
  if (ls.size() == 0)
    return -1e30f;

  float min= ls[0]->distance_squared( x, y);

  for (unsigned int i=1; i< ls.size(); i++)
  {
    float d= ls[i]->distance_squared( x, y);
    if ( d< min ) min= d;
  }

  return min;
}

void vgui_soview2D_group::get_centroid(float* x, float* y) const
{
  *x = 0;
  *y = 0;
  int n = ls.size();

  for (int i=0; i < n; i++)
  {
    float cx, cy;
    ls[i]->get_centroid(&cx, &cy);
    *x += cx;
    *y += cy;
  }

  float s = 1.0f/n;
  *x *= s;
  *y *= s;
}

void vgui_soview2D_group::translate(float tx, float ty)
{
  for (unsigned int i=0; i < ls.size(); i++)
    ls[i]->translate(tx, ty);
}

//--------------------------------------------------------------------------//

vcl_ostream& vgui_soview2D_infinite_line::print(vcl_ostream& s) const
{
  s << "[vgui_soview2D_infinite_line " << a << "," << b << "," << c;
  s << " "; return vgui_soview2D::print(s) << "]";
}

void vgui_soview2D_infinite_line::draw() const
{
  vgui_draw_line(a, b, c);
}


float vgui_soview2D_infinite_line::distance_squared(float x, float y) const
{
  float tmp = a*x + b*y + c;
  return tmp*tmp/(a*a + b*b);
}

void vgui_soview2D_infinite_line::get_centroid(float* x, float* y) const
{
  *x = 0;
  *y = 0;
}

void vgui_soview2D_infinite_line::translate(float tx, float ty)
{
  c += a * tx + b * ty;
}

//--------------------------------------------------------------------------//

const int vgui__CIRCLE2D_LIST = 1;

void vgui_soview2D_circle::compile()
{
  glNewList(vgui__CIRCLE2D_LIST, GL_COMPILE);
  glBegin(GL_LINE_LOOP);
  for (unsigned int i=0;i<100;i++)
  {
    double angle = i*(2*vnl_math::pi/100);
    glVertex2d(vcl_cos(angle), vcl_sin(angle));
  }
  glEnd();
  glEndList();
}


vcl_ostream& vgui_soview2D_circle::print(vcl_ostream& s) const
{
  s << "[vgui_soview2D_circle " << x << "," << y << " r" << r;
  s << " "; return vgui_soview2D::print(s) << "]";
}

void vgui_soview2D_circle::draw() const
{
  glBegin(GL_LINE_LOOP);
  for (unsigned int i=0;i<100;i++)
  {
    double angle = i*(2*vnl_math::pi/100);
    glVertex2d(x+r*vcl_cos(angle), y+r*vcl_sin(angle));
  }
  glEnd();
}

float vgui_soview2D_circle::distance_squared(float x, float y) const
{
  float dx = this->x - x;
  float dy = this->y - y;

  // distance from point to centre
  float dcentre = vcl_sqrt(dx*dx + dy*dy);

  // signed distance from point to circumference
  float dcircum = dcentre - this->r;

  return dcircum * dcircum;
}

void vgui_soview2D_circle::get_centroid(float* x, float* y) const
{
  *x = this->x;
  *y = this->y;
}

void vgui_soview2D_circle::translate(float tx, float ty)
{
  x += tx;
  y += ty;
}

//--------------------------------------------------------------------------------//

vcl_ostream& vgui_soview2D_ellipse::print(vcl_ostream& s) const
{
  s << "[vgui_soview2D_ellipse " << x << "," << y;
  s << " w" << w << " h" << h << " phi" << phi;
  s << "  "; return vgui_soview2D::print(s) << "]";
}

void vgui_soview2D_ellipse::draw() const
{
  double px, py;

  glBegin(GL_LINE_LOOP);
  for (unsigned int i=0;i<100;i++)
  {
    double angle = i*(2*vnl_math::pi/100);
    px = w*vcl_cos(this->phi)*vcl_cos(angle) + h*vcl_sin(this->phi)*vcl_sin(angle);
    py = h*vcl_cos(this->phi)*vcl_sin(angle) - w*vcl_sin(this->phi)*vcl_cos(angle);
    glVertex2d(x+px, y+py);
  }
  glEnd();
}

float vgui_soview2D_ellipse::distance_squared(float x, float y) const
{
  return (x - this->x)*(x - this->x) + (y - this->y)*(y - this->y);
  // not implemented - TODO
}

void vgui_soview2D_ellipse::get_centroid(float* x, float* y) const
{
  *x = this->x;
  *y = this->y;
}

void vgui_soview2D_ellipse::translate(float tx, float ty)
{
  x += tx;
  y += ty;
}


//--------------------------------------------------------------------------------//

vgui_soview2D_linestrip::vgui_soview2D_linestrip(unsigned n_, float const *x_, float const *y_)
  : n(n_), x(new float[n]), y(new float[n])
{
  for (unsigned i=0; i<n; ++i)
  {
    x[i] = x_[i];
    y[i] = y_[i];
  }
}

vgui_soview2D_linestrip::~vgui_soview2D_linestrip()
{
  n=0;
  delete [] x; x=0;
  delete [] y; y=0;
}

void vgui_soview2D_linestrip::draw() const
{
  glBegin(GL_LINE_STRIP);
  for (unsigned i=0; i<n; ++i)
    glVertex2f(x[i], y[i]);
  glEnd();
}

vcl_ostream& vgui_soview2D_linestrip::print(vcl_ostream&s) const { return s << "[a linestrip. FIXME]"; }

float vgui_soview2D_linestrip::distance_squared(float x, float y) const
{
  double tmp = vgl_distance_to_non_closed_polygon(this->x, this->y, this->n, x, y);
  return tmp*tmp;
}

void vgui_soview2D_linestrip::get_centroid(float* x, float* y) const
{
  *x = 0;
  *y = 0;
  for (unsigned i=0; i<n; ++i)
  {
    *x += this->x[i];
    *y += this->y[i];
  }
  float s = 1.0f / float(n);
  *x *= s;
  *y *= s;
}

void vgui_soview2D_linestrip::translate(float tx, float ty)
{
  for (unsigned i=0; i<n; ++i)
  {
    x[i] += tx;
    y[i] += ty;
  }
}

void vgui_soview2D_linestrip::set_size(unsigned nn)
{
  if (nn < n) { n = nn; return; }

  // we know that n <= nn
  float *nx = new float[nn];
  float *ny = new float[nn];
  for (unsigned i=0; i<n; ++i)
  {
    nx[i] = x[i];
    ny[i] = y[i];
  }

  n = nn;
  delete [] x; x = nx;
  delete [] y; y = ny;
}

//--------------------------------------------------------------------------------//

vgui_soview2D_polygon::vgui_soview2D_polygon(unsigned n_, float const *x_, float const *y_)
  : n(n_), x(new float[n]), y(new float[n])
{
  for (unsigned i=0; i<n; ++i)
  {
    x[i] = x_[i];
    y[i] = y_[i];
  }
}

vgui_soview2D_polygon::~vgui_soview2D_polygon()
{
  n=0;
  delete [] x; x=0;
  delete [] y; y=0;
}

void vgui_soview2D_polygon::draw() const
{
  glBegin(GL_LINE_LOOP);
  for (unsigned i=0; i<n; ++i)
    glVertex2f(x[i], y[i]);
  glEnd();
}

vcl_ostream& vgui_soview2D_polygon::print(vcl_ostream&s) const { return s << "[a polygon. FIXME]"; }

float vgui_soview2D_polygon::distance_squared(float x, float y) const
{
  double tmp = vgl_distance_to_closed_polygon(this->x, this->y, this->n, x, y);
  return tmp*tmp;
}

void vgui_soview2D_polygon::get_centroid(float* x, float* y) const
{
  *x = 0;
  *y = 0;
  for (unsigned i=0; i<n; ++i)
  {
    *x += this->x[i];
    *y += this->y[i];
  }
  float s = 1.0f / float(n);
  *x *= s;
  *y *= s;
}

void vgui_soview2D_polygon::translate(float tx, float ty)
{
  for (unsigned i=0; i<n; ++i)
  {
    x[i] += tx;
    y[i] += ty;
  }
}

void vgui_soview2D_polygon::set_size(unsigned nn)
{
  if (nn < n) { n = nn; return; }

  // we know that n <= nn
  float *nx = new float[nn];
  float *ny = new float[nn];
  for (unsigned i=0; i<n; ++i)
  {
    nx[i] = x[i];
    ny[i] = y[i];
  }

  n = nn;
  delete [] x; x = nx;
  delete [] y; y = ny;
}


//-----------------------------------------------------------
void vgui_soview2D_image::set_image(float x, float y, float w, float h, char *data)
{
  x_ = x;
  y_ = y;
  width_ = w;
  height_ = h;

  img_ = data; 

  //defaults
  img_format_ = GL_RGB;
  img_type_ = GL_UNSIGNED_BYTE;

  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
}

vgui_soview2D_image::~vgui_soview2D_image()
{
}

void vgui_soview2D_image::draw() const
{
  glEnable(GL_BLEND);
  glRasterPos2i(x_,y_);
  glDrawPixels(width_,height_,img_format_,img_type_,img_);
  glFlush();
#if 0
  vil1_memory_image_of< vil1_rgb< unsigned char > > test;
  test.resize(width_,height_);
  test.put_section(img_,0,0,width_,height_);
  vil1_save(test,"shouldwork.jpg","jpeg");
#endif // 0
}

vcl_ostream& vgui_soview2D_image::print(vcl_ostream&s) const { return s << "[a image. FIXME]"; }

float vgui_soview2D_image::distance_squared(float x, float y) const
{
  float dx = (x_ + (width_ / 2)) - x;
  float dy = (y_ + (height_ / 2)) - y;
  return dx*dx + dy*dy;
}

void vgui_soview2D_image::get_centroid(float* x, float* y) const
{
  float x1 = x_ + (width_ / 2);
  float y1 = y_ + (height_ / 2);

  *x = x1;
  *y = y1;
}

void vgui_soview2D_image::translate(float tx, float ty)
{
    x_ += tx;
    y_ += ty;
}
