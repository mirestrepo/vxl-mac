#include "bgui_vsol_camera_tableau.h"
//:
// \file

#include <bgui/bgui_vsol_soview2D.h>
//#include <vgui/vgui.h>
#include <vgui/vgui_style.h>
#include <bgui/bgui_style.h>
#include <vsol/vsol_point_3d.h>
#include <vsol/vsol_point_2d.h>
#include <vsol/vsol_line_3d.h>
#include <vsol/vsol_line_2d.h>
#include <vsol/vsol_polygon_3d.h>
#include <vsol/vsol_polygon_2d.h>
#include <vsol/vsol_box_3d.h>

bgui_vsol_camera_tableau::bgui_vsol_camera_tableau(const char* n) : vgui_easy2D_tableau(n)
{ this->init(); }

bgui_vsol_camera_tableau::bgui_vsol_camera_tableau(vgui_image_tableau_sptr const& it,
                                         const char* n) :
  vgui_easy2D_tableau(it, n) { this->init(); }

bgui_vsol_camera_tableau::bgui_vsol_camera_tableau(vgui_tableau_sptr const& t,
                                         const char* n) :
  vgui_easy2D_tableau(t, n) { this->init(); }

bgui_vsol_camera_tableau::~bgui_vsol_camera_tableau()
{
}

vgl_point_2d<double> 
bgui_vsol_camera_tableau::project(vsol_point_3d_sptr const& p3d)
{
  vgl_homg_point_3d<double> hp3d = p3d->homg_point();
  vgl_homg_point_2d<double> hp2d = camera_(hp3d);
  vgl_point_2d<double> p2d(hp2d);
  return p2d;
}
void bgui_vsol_camera_tableau::init()
{
  old_id_ = 0;
  highlight_ = true;
  //set highlight display style parameters
  highlight_style_ = new bgui_style(0.0f, 0.0f, 1.0f, 5.0f, 5.0f);

  //define default soview styles
  //these can be overridden by later set_*_syle commands prior to drawing.
  //
   this->set_vsol_point_3d_style(0.0f, 1.0f, 0.0f, 5.0f);
   this->set_vsol_line_3d_style(0.8f, 0.2f, 0.9f, 3.0f);
   this->set_vsol_polygon_3d_style(0.0f, 1.0f, 0.0f, 3.0f);
   this->set_vsol_box_3d_style(0.8f, 0.2f, 0.9f, 3.0f);
   camera_.set_identity();
}
//:
// Provide roaming highlighting for soviews in the tableau.
// As the mouse moves the soview closest to the mouse is
// changed to the highlighted style.
// vgui_displaylist2D_tableau::motion(..) has a mechanism for
// highlighting the nearest object but it doesn't work.

bool bgui_vsol_camera_tableau::handle(vgui_event const &e)
{
  if (e.type == vgui_MOTION&&highlight_)
    {
      //retrive the previously highlighted soview and
      //restore it to its default style
      vgui_soview* old_so = vgui_soview::id_to_object(old_id_);
      if (old_so)
        {
          bgui_style_sptr default_sty = style_map_[old_so->type_name()];
          bgui_style* bs = (bgui_style*)default_sty.ptr();
          vgui_style* s = (vgui_style*)bs;
          old_so->set_style(s);
        }
      //get the soview that is closest to the mouse
      vgui_soview2D* high_so = (vgui_soview2D*)get_highlighted_soview();
      if (high_so&&high_so->get_style())
        {
          //replace the old soview with the currently closest view
          int id = high_so->get_id();
          old_id_ = id;
          //set soview style to the highlight color and weight
          bgui_style* bsh = (bgui_style*)highlight_style_.ptr();
          vgui_style* sh = (vgui_style*)bsh;
          high_so->set_style(sh);
          this->post_redraw();
        }
    }
  // We aren't interested in other events so pass them to the base class.
  return vgui_easy2D_tableau::handle(e);
}

bgui_vsol_soview2D_point*
bgui_vsol_camera_tableau::add_vsol_point_3d(vsol_point_3d_sptr const& p)
{
  bgui_vsol_soview2D_point* obj =
    new bgui_vsol_soview2D_point();
  vgl_homg_point_3d<double> hp3d = p->homg_point();
  vgl_homg_point_2d<double> hp2d = camera_(hp3d);
  vgl_point_2d<double> p2d(hp2d);//Get 
  obj->x = p2d.x();
  obj->y = p2d.y();
  add(obj);
   bgui_style_sptr s = style_map_[obj->type_name()];
   obj->set_style(s.ptr());
  return obj;
}

bgui_vsol_soview2D_line_seg*
bgui_vsol_camera_tableau::add_vsol_line_3d(vsol_line_3d_sptr const& line)
{
  vsol_point_3d_sptr p0 = line->p0();
  vsol_point_3d_sptr p1 = line->p1();
  vgl_point_2d<double> p0_2d = this->project(p0);
  vgl_point_2d<double> p1_2d = this->project(p1);

  vsol_line_2d_sptr l2d = new vsol_line_2d(p0_2d, p1_2d);
  bgui_vsol_soview2D_line_seg* obj =
    new bgui_vsol_soview2D_line_seg(l2d);
  add(obj);
  //set the default style
  bgui_style_sptr sty = style_map_[obj->type_name()];
  obj->set_style(sty.ptr());
  return obj;
}

bgui_vsol_soview2D_polygon*
bgui_vsol_camera_tableau::add_vsol_polygon_3d(vsol_polygon_3d_sptr const& poly)
{
  int n = poly->size();
  vcl_vector<vsol_point_2d_sptr> vertices(n);
  //project the polygon
  for(int i = 0; i<n; i++)
    {
      vsol_point_3d_sptr v = poly->vertex(i);
      vgl_point_2d<double> p2d = this->project(v);
      vsol_point_2d_sptr p = new vsol_point_2d(p2d);
      vertices[i] = p;
    }
  vsol_polygon_2d_sptr poly_2d = new vsol_polygon_2d(vertices);
  bgui_vsol_soview2D_polygon* obj =
    new bgui_vsol_soview2D_polygon(poly_2d);
  //set the default style
  bgui_style_sptr sty = style_map_[obj->type_name()];
  add(obj);
  if (obj)
    obj->set_style(sty.ptr());
  return obj;
}

bgui_vsol_soview2D_polygon_group*
bgui_vsol_camera_tableau::add_vsol_box_3d(vsol_box_3d_sptr const& box)
{
  // top face points
  vsol_point_3d_sptr pt0 =  
    new vsol_point_3d(box->get_min_x(), box->get_min_y(), box->get_max_z());
  vgl_point_2d<double> vpt0_2d = this->project(pt0);
  vsol_point_2d_sptr pt0_2d = new vsol_point_2d(vpt0_2d);

  vsol_point_3d_sptr pt1 =  
    new vsol_point_3d(box->get_max_x(), box->get_min_y(), box->get_max_z());
  vgl_point_2d<double> vpt1_2d = this->project(pt1);
  vsol_point_2d_sptr pt1_2d = new vsol_point_2d(vpt1_2d);

  vsol_point_3d_sptr pt2 =  
    new vsol_point_3d(box->get_max_x(), box->get_max_y(), box->get_max_z());
  vgl_point_2d<double> vpt2_2d = this->project(pt2);
  vsol_point_2d_sptr pt2_2d = new vsol_point_2d(vpt2_2d);

  vsol_point_3d_sptr pt3 =  
    new vsol_point_3d(box->get_min_x(), box->get_max_y(), box->get_max_z());
  vgl_point_2d<double> vpt3_2d = this->project(pt3);
  vsol_point_2d_sptr pt3_2d = new vsol_point_2d(vpt3_2d);

  // bottom face points
  vsol_point_3d_sptr pb0 =  
    new vsol_point_3d(box->get_min_x(), box->get_min_y(), box->get_min_z());
  vgl_point_2d<double> vpb0_2d = this->project(pb0);
  vsol_point_2d_sptr pb0_2d = new vsol_point_2d(vpb0_2d);

  vsol_point_3d_sptr pb1 =  
    new vsol_point_3d(box->get_max_x(), box->get_min_y(), box->get_min_z());
  vgl_point_2d<double> vpb1_2d = this->project(pb1);
  vsol_point_2d_sptr pb1_2d = new vsol_point_2d(vpb1_2d);

  vsol_point_3d_sptr pb2 =  
    new vsol_point_3d(box->get_max_x(), box->get_max_y(), box->get_min_z());
  vgl_point_2d<double> vpb2_2d = this->project(pb2);
  vsol_point_2d_sptr pb2_2d = new vsol_point_2d(vpb2_2d);

  vsol_point_3d_sptr pb3 =  
    new vsol_point_3d(box->get_min_x(), box->get_max_y(), box->get_min_z());
  vgl_point_2d<double> vpb3_2d = this->project(pb3);
  vsol_point_2d_sptr pb3_2d = new vsol_point_2d(vpb3_2d);

  //construct the 6 polygons that make up the box
  //Top face
  vcl_vector<vsol_point_2d_sptr> top_verts;
  top_verts.push_back(pt0_2d);  top_verts.push_back(pt1_2d);
  top_verts.push_back(pt2_2d);  top_verts.push_back(pt3_2d);
  vsol_polygon_2d_sptr top_poly = new vsol_polygon_2d(top_verts);
  //Bottom face
  vcl_vector<vsol_point_2d_sptr> bottom_verts;
  bottom_verts.push_back(pb0_2d);  bottom_verts.push_back(pb1_2d);
  bottom_verts.push_back(pb2_2d);  bottom_verts.push_back(pb3_2d);
  vsol_polygon_2d_sptr bottom_poly = new vsol_polygon_2d(bottom_verts);
  //Side face 0
  vcl_vector<vsol_point_2d_sptr> side0_verts;
  side0_verts.push_back(pb0_2d);  side0_verts.push_back(pt0_2d);
  side0_verts.push_back(pt1_2d);  side0_verts.push_back(pb1_2d);
  vsol_polygon_2d_sptr side0_poly = new vsol_polygon_2d(side0_verts);
  //Side face 1
  vcl_vector<vsol_point_2d_sptr> side1_verts;
  side1_verts.push_back(pb1_2d);  side1_verts.push_back(pt1_2d);
  side1_verts.push_back(pt2_2d);  side1_verts.push_back(pb2_2d);
  vsol_polygon_2d_sptr side1_poly = new vsol_polygon_2d(side1_verts);
  //Side face 2
  vcl_vector<vsol_point_2d_sptr> side2_verts;
  side2_verts.push_back(pb2_2d);  side2_verts.push_back(pt2_2d);
  side2_verts.push_back(pt3_2d);  side2_verts.push_back(pb3_2d);
  vsol_polygon_2d_sptr side2_poly = new vsol_polygon_2d(side2_verts);
  //Side face 3
  vcl_vector<vsol_point_2d_sptr> side3_verts;
  side3_verts.push_back(pb3_2d);  side3_verts.push_back(pt3_2d);
  side3_verts.push_back(pt0_2d);  side3_verts.push_back(pb0_2d);
  vsol_polygon_2d_sptr side3_poly = new vsol_polygon_2d(side3_verts);
  //Construct the poly group
  vcl_vector<vsol_polygon_2d_sptr> polys;
  polys.push_back(top_poly);   polys.push_back(bottom_poly);
  polys.push_back(side0_poly);   polys.push_back(side1_poly);
  polys.push_back(side2_poly);   polys.push_back(side3_poly);

  bgui_vsol_soview2D_polygon_group* obj =
    new bgui_vsol_soview2D_polygon_group(polys);
  //set the default style
  bgui_style_sptr sty = style_map_[obj->type_name()];
  add(obj);
  obj->set_style(sty.ptr());
  return obj;
}

//--------------------------------------------------------------
// Add general spatial objects
//
void bgui_vsol_camera_tableau::
add_spatial_objects_3d(vcl_vector<vsol_spatial_object_3d_sptr> const& sos)
{
  for (vcl_vector<vsol_spatial_object_3d_sptr>::const_iterator sit = sos.begin();
       sit != sos.end(); sit++)
    {
      add_spatial_object_3d( (*sit) );
    }
}

void bgui_vsol_camera_tableau::
add_spatial_object_3d(vsol_spatial_object_3d_sptr const& so)
{
  if (so->cast_to_point()) {
    {
      vsol_point_3d_sptr p = so->cast_to_point();
      this->add_vsol_point_3d(p);
    }
  } 
  if (so->cast_to_curve()) {
    if (so->cast_to_curve()->cast_to_line())
      {
        vsol_line_3d_sptr line =
          so->cast_to_curve()->cast_to_line();
        this->add_vsol_line_3d(line);
      }
  }

  if (so->cast_to_region()) {
    if (so->cast_to_region()->cast_to_polygon())
      {
        vsol_polygon_3d_sptr poly =
          so->cast_to_region()->cast_to_polygon();
        this->add_vsol_polygon_3d(poly);
      }
  }

  return;
}

void bgui_vsol_camera_tableau::
add_points_3d(vcl_vector<vsol_point_3d_sptr> const& points)
{
  for (vcl_vector<vsol_point_3d_sptr>::const_iterator pit = points.begin();
       pit != points.end(); pit++)
    add_vsol_point_3d(*pit);
}

void bgui_vsol_camera_tableau::
add_lines_3d(vcl_vector<vsol_line_3d_sptr> const& lines)
{
  for (vcl_vector<vsol_line_3d_sptr>::const_iterator lit = lines.begin();
       lit != lines.end(); lit++)
    add_vsol_line_3d(*lit);
}

void bgui_vsol_camera_tableau::
add_polygons_3d(vcl_vector<vsol_polygon_3d_sptr> const& polys)
{
  for (vcl_vector<vsol_polygon_3d_sptr>::const_iterator pit = polys.begin();
       pit != polys.end(); pit++)
    add_vsol_polygon_3d(*pit);
}

void bgui_vsol_camera_tableau::clear_all()
{
  bool temp = highlight_;
  highlight_ = false;//in case of event interrupts during the clear
  obj_map_.clear();
  vgui_easy2D_tableau::clear();
  old_id_ = 0;
  highlight_ = temp;
  this->post_redraw();
}

void 
bgui_vsol_camera_tableau::set_vsol_point_3d_style(const float r,
                                                  const float g,
                                                  const float b,
                                                  const float point_radius)
{
  bgui_style_sptr sty = new bgui_style(r, g, b, point_radius, 0.0f);
  bgui_vsol_soview2D_point p;
  style_map_[p.type_name()]=sty;
}

void 
bgui_vsol_camera_tableau::set_vsol_line_3d_style(const float r,
                                                 const float g,
                                                 const float b,
                                                 const float line_width)
{
  bgui_style_sptr sty = new bgui_style(r, g, b, 0.0f, line_width);
  bgui_vsol_soview2D_line_seg seg;
  style_map_[seg.type_name()]=sty;
}


void 
bgui_vsol_camera_tableau::set_vsol_polygon_3d_style(const float r,
                                                    const float g,
                                                    const float b,
                                                    const float line_width)
{
  bgui_style_sptr sty = new bgui_style(r, g, b, 0.0f, line_width);
  bgui_vsol_soview2D_polygon poly;
  style_map_[poly.type_name()]=sty;
}

void bgui_vsol_camera_tableau::set_vsol_box_3d_style(const float r, 
                                                const float g,
                                                const float b,
                                                const float line_width)
{
  bgui_style_sptr sty = new bgui_style(r, g, b, 0.0f, line_width);
  bgui_vsol_soview2D_polygon_group pg;
  style_map_[pg.type_name()]=sty;
}

