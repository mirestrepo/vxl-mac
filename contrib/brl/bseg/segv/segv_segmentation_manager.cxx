// This is brl/bseg/segv/segv_segmentation_manager.cxx
#include "segv_segmentation_manager.h"
//:
// \file
// \author J.L. Mundy

#include <vcl_cstdlib.h> // for vcl_exit()
#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vbl/vbl_array_2d.h>
#include <vil1/vil1_image.h>
#include <vil1/vil1_memory_image_of.h>
#include <vil1/vil1_load.h>
#include <vil1/vil1_crop.h>
#include <vdgl/vdgl_digital_curve.h>
#include <vdgl/vdgl_digital_curve_sptr.h>
#include <sdet/sdet_detector_params.h>
#include <sdet/sdet_detector.h>
#include <sdet/sdet_harris_detector_params.h>
#include <sdet/sdet_harris_detector.h>
#include <sdet/sdet_fit_lines_params.h>
#include <sdet/sdet_fit_lines.h>
#include <sdet/sdet_grid_finder_params.h>
#include <sdet/sdet_grid_finder.h>
#include <vgui/vgui.h>
#include <vgui/vgui_find.h>
#include <vgui/vgui_tableau.h>
#include <vgui/vgui_dialog.h>
#include <vgui/vgui_style_sptr.h>
#include <vgui/vgui_style.h>
#include <vgui/vgui_viewer2D_tableau.h>
#include <vgui/vgui_shell_tableau.h>
#include <vgui/vgui_grid_tableau.h>
#include <vgui/vgui_rubberband_tableau.h>
#include <vgui/vgui_range_map_params.h>
#include <bgui/bgui_image_tableau.h>
#include <bgui/bgui_vtol2D_tableau.h>
#include <bgui/bgui_vtol2D_rubberband_client.h>
#include <bgui/bgui_picker_tableau.h>
#include <vsol/vsol_point_2d.h>
#include <vsol/vsol_point_2d_sptr.h>
#include <vsol/vsol_curve_2d.h>
#include <vsol/vsol_curve_2d_sptr.h>
#include <vsol/vsol_polygon_2d_sptr.h>
#include <vsol/vsol_polygon_2d.h>
#include <vtol/vtol_vertex_2d.h>
#include <vtol/vtol_vertex.h>
#include <vtol/vtol_edge_2d.h>
#include <vtol/vtol_one_chain_sptr.h>
#include <vtol/vtol_one_chain.h>
#include <vtol/vtol_intensity_face.h>
#include <bbas/bsol/bsol_algs.h>
#include <brip/brip_vil1_float_ops.h>
#include <brip/brip_para_cvrg_params.h>
#include <brip/brip_para_cvrg.h>
#include <brip/brip_watershed_params.h>
#include <sdet/sdet_watershed_region_proc_params.h>
#include <sdet/sdet_watershed_region_proc.h>
#include <sdet/sdet_vehicle_finder_params.h>
#include <sdet/sdet_vehicle_finder.h>
#include <sdet/sdet_region_proc_params.h>
#include <sdet/sdet_region_proc.h>
#include <strk/strk_region_info_params.h>
#include <strk/strk_region_info.h>
#include <strk/strk_io.h>

segv_segmentation_manager *segv_segmentation_manager::instance_ = 0;

segv_segmentation_manager *segv_segmentation_manager::instance()
{
  if (!instance_)
  {
    instance_ = new segv_segmentation_manager();
    instance_->init();
  }
  return segv_segmentation_manager::instance_;
}

//-----------------------------------------------------------
// constructors/destructor
//
segv_segmentation_manager::segv_segmentation_manager():vgui_wrapper_tableau()
{
  first_ = true;
}

segv_segmentation_manager::~segv_segmentation_manager()
{
}

//: Set up the tableaux
void segv_segmentation_manager::init()
{
  bgui_image_tableau_sptr itab = bgui_image_tableau_new();
  bgui_vtol2D_tableau_sptr t2D = bgui_vtol2D_tableau_new(itab);
  bgui_vtol2D_rubberband_client* rcl =  new bgui_vtol2D_rubberband_client(t2D);
  vgui_rubberband_tableau_sptr rubber = vgui_rubberband_tableau_new(rcl);
  vgui_composite_tableau_new comp(t2D,rubber);
  bgui_picker_tableau_sptr picktab = bgui_picker_tableau_new(comp);
  vgui_viewer2D_tableau_sptr v2D = vgui_viewer2D_tableau_new(picktab);
  grid_ = vgui_grid_tableau_new(1,1);
  grid_->set_grid_size_changeable(true);
  grid_->add_at(v2D, 0, 0);
  vgui_shell_tableau_sptr shell = vgui_shell_tableau_new(grid_);
  this->add_child(shell);
  first_ = true;
}

//: set the image at the currently selected grid cell
void segv_segmentation_manager::set_selected_grid_image(vil1_image& image)
{
  bgui_image_tableau_sptr itab = this->selected_image_tab();
  if (!itab)
    this->add_image(image);
  else
    itab->set_image(image);
  itab->post_redraw();
}
//: Add an image at the specified grid cell
void segv_segmentation_manager::
add_image_at(vil1_image& image, const unsigned col, const unsigned row)
{
  bgui_image_tableau_sptr itab = bgui_image_tableau_new(image);
  bgui_vtol2D_tableau_sptr t2D = bgui_vtol2D_tableau_new(itab);
  bgui_vtol2D_rubberband_client* rcl =  new bgui_vtol2D_rubberband_client(t2D);
  vgui_rubberband_tableau_sptr rubber = vgui_rubberband_tableau_new(rcl);
  vgui_composite_tableau_new comp(t2D,rubber);
  bgui_picker_tableau_sptr picktab = bgui_picker_tableau_new(comp);
  vgui_viewer2D_tableau_sptr v2D = vgui_viewer2D_tableau_new(picktab);
  grid_->add_at(v2D, col, row);
  itab->post_redraw();
}

//: Add an image to the currently selected grid cell
void segv_segmentation_manager::add_image(vil1_image& image)
{
  unsigned row=0, col=0;
  grid_->get_last_selected_position(&col, &row);
  this->add_image_at(image, col, row);
}

//: Get the image tableau for the currently selected grid cell
bgui_image_tableau_sptr segv_segmentation_manager::selected_image_tab()
{
  unsigned row=0, col=0;
  grid_->get_last_selected_position(&col, &row);
  vgui_tableau_sptr top_tab = grid_->get_tableau_at(col, row);
  if (top_tab)
  {
    bgui_image_tableau_sptr itab;
    itab.vertical_cast(vgui_find_below_by_type_name(top_tab,
                                                    vcl_string("vgui_image_tableau")));
    if (itab)
      return itab;
  }
  vcl_cout << "Unable to get bgui_image_tableau at (" << col
           << ", " << row << ")\n";
  return bgui_image_tableau_sptr();
}

//: Get the vtol2D tableau for the currently selected grid cell
bgui_vtol2D_tableau_sptr segv_segmentation_manager::selected_vtol2D_tab()
{
  unsigned row=0, col=0;
  grid_->get_last_selected_position(&col, &row);
  vgui_tableau_sptr top_tab = grid_->get_tableau_at(col, row);
  if (top_tab)
  {
    bgui_vtol2D_tableau_sptr v2D;
    v2D.vertical_cast(vgui_find_below_by_type_name(top_tab,
                                                   vcl_string("bgui_vtol2D_tableau")));
    if (v2D)
      return v2D;
  }
  vcl_cout << "Unable to get bgui_vtol2D_tableau at (" << col
           << ", " << row << ")\n";
  return bgui_vtol2D_tableau_sptr();
}

//: Get the picker tableau for the currently selected grid cell
bgui_picker_tableau_sptr segv_segmentation_manager::selected_picker_tab()
{
  unsigned row=0, col=0;
  grid_->get_last_selected_position(&col, &row);
  vgui_tableau_sptr top_tab = grid_->get_tableau_at(col, row);
  if (top_tab)
  {
    bgui_picker_tableau_sptr pick;
    pick.vertical_cast(vgui_find_below_by_type_name(top_tab,
                                                    vcl_string("bgui_picker_tableau")));
    if (pick)
      return pick;
  }
  vcl_cout << "Unable to get bgui_picker_tableau at (" << col
           << ", " << row << ")\n";
  return bgui_picker_tableau_sptr();
}

//: Get the rubberband tableau at the selected grid cell
vgui_rubberband_tableau_sptr segv_segmentation_manager::selected_rubber_tab()
{
  unsigned row=0, col=0;
  grid_->get_last_selected_position(&col, &row);
  vgui_tableau_sptr top_tab = grid_->get_tableau_at(col, row);
  if (top_tab)
  {
    vgui_rubberband_tableau_sptr rubber;
    rubber.vertical_cast(vgui_find_below_by_type_name(top_tab,
                                                      vcl_string("vgui_rubberband_tableau")));
    if (rubber)
      return rubber;
  }
  vcl_cout << "Unable to get vgui_rubberband_tableau at (" << col
           << ", " << row << ")\n";
  return vgui_rubberband_tableau_sptr();
}

vil1_image segv_segmentation_manager::selected_image()
{
  bgui_image_tableau_sptr itab = this->selected_image_tab();
  if (!itab)
    return vil1_image();
  return itab->get_image();
}

vil1_image segv_segmentation_manager::image_at(const unsigned col,
                                               const unsigned row)
{
  vgui_tableau_sptr top_tab = grid_->get_tableau_at(col, row);
  if (!top_tab)
    return vil1_image();

  bgui_image_tableau_sptr itab;
  itab.vertical_cast(vgui_find_below_by_type_name(top_tab,
                                                  vcl_string("vgui_image_tableau")));
  if (!itab)
  {
    vcl_cout << "Unable to get bgui_image_tableau at (" << col
             << ", " << row << ")\n";
    return vil1_image();
  }
  return itab->get_image();
}

void segv_segmentation_manager::quit()
{
  vcl_exit(1);
}

void segv_segmentation_manager::load_image()
{
  static bool greyscale = true;
  vgui_dialog load_image_dlg("Load image file");
  static vcl_string image_filename = "/home/dec/images/cal_image1.tif";
  static vcl_string ext = "*.*";
  load_image_dlg.file("Image Filename:", ext, image_filename);
  load_image_dlg.checkbox("greyscale ", greyscale);
  if (!load_image_dlg.ask())
    return;

  vil1_image temp = vil1_load(image_filename.c_str()), image;

  if (greyscale)
    image = brip_vil1_float_ops::convert_to_grey(temp);
  else
    image = temp;

  if (first_)
  {
    this->set_selected_grid_image(image);
    first_ = false;
  }
  else
    this->add_image(image);
}

//-----------------------------------------------------------------------------
//: Clear the display
//-----------------------------------------------------------------------------
void segv_segmentation_manager::clear_display()
{
  bgui_vtol2D_tableau_sptr t2D = this->selected_vtol2D_tab();
  if (!t2D)
    return;
  t2D->clear_all();
}

//-----------------------------------------------------------------------------
//: Draw edges onto the tableau
//-----------------------------------------------------------------------------
void
segv_segmentation_manager::draw_edges(vcl_vector<vtol_edge_2d_sptr>& edges,
                                      bool verts)
{
  bgui_vtol2D_tableau_sptr t2D = this->selected_vtol2D_tab();
  if (!t2D)
    return;
  this->clear_display();
#if 0
  vgui_image_tableau_sptr itab = t2D->get_image_tableau();
  if (!itab)
  {
    vcl_cout << "In segv_segmentation_manager::draw_edges - null image tab\n";
    return;
  }
#endif
  for (vcl_vector<vtol_edge_2d_sptr>::iterator eit = edges.begin();
       eit != edges.end(); eit++)
  {
    t2D->add_edge(*eit);
    //optionally display the edge vertices
    if (verts)
    {
      if ((*eit)->v1())
      {
        vtol_vertex_2d_sptr v1 = (*eit)->v1()->cast_to_vertex_2d();
        t2D->add_vertex(v1);
      }
      if ((*eit)->v2())
      {
        vtol_vertex_2d_sptr v2 = (*eit)->v2()->cast_to_vertex_2d();
        t2D->add_vertex(v2);
      }
    }
  }
  t2D->post_redraw();
}


//-----------------------------------------------------------------------------
//: Draw polylines on the tableau
//-----------------------------------------------------------------------------
void segv_segmentation_manager::
draw_polylines(vcl_vector<vsol_polyline_2d_sptr > const& polys)
{
  bgui_vtol2D_tableau_sptr t2D = this->selected_vtol2D_tab();
  if (!t2D)
    return;
  //this->clear_display();
  vgui_image_tableau_sptr itab = t2D->get_image_tableau();
  if (!itab)
  {
    vcl_cout << "In segv_segmentation_manager::draw_edges - null image tab\n";
    return;
  }
  for (vcl_vector<vsol_polyline_2d_sptr>::const_iterator pit = polys.begin();
       pit != polys.end(); pit++)
    {
      t2D->add_vsol_polyline_2d(*pit);
    }

  t2D->post_redraw();
}

//-----------------------------------------------------------------------------
//: Draw line segments on the tableau
//-----------------------------------------------------------------------------
void segv_segmentation_manager::
draw_lines(vcl_vector<vsol_line_2d_sptr > const& lines,
           const vgui_style_sptr& style)
{
  bgui_vtol2D_tableau_sptr t2D = this->selected_vtol2D_tab();
  if (!t2D)
    return;
  //this->clear_display();
#if 0
  vgui_image_tableau_sptr itab = t2D->get_image_tableau();
  if (!itab)
  {
    vcl_cout << "In segv_segmentation_manager::draw_edges - null image tab\n";
    return;
  }
#endif
  for (vcl_vector<vsol_line_2d_sptr>::const_iterator lit = lines.begin();
       lit != lines.end(); lit++)
    {
      t2D->add_vsol_line_2d(*lit,style);
    }

  t2D->post_redraw();
}

//-----------------------------------------------------------------------------
//: Draw points on the tableau
//-----------------------------------------------------------------------------
void segv_segmentation_manager::
draw_points(vcl_vector<vsol_point_2d_sptr> const& points, const vgui_style_sptr& style)
{
  bgui_vtol2D_tableau_sptr t2D = this->selected_vtol2D_tab();
  if (!t2D)
    return;
  //this->clear_display();
  vgui_image_tableau_sptr itab = t2D->get_image_tableau();
  if (!itab)
  {
    vcl_cout << "In segv_segmentation_manager::draw_edges - null image tab\n";
    return;
  }
  for (vcl_vector<vsol_point_2d_sptr>::const_iterator pit = points.begin();
       pit != points.end(); pit++)
    {
      t2D->add_vsol_point_2d(*pit,style);
    }

  t2D->post_redraw();
}

void segv_segmentation_manager::draw_regions(vcl_vector<vtol_intensity_face_sptr>& regions,
                                             bool verts)
{
  bgui_vtol2D_tableau_sptr t2D = this->selected_vtol2D_tab();
  if (!t2D)
    return;
  for (vcl_vector<vtol_intensity_face_sptr>::iterator rit = regions.begin();
       rit != regions.end(); rit++)
    {
      vtol_face_2d_sptr f = (*rit)->cast_to_face_2d();
      t2D->add_face(f);
      if (verts)
      {
        vcl_vector<vtol_vertex_sptr> vts;
        f->vertices(vts);
        for (vcl_vector<vtol_vertex_sptr>::iterator vit = vts.begin();
             vit != vts.end(); vit++)
          {
            vtol_vertex_2d_sptr v = (*vit)->cast_to_vertex_2d();
            t2D->add_vertex(v);
          }
      }
    }
  t2D->post_redraw();
}

void segv_segmentation_manager::original_image()
{
#if 0
  if (img_)
  {
    t2D->get_image_tableau()->set_image(img_);
    t2D->post_redraw();
  }
#endif
}

void segv_segmentation_manager::roi()
{
  vil1_image img = selected_image();
  if (!img)
  {
    vcl_cout << "In segv_segmentation_manager::roi() - no image\n";
    return;
  }
  float x0=0, y0=0, x1=0, y1=0;
  bgui_picker_tableau_sptr picktab = this->selected_picker_tab();
  vcl_cout << "Choose upper left corner of ROI.\n";
  picktab->pick_point(&x0,&y0);
  vcl_cout << "picked (x="<<x0<<", y="<<y0<<")\n"
           << "Choose lower right corner of ROI.\n";
  picktab->pick_point(&x1,&y1);
  vcl_cout << "picked (x="<<x1<<", y="<<y1<<")\n";
  if ( (x1 > x0) && (y1 > y0) )
  {
    int w = int(x1 - x0);
    int h = int(y1 - y0);
    vil1_image cropped = vil1_crop(img,int(x0),int(y0),w,h);
    vcl_cout << "cropped x=" <<x0<<" y=" <<y0<< " w=" <<w<<" h=" <<h<< '\n';
#if 0
    if (cropped)
    {
      img = cropped;
      t2D->get_image_tableau()->set_image(cropped);
      t2D->post_redraw();
      return;
    }
#endif
    if (cropped)
    {
      this->add_image(cropped);
      return;
    }
    vcl_cout << "crop failed.\n";
    return;
  }
  vcl_cout << "invalid ROI\n";
  return;
}

void segv_segmentation_manager::gaussian()
{
  vil1_image img = selected_image();
  if (!img)
  {
    vcl_cout << "In segv_segmentation_manager::gaussian() - no image\n";
    return;
  }
  static float sigma = 1.0;
  vgui_dialog gauss_dialog("Gaussian Smooth");
  gauss_dialog.field("Gaussian sigma", sigma);
  if (!gauss_dialog.ask())
    return;
  vil1_memory_image_of<float> input(img);
  vil1_memory_image_of<float> smooth = brip_vil1_float_ops::gaussian(input, sigma);
  vil1_memory_image_of<unsigned char> char_smooth =
    brip_vil1_float_ops::convert_to_byte(smooth);
  this->add_image(char_smooth);
}

void segv_segmentation_manager::convolution()
{
  vgui_dialog kernel_dlg("Load Kernel");
  static vcl_string kernel_filename = "c:/images";
  static vcl_string ext = "*.*";
  kernel_dlg.file("Kernel Filename:", ext, kernel_filename);
  if (!kernel_dlg.ask())
    return;
  vbl_array_2d<float> kernel = brip_vil1_float_ops::load_kernel(kernel_filename);

  //convert input image
  vil1_image img = selected_image();
  if (!img)
  {
    vcl_cout << "In segv_segmentation_manager::convolution() - no image\n";
    return;
  }
  vil1_memory_image_of<unsigned char> temp(img);
  vil1_memory_image_of<float> temp2 = brip_vil1_float_ops::convert_to_float(temp);

  //convolve
  vil1_memory_image_of<float> conv = brip_vil1_float_ops::convolve(temp2, kernel);

  //convert back to unsigned char
  vil1_memory_image_of<unsigned char> char_conv =
    brip_vil1_float_ops::convert_to_byte(conv);

  //display the image
  this->add_image(char_conv);
}

void segv_segmentation_manager::downsample()
{
  vil1_image img = selected_image();
  if (!img)
  {
    vcl_cout << "In segv_segmentation_manager::downsample) - no image\n";
    return;
  }
  static float filter_factor = 0.36f;
  vgui_dialog downsample_dialog("Downsample");
  downsample_dialog.field("Bert-Adelson Factor", filter_factor);
  if (!downsample_dialog.ask())
    return;
  vil1_memory_image_of<unsigned char> input(img);
  vil1_memory_image_of<float> inputf = brip_vil1_float_ops::convert_to_float(input);
  vil1_memory_image_of<float> half_res =
    brip_vil1_float_ops::half_resolution(inputf, filter_factor);
  vil1_memory_image_of<unsigned char> char_half_res =
    brip_vil1_float_ops::convert_to_byte(half_res);
  this->add_image(char_half_res);
}

void segv_segmentation_manager::harris_measure()
{
  vil1_image img = selected_image();
  if (!img)
  {
    vcl_cout << "In segv_segmentation_manager::harris_measure) - no image\n";
    return;
  }
  static sdet_harris_detector_params hdp;
  vgui_dialog harris_dialog("harris");
  harris_dialog.field("sigma", hdp.sigma_);
  harris_dialog.field("thresh", hdp.thresh_);
  harris_dialog.field("N = 2n+1, (n)", hdp.n_);
  harris_dialog.field("Max No Corners(percent)", hdp.percent_corners_);
  harris_dialog.field("scale_factor", hdp.scale_factor_);
  if (!harris_dialog.ask())
    return;
  sdet_harris_detector hd(hdp);
  hd.set_image(img);
  hd.extract_corners();
  vcl_vector<vsol_point_2d_sptr>& points = hd.get_points();
  int N = points.size();
  if (!N)
    return;
  bgui_vtol2D_tableau_sptr t2D = this->selected_vtol2D_tab();
  if (!t2D)
    return;
  t2D->clear_all();
  for (int i=0; i<N; i++)
    t2D->add_vsol_point_2d(points[i]);
  t2D->post_redraw();
}

void segv_segmentation_manager::beaudet_measure()
{
  vil1_image img = selected_image();
  if (!img)
  {
    vcl_cout <<"In segv_segmentation_manager::beaudet_measure) - no image\n";
    return;
  }
  static float sigma = 1.0f;
  //static float scale_factor = 0.04f;
  //static int n = 2;
  static float cmax=100;
  vgui_dialog harris_dialog("beaudet");
  harris_dialog.field("sigma", sigma);
  harris_dialog.field("range", cmax);
  if (!harris_dialog.ask())
    return;
  int w = img.width(), h = img.height();
  vil1_memory_image_of<unsigned char> input(img);
  vil1_memory_image_of<float> inputf = brip_vil1_float_ops::convert_to_float(input);
  vil1_memory_image_of<float> smooth = brip_vil1_float_ops::gaussian(inputf, sigma);
  vil1_memory_image_of<float> Ixx, Ixy, Iyy, b;
  Ixx.resize(w,h);  Ixy.resize(w,h);   Iyy.resize(w,h);
  brip_vil1_float_ops::hessian_3x3(smooth, Ixx, Ixy, Iyy);
  b = brip_vil1_float_ops::beaudet(Ixx, Ixy, Iyy);
  vil1_memory_image_of<unsigned char> uchar_b =
    brip_vil1_float_ops::convert_to_byte(b,0.0f, cmax);
  this->add_image(uchar_b);
}

void segv_segmentation_manager::vd_edges()
{
  this->clear_display();
  static bool agr = true;
  static sdet_detector_params dp;
  static float nm = 2.0;

  vgui_dialog vd_dialog("VD Edges");
  vd_dialog.field("Gaussian sigma", dp.smooth);
  vd_dialog.field("Noise Threshold", nm);
  vd_dialog.checkbox("Automatic Threshold", dp.automatic_threshold);
  vd_dialog.checkbox("Agressive Closure", agr);
  vd_dialog.checkbox("Compute Junctions", dp.junctionp);
  if (!vd_dialog.ask())
    return;
  dp.noise_multiplier=nm;
  if (agr)
    dp.aggressive_junction_closure=1;
  else
    dp.aggressive_junction_closure=0;
  vil1_image img = selected_image();
  if (!img)
  {
    vcl_cout << "In segv_segmentation_manager::vd_edges() - no image\n";
    return;
  }
  sdet_detector det(dp);
  det.SetImage(img);

  det.DoContour();
  vcl_vector<vtol_edge_2d_sptr>* edges = det.GetEdges();
  if (edges)
    this->draw_edges(*edges, true);
}

void segv_segmentation_manager::regions()
{
  this->clear_display();
  static bool debug = false;
  static bool agr = true;
  static bool residual = false;
  static sdet_detector_params dp;
  static float nm = 1.0;
  vgui_dialog region_dialog("Edgel Regions");
  region_dialog.field("Gaussian sigma", dp.smooth);
  region_dialog.field("Noise Threshold", nm);
  region_dialog.checkbox("Automatic Threshold", dp.automatic_threshold);
  region_dialog.checkbox("Agressive Closure", agr);
  region_dialog.checkbox("Compute Junctions", dp.junctionp);
  region_dialog.checkbox("Debug", debug);
  region_dialog.checkbox("Residual Image", residual);
  if (!region_dialog.ask())
    return;
  dp.noise_multiplier=nm;
  if (agr)
    dp.aggressive_junction_closure=1;
  else
    dp.aggressive_junction_closure=0;

  vil1_image img = selected_image();
  if (!img)
  {
    vcl_cout << "In segv_segmentation_manager::regions() - no image\n";
    return;
  }
  sdet_region_proc_params rpp(dp, true, debug, 2);
  sdet_region_proc rp(rpp);
  rp.set_image(img);
  rp.extract_regions();
  if (debug)
  {
    vil1_image ed_img = rp.get_edge_image();
#if 0
    vgui_image_tableau_sptr itab =  t2D->get_image_tableau();
    if (!itab)
    {
      vcl_cout << "In segv_segmentation_manager::regions() - null image tableau\n";
      return;
    }
    itab->set_image(ed_img);
    itab->post_redraw();
#endif
    this->add_image(ed_img);
  }
  if (!debug)
  {
    vcl_vector<vtol_intensity_face_sptr>& regions = rp.get_regions();
    this->draw_regions(regions, true);
  }
  if (residual)
  {
    vil1_image res_img = rp.get_residual_image();
#if 0
    vgui_image_tableau_sptr itab =  t2D->get_image_tableau();
    if (!itab)
    {
      vcl_cout << "In segv_segmentation_manager::regions() - null image tableau\n";
      return;
    }
    itab->set_image(res_img);
    itab->post_redraw();
#endif
    this->add_image(res_img);
  }
}

// Test calculated camera parameter matrices (K and M) by reading from a file.
// file should be in the following format:
// K n_views M1, M2, M3... Mn_views
// where K is the 3x3 intrinsic parameter matrix,
// n_views is an integer value > 0,
// and M1 - Mn_views are the 4x4 homogeneous extrinsic parameter matrices.
void segv_segmentation_manager::test_camera_parms()
{
  this->clear_display();
  vgui_dialog tcp_dialog("Test Camera Parms");
  static vcl_string camera_parms_filename = "/home/dec/camera_parms.left.txt";
  static vcl_string input_points_filename = "/home/dec/point_correspondences.left.txt";
  static vcl_string ext = "*.*";
  static bool show_input_points = true;
  static bool show_calculated_points = true;
  static int view_num = 1;
  tcp_dialog.file("camera parms file:", ext, camera_parms_filename);
  tcp_dialog.file("input points file:", ext, input_points_filename);
  tcp_dialog.field("view number:",view_num);
  tcp_dialog.checkbox("show input points",show_input_points);
  tcp_dialog.checkbox("show calculated points",show_calculated_points);

  if (!tcp_dialog.ask())
    return;

  if (show_calculated_points)
  {
    // read K
    vnl_matrix_fixed<double,3,3> K;
    vcl_ifstream parms_instream(camera_parms_filename.c_str());

    double k_values[9];
    for (int i=0; i < 9; i++)
    {
      parms_instream >> k_values[i];
      //vcl_cout << "k_values["<<i<<"] = "<<k_values[i] << '\n';
    }
    K.put(0,0,k_values[0]); K.put(0,1,k_values[1]); K.put(0,2,k_values[2]);
    K.put(1,0,k_values[3]); K.put(1,1,k_values[4]); K.put(1,2,k_values[5]);
    K.put(2,0,k_values[6]); K.put(2,1,k_values[7]); K.put(2,2,k_values[8]);

    vcl_cout << "K =\n" << K << '\n';

    // read number of views
    int n_views = 0;
    parms_instream >> n_views;

    // read M
    vnl_matrix_fixed<double,3,4> M;

    double m_values[12];
    double dummy;
    for (int v = 1; v <= view_num; v++)
    {
      if (v > n_views)
      {
        vcl_cout << "error: view number > n_views\n";
        break;
      }
      for (int i=0; i < 12; i++)
      {
        parms_instream >> m_values[i];
        //vcl_cout << "m_values["<<i<<"] = "<<m_values[i] << '\n';
      }
      // read 4th row, should just be [0 0 0 1]
      for (int i=0; i < 4; i++)
      {
        parms_instream >> dummy;
      }
    }
    parms_instream.close();

    M.put(0,0,m_values[0]); M.put(0,1,m_values[1]); M.put(0,2,m_values[2]);  M.put(0,3,m_values[3]);
    M.put(1,0,m_values[4]); M.put(1,1,m_values[5]); M.put(1,2,m_values[6]);  M.put(1,3,m_values[7]);
    M.put(2,0,m_values[8]); M.put(2,1,m_values[9]); M.put(2,2,m_values[10]); M.put(2,3,m_values[11]);

    vcl_cout << "M =\n" << M << '\n';

    //transform the grid points to the image
    sdet_grid_finder_params gfp;
    sdet_grid_finder gf(gfp);
    vcl_vector<vsol_point_2d_sptr> calculated_points;
    gf.transform_grid_points(K,M,calculated_points);

    // draw points on image
    vgui_style_sptr style = vgui_style::new_style(1.0f,0.0f,0.0f,5.0f,1.0f);
    this->draw_points(calculated_points,style);
  }
  if (show_input_points)
  {
    vcl_vector<vsol_point_2d_sptr> input_points;
    vcl_ifstream points_instream(input_points_filename.c_str());
    int n_points = 0;
    // read number of points
    points_instream >> n_points;
    // read grid points, discard
    double dummy;
    double* points_x = new double[n_points];
    double* points_y = new double[n_points];
    for (int i = 0; i < n_points; i++)
    {
      points_instream >> dummy; //x
      points_instream >> dummy; //y
    }

    // read number of views
    int n_views = 0;
    points_instream >> n_views;

    // read grid points for view number
    for (int v = 1; v <= view_num; v++)
    {
      if (v > n_views)
      {
        vcl_cout << "error: view number > n_views\n";
        break;
      }
      for (int i = 0; i < n_points; i++)
      {
        points_instream >> points_x[i];
        points_instream >> points_y[i];
      }
    }
    for (int i = 0; i < n_points; i++)
    {
      vsol_point_2d_sptr point = new vsol_point_2d(points_x[i],points_y[i]);
      input_points.push_back(point);
    }
    vgui_style_sptr style = vgui_style::new_style(0.0f,1.0f,0.0f,4.0f,1.0f);
    this->draw_points(input_points,style);
    delete[] points_x;
    delete[] points_y;
  }

  return;
}

void segv_segmentation_manager::fit_lines()
{
  this->clear_display();
  static sdet_grid_finder_params gfp;
  vcl_vector<vcl_string> choices;
  gfp.get_debug_choices(choices);
  static bool agr = true;
  static sdet_detector_params dp;
  dp.borderp=false;
  static sdet_fit_lines_params flp;
  static float nm = 2.0;
  static bool detect_grid=true;
  //static bool grid_debug=false;
  static bool matched_lines = false;
  static bool manual_pt_selection = false;
  vgui_dialog vd_dialog("Fit Lines");
  vd_dialog.field("Gaussian sigma", dp.smooth);
  vd_dialog.field("Noise Threshold", nm);
  vd_dialog.checkbox("Automatic Threshold", dp.automatic_threshold);
  vd_dialog.checkbox("Agressive Closure", agr);
  vd_dialog.checkbox("Compute Junctions", dp.junctionp);
  vd_dialog.field("Min Fit Length", flp.min_fit_length_);
  vd_dialog.field("RMS Distance", flp.rms_distance_);
  vd_dialog.field("Angle Tolerance", gfp.angle_tol_);
  vd_dialog.field("Line Count Threshold", gfp.thresh_);
  vd_dialog.checkbox("Detect Grid", detect_grid);
  vd_dialog.checkbox("Grid Debug Output", gfp.verbose_);
  vd_dialog.checkbox("Matched Lines", matched_lines);
  vd_dialog.checkbox("Use Manual Point Selection",manual_pt_selection);
  vd_dialog.choice("Choose Debug Line Display", choices, gfp.debug_state_);
  if (!vd_dialog.ask())
    return;
  dp.noise_multiplier=nm;
  if (agr)
    dp.aggressive_junction_closure=1;
  else
    dp.aggressive_junction_closure=0;
  dp.borderp = false;
  sdet_detector det(dp);
  vil1_image img = selected_image();

  if (!img)
  {
    vcl_cout << "In segv_segmentation_manager::lines() - no image\n";
    return;
  }
  det.SetImage(img);

  det.DoContour();
  vcl_vector<vtol_edge_2d_sptr>* edges = det.GetEdges();
  if (!edges)
  {
    vcl_cout << "No edges to fit lines\n";
    return;
  }
  sdet_fit_lines fl(flp);
  fl.set_edges(*edges);
  fl.fit_lines();
  vcl_vector<vsol_line_2d_sptr> lines = fl.get_line_segs();
  if (detect_grid)
  {
    sdet_grid_finder gf(gfp);
    if (!gf.set_lines(img.width(), img.height(), lines))
    {
      vcl_cout << "Less than two dominant groups\n";
      return;
    }
    vcl_vector<vsol_line_2d_sptr> mapped_lines;
    vcl_vector<vsol_line_2d_sptr> mapped_grid_lines;
    if (manual_pt_selection)
    {
      bgui_picker_tableau_sptr picktab = this->selected_picker_tab();
      if (!picktab)
        return;
      vsol_point_2d_sptr corners[4];
      vcl_cout << "Select the four corners of the grid, starting with "
               << "the upper left and moving clockwise.\n";
      for (int p=0; p<4; p++)
      {
        float x=0, y=0;
        picktab->pick_point(&x,&y);
        vcl_cout << "corner "<< p <<" (x=" << x << ", y=" << y <<")\n";
        corners[p] = new vsol_point_2d(x,y);
      }
      gf.compute_manual_homography(corners[0],corners[1],
                                   corners[2],corners[3]);
      if (!gfp.debug_state_)
        //gf.get_mapped_lines(mapped_lines);
        gf.get_backprojected_grid(mapped_lines);
      else
        gf.get_debug_lines(mapped_lines);

      this->draw_lines(mapped_lines);
      return;
    }

    gf.compute_homography();
    // double-check grid match
    if (!gf.check_grid_match(img))
    {
      // for now just display message - if this was a video process
      // we would want to disregard this homography and move on
      vcl_cout << "warning: grid match failed double-check\n";
    }
    if (!gfp.debug_state_)
      //gf.get_mapped_lines(mapped_lines);
      gf.get_backprojected_grid(mapped_lines);
    else
    {
      gf.get_debug_lines(mapped_lines);
      gf.get_debug_grid_lines(mapped_grid_lines);
      vgui_style_sptr style = vgui_style::new_style(1.0f,0.0f,0.0f,1.0f,1.0f);
      this->draw_lines(mapped_grid_lines,style);
    }
    this->draw_lines(mapped_lines);
    return;
  }
  this->draw_lines(lines);
}

void segv_segmentation_manager::test_face()
{
  vil1_image img = selected_image();
  if (!img)
  {
    vcl_cout << "In segv_segmentation_manager::test_face() - no image\n";
    return;
  }
  int sx = img.cols(), sy = img.rows();
  if (sx<10||sy<10)
    return;

  bgui_vtol2D_tableau_sptr t2D = this->selected_vtol2D_tab();
  if (!t2D)
    return;
  t2D->set_foreground(0.0, 1.0, 0.0);
  vsol_point_2d_sptr pa = new vsol_point_2d(1,1);
  vsol_point_2d_sptr pb = new vsol_point_2d(sx-2,1);
  vsol_point_2d_sptr pc = new vsol_point_2d(sx-2,sy-2);
  vsol_point_2d_sptr pd = new vsol_point_2d(1,sy-2);
  vsol_curve_2d_sptr cab = new vdgl_digital_curve(pa, pb);
  vsol_curve_2d_sptr cbc = new vdgl_digital_curve(pb, pc);
  vsol_curve_2d_sptr ccd = new vdgl_digital_curve(pc, pd);
  vsol_curve_2d_sptr cda = new vdgl_digital_curve(pd, pa);
  vtol_vertex_2d_sptr va = new vtol_vertex_2d(*pa);
  vtol_vertex_2d_sptr vb = new vtol_vertex_2d(*pb);
  vtol_vertex_2d_sptr vc = new vtol_vertex_2d(*pc);
  vtol_vertex_2d_sptr vd = new vtol_vertex_2d(*pd);
  vtol_edge_2d_sptr eab = new vtol_edge_2d(va, vb, cab);
  vtol_edge_2d_sptr ebc = new vtol_edge_2d(vb, vc, cbc);
  vtol_edge_2d_sptr ecd = new vtol_edge_2d(vc, vd, ccd);
  vtol_edge_2d_sptr eda = new vtol_edge_2d(vd, va, cda);
  vcl_vector<vtol_edge_sptr> edges;
  edges.push_back(eab->cast_to_edge());
  edges.push_back(ebc->cast_to_edge());
  edges.push_back(ecd->cast_to_edge());
  edges.push_back(eda->cast_to_edge());
  vtol_one_chain_sptr b_onch = new vtol_one_chain(edges,true);
  vtol_face_2d_sptr b_f = new  vtol_face_2d(b_onch);

  int px = sx/2, py = sy/2;
  vsol_point_2d_sptr p1 = new vsol_point_2d(px+3,py+3);
  vsol_point_2d_sptr p2 = new vsol_point_2d(px,py-3);
  vsol_point_2d_sptr p3 = new vsol_point_2d(px-3,py+3);
  vsol_curve_2d_sptr c12 = new vdgl_digital_curve(p1, p2);
  vsol_curve_2d_sptr c23 = new vdgl_digital_curve(p2, p3);
  vsol_curve_2d_sptr c31 = new vdgl_digital_curve(p3, p1);
  vtol_vertex_2d_sptr v1 = new vtol_vertex_2d(*p1);
  vtol_vertex_2d_sptr v2 = new vtol_vertex_2d(*p2);
  vtol_vertex_2d_sptr v3 = new vtol_vertex_2d(*p3);
  vtol_edge_2d_sptr e12 = new vtol_edge_2d(v1, v2, c12);
  vtol_edge_2d_sptr e23 = new vtol_edge_2d(v2, v3, c23);
  vtol_edge_2d_sptr e31 = new vtol_edge_2d(v3, v1, c31);
  edges.clear();
  edges.push_back(e12->cast_to_edge());
  edges.push_back(e23->cast_to_edge());
  edges.push_back(e31->cast_to_edge());
  vtol_one_chain_sptr t_onch = new vtol_one_chain(edges,true);
  vtol_face_2d_sptr t_f = new  vtol_face_2d(t_onch);
  t2D->add_face(b_f);   t2D->add_face(t_f);
  t2D->set_foreground(1.0, 0.0, 0.0);
  for (int x = 0; x<sx; x+=20)
    for (int y = 0; y<sy; y+=20)
    {
      vtol_vertex_2d_sptr v = new vtol_vertex_2d(x, y);
      t2D->add_vertex(v);
    }
}

void segv_segmentation_manager::test_digital_lines()
{
  bgui_vtol2D_tableau_sptr t2D = this->selected_vtol2D_tab();
  if (!t2D)
    return;
  t2D->set_foreground(1.0, 1.0, 0.0);
  vsol_point_2d_sptr pa = new vsol_point_2d(0,0);
  vsol_point_2d_sptr pb = new vsol_point_2d(20,0);
  vsol_point_2d_sptr pc = new vsol_point_2d(10, 20);
  vsol_point_2d_sptr pd = new vsol_point_2d(20,20);
  vsol_point_2d_sptr pe = new vsol_point_2d(20, 10);
  vsol_point_2d_sptr pf = new vsol_point_2d(0, 20);
  vdgl_digital_curve_sptr cab = new vdgl_digital_curve(pa, pb);
  vdgl_digital_curve_sptr cac = new vdgl_digital_curve(pa, pc);
  vdgl_digital_curve_sptr cad = new vdgl_digital_curve(pa, pd);
  vdgl_digital_curve_sptr cae = new vdgl_digital_curve(pa, pe);
  vdgl_digital_curve_sptr caf = new vdgl_digital_curve(pa, pf);
}

void segv_segmentation_manager::display_IHS()
{
  vil1_image img = selected_image();
  if (!img)
  {
    vcl_cout << "In segv_segmentation_manager::display_IHS() - no image\n";
    return;
  }
  vil1_memory_image_of<float> I,H,S;

  vil1_memory_image_of<vil1_rgb<unsigned char> > in_image(img), out_image;
  if (!in_image)
    return;
  brip_vil1_float_ops::convert_to_IHS(in_image, I, H, S);
  brip_vil1_float_ops::display_IHS_as_RGB(I, H, S, out_image);
  this->add_image(out_image);
}

vtol_face_2d_sptr
segv_segmentation_manager::face_at(const int col, const int row)
{
  vtol_face_2d_sptr out;
  bgui_vtol2D_tableau_sptr v2D;
  vgui_tableau_sptr top_tab = grid_->get_tableau_at(col, row);
  if (!top_tab)
    return out;
  v2D.vertical_cast(vgui_find_below_by_type_name(top_tab,
                                                 vcl_string("bgui_vtol2D_tableau")));
  if (!v2D)
    return out;

  vtol_topology_object_sptr to = v2D->get_temp();
  if (!to)
    return out;
  vtol_face_sptr f = to->cast_to_face();
  out = f->cast_to_face_2d();
  return out;
}

void segv_segmentation_manager::compute_mutual_info()
{
  static strk_region_info_params rip;
  static bool mapped_ = false;
  vgui_dialog info_dialog("Mutual Information");
  info_dialog.field("Min Gradient Magnitude", rip.min_gradient_);
  info_dialog.field("Parzen Sigma", rip.parzen_sigma_);
  info_dialog.checkbox("Add Gradient Info", rip.gradient_info_);
  info_dialog.checkbox("Add Color Info", rip.color_info_);
  info_dialog.checkbox("Verbose", rip.verbose_);
  info_dialog.checkbox("Debug", rip.debug_);
  info_dialog.checkbox("Show Mapped Image", mapped_);
  if (!info_dialog.ask())
    return;
  vcl_cout << rip << '\n';
  //Get the v2D tableau at each grid.  V2D0 is considered the model
  unsigned row= 0, col=0, nrows = grid_->rows(), ncols = grid_->cols();
  grid_->get_last_selected_position(&col, &row);
  int other_row = nrows-1-row, other_col = ncols-1-col;
  vtol_face_2d_sptr f0 = face_at(col, row);
  vtol_face_2d_sptr fi = face_at(other_col, other_row);
  //if (!f0||!fi)
  //  return;
  vil1_image img_0 = this->image_at(col, row);
  vil1_image img_i = this->image_at(other_col, other_row);
  if (!img_0||!img_i)
    return;
  strk_region_info ri(rip);
  ri.set_face_0(f0);
  ri.set_face_i(fi);
  ri.set_image_0(img_0);
  ri.set_image_i(img_i);
  if (mapped_)
  {
    vil1_memory_image_of<unsigned char> im0 = ri.image_0();
    this->set_image_at(col, row, im0);
    vil1_memory_image_of<unsigned char> imi = ri.image_i();
    this->set_image_at(other_col, other_row, imi);
  }
  ri.evaluate_info();
}

void segv_segmentation_manager::compute_background_info()
{
  if (!foreground_face_||!background_face_)
  {
    vcl_cout << "No foreground or background faces defined\n";
    return;
  }
  static strk_region_info_params rip;
  vgui_dialog bf_dialog("Background/Forground Entropy");
  bf_dialog.field("Min Gradient Magnitude", rip.min_gradient_);
  bf_dialog.field("Parzen Sigma", rip.parzen_sigma_);
  bf_dialog.checkbox("Add Gradient Info", rip.gradient_info_);
  bf_dialog.checkbox("Add Color Info", rip.color_info_);
  bf_dialog.checkbox("Verbose", rip.verbose_);
  bf_dialog.checkbox("Debug", rip.debug_);
  if (!bf_dialog.ask())
    return;
  vcl_cout << rip << '\n';
  unsigned row= 0, col=0;
  grid_->get_last_selected_position(&col, &row);
  vil1_image img_0 = this->image_at(col, row);
  if (!img_0)
    return;
  strk_region_info ri(rip);
  ri.set_face_0(foreground_face_);
  ri.set_background_face(background_face_);
  ri.set_image_0(img_0);
  ri.evaluate_background_info();
}

void segv_segmentation_manager::rotate_image()
{
  vil1_image img = selected_image();
  if (!img)
  {
    vcl_cout << "In segv_segmentation_manager::rotate_image - no image\n";
    return;
  }
  vil1_memory_image_of<float> flt =
    brip_vil1_float_ops::convert_to_float(img);
  static double angle = 0;
  vgui_dialog rotate_dialog("Rotate Image");
  rotate_dialog.field("Rotation Angle (deg)", angle);
  if (!rotate_dialog.ask())
    return;

  vil1_memory_image_of<float> temp = brip_vil1_float_ops::rotate(flt, angle);
  vil1_memory_image_of<unsigned char> out_image =
    brip_vil1_float_ops::convert_to_byte(temp, 0, 255);
  this->add_image(out_image);
}

void segv_segmentation_manager::create_box()
{
  vgui_rubberband_tableau_sptr rubber = this->selected_rubber_tab();
  rubber->rubberband_box();
}

void segv_segmentation_manager::create_polygon()
{
  vgui_rubberband_tableau_sptr rubber = this->selected_rubber_tab();
  rubber->rubberband_polygon();
}

bool segv_segmentation_manager::
set_image_at(const unsigned col, const unsigned row, vil1_image& image)
{
  vgui_tableau_sptr top_tab = grid_->get_tableau_at(col, row);
  if (!top_tab)
    return false;

  bgui_image_tableau_sptr itab;
  itab.vertical_cast(vgui_find_below_by_type_name(top_tab,
                                                  vcl_string("vgui_image_tableau")));
  if (!itab)
  {
    vcl_cout << "Unable to get bgui_image_tableau at (" << col
             << ", " << row << ")\n";
    return false;
  }
  itab->set_image(image);
  return true;
}

void segv_segmentation_manager::set_foreground_face()
{
  unsigned row= 0, col=0;
  grid_->get_last_selected_position(&col, &row);
  foreground_face_ = face_at(col, row);
}

void segv_segmentation_manager::set_background_face()
{
  unsigned row= 0, col=0;
  grid_->get_last_selected_position(&col, &row);
  background_face_ = face_at(col, row);
}

void segv_segmentation_manager::compute_parallel_coverage()
{
  static brip_para_cvrg_params pcp;
  static bool combined=true;
  vgui_dialog para_dialog("Parallel Coverage");
  para_dialog.field("Sigma", pcp.sigma_);
  para_dialog.field("Projection Width", pcp.proj_width_);
  para_dialog.field("Projection Height", pcp.proj_height_);
  para_dialog.checkbox("Display Coverage and Direction Combined", combined);
  para_dialog.checkbox("Verbose", pcp.verbose_);
  if (!para_dialog.ask())
    return;
  unsigned row= 0, col=0;
  grid_->get_last_selected_position(&col, &row);
  vil1_image image = this->image_at(col, row);
  brip_para_cvrg pc(pcp);
  pc.do_coverage(image);
  vil1_image cov_image;
  if (combined)
    cov_image = pc.get_combined_image();
  else
    cov_image = pc.get_detection_image();
  this->add_image(cov_image);
}

void segv_segmentation_manager::compute_watershed_regions()
{
  static bool print_region_info = false;
  static bool merge_regions = false;
  static sdet_watershed_region_proc_params wrpp;
  vgui_dialog watershed_dialog("Watershed Regions");
  watershed_dialog.field("Sigma", wrpp.wp_.sigma_);
  watershed_dialog.field("Thresh", wrpp.wp_.thresh_);
  watershed_dialog.checkbox("Merge Regions", merge_regions);
  watershed_dialog.field("Merge Order", wrpp.merge_priority_);
  watershed_dialog.field("Merge Tolerance", wrpp.merge_tol_);
  watershed_dialog.checkbox("Print Region info", print_region_info);
  watershed_dialog.checkbox("Verbose", wrpp.wp_.verbose_);
  if (!watershed_dialog.ask())
    return;
  vcl_cout << wrpp << '\n';
  unsigned row= 0, col=0;
  grid_->get_last_selected_position(&col, &row);
  vil1_image image = this->image_at(col, row);
  vtol_face_2d_sptr f = this->face_at(col,row);
  vsol_box_2d_sptr box;
  if (f)
    box = f->get_bounding_box();
  sdet_watershed_region_proc wrp(wrpp);
  wrp.set_image(image, box);
  wrp.extract_regions();
  if (print_region_info)
    wrp.print_region_info();
  //   vil1_image region_image = wrp.overlay_image();
  //
  vil1_image region_image = wrp.region_image();
  this->add_image(region_image);
  if (merge_regions)
    wrp.merge_regions();
  bgui_vtol2D_tableau_sptr t2D = this->selected_vtol2D_tab();
  if (!t2D)
    return;
  vcl_vector<vsol_polygon_2d_sptr> polys = wrp.boundaries();
  for (vcl_vector<vsol_polygon_2d_sptr>::iterator pit = polys.begin();
       pit != polys.end(); pit++)
    t2D->add_vsol_polygon_2d(*pit);
  t2D->post_redraw();
}

void segv_segmentation_manager::find_vehicle()
{
  static bool show_region_boxes = false;
  static sdet_vehicle_finder_params vfp;
  vgui_dialog vehicle_finder_dialog("Vehicle_Finder Regions");
  vehicle_finder_dialog.field("Watershed Sigma", vfp.wrpp_.wp_.sigma_);
  vehicle_finder_dialog.field("Watershed Grad Diff Thresh",
                              vfp.wrpp_.wp_.thresh_);
  vehicle_finder_dialog.field("Region Merge Tolerance",
                              vfp.wrpp_.merge_tol_);
  vehicle_finder_dialog.field("Para proj width", vfp.pcp_.proj_width_);
  vehicle_finder_dialog.field("Para proj height", vfp.pcp_.proj_height_);
  vehicle_finder_dialog.field("Min Region Area", vfp.wrpp_.min_area_);
  vehicle_finder_dialog.checkbox("Watershed Verbose", vfp.wrpp_.wp_.verbose_);
  vehicle_finder_dialog.field("Vehicle Search Radius", vfp.search_radius_);
  vehicle_finder_dialog.field("Shadow Thresh", vfp.shadow_thresh_);
  vehicle_finder_dialog.field("Para Cvrg Thresh", vfp.para_thresh_);
  vehicle_finder_dialog.field("Region Distance Scale", vfp.distance_scale_);
  vehicle_finder_dialog.checkbox("Show Para and Shadow Boxes",
                                 show_region_boxes);
  vehicle_finder_dialog.checkbox("Verbose", vfp.verbose_);

  if (!vehicle_finder_dialog.ask())
    return;
  unsigned row= 0, col=0;
  grid_->get_last_selected_position(&col, &row);
  vil1_image image = this->image_at(col, row);
  sdet_vehicle_finder vf(vfp);
  vf.set_image(image);
  bgui_picker_tableau_sptr picktab = this->selected_picker_tab();
  vcl_cout << "\nPick vehicle...\n";
  float x0=0, y0=0;
  picktab->pick_point(&x0,&y0);
  vcl_cout << "Pick(" << x0 << ' ' << y0 << ")\n";
  vf.set_pick((int)x0, (int)y0);
  vsol_box_2d_sptr box = vf.search_box();
  vsol_polygon_2d_sptr poly = bsol_algs::poly_from_box(box);
  bgui_vtol2D_tableau_sptr t2D = this->selected_vtol2D_tab();
  if (!t2D)
    return;
  t2D->add_vsol_polygon_2d(poly);
  vf.detect_shadow_regions();
  vsol_polygon_2d_sptr hull = vf.shadow_hull();

  if (hull&&show_region_boxes)
  {
    t2D->add_vsol_polygon_2d(hull);
  }
  vf.detect_para_regions();
  hull = vf.para_hull();
  if (hull&&show_region_boxes)
  {
    t2D->add_vsol_polygon_2d(hull);
  }
  if (vf.compute_track_boundary())
  {
    poly = vf.vehicle_track_poly();
    t2D->add_vsol_polygon_2d(poly);
  }
  t2D->post_redraw();
}

void segv_segmentation_manager::display_histogram_track()
{
  vgui_dialog hist_dlg("Histogram Track File");
  static vcl_string hist_filename = "";
  static vcl_string ext = "*.*";
  hist_dlg.file("Tracked Histogram filename:", ext, hist_filename);
  if (!hist_dlg.ask())
    return;
  vcl_ifstream hist_instr(hist_filename.c_str());
  unsigned int start_frame, n_frames, n_pixels;
  unsigned int n_int_bins, n_grad_dir_bins, n_color_bins;
  float diameter, aspect_ratio;
  vnl_matrix<float> hist_data;
  if (!strk_io::read_histogram_data(hist_instr, start_frame, n_frames,
                                   n_pixels, diameter, aspect_ratio,
                                   n_int_bins, n_grad_dir_bins, n_color_bins,
                                   hist_data))
    return;
  vil1_memory_image_of<float> temp =
    brip_vil1_float_ops::convert_to_float(hist_data);
  vil1_memory_image_of<unsigned char> image =
    brip_vil1_float_ops::convert_to_byte(temp);
  this->add_image(image);
}

void segv_segmentation_manager::set_range_params()
{
  bgui_image_tableau_sptr itab = this->selected_image_tab();
  if (!itab)
    return;
  static double min = 0.0, max = 255;
  static float gamma = 1.0;
  static bool invert = false;
  vgui_dialog range_dlg("Set Range Map Params");
  range_dlg.field("Range min:", min);
  range_dlg.field("Range max:", max);
  range_dlg.field("Gamma:", gamma);
  range_dlg.checkbox("Invert:", invert);
  if (!range_dlg.ask())
    return;

  vgui_range_map_params_sptr rmps = new vgui_range_map_params(min,
                                                              max,
                                                              gamma,
                                                              invert);

  itab->set_mapping(rmps);
}
