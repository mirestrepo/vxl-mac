// This is brl/bmvl/bmvv/bmvv_multiview_manager.cxx
#include "bmvv_multiview_manager.h"
//:
// \file
// \author J.L. Mundy

#include <vcl_cstdlib.h> // for vcl_exit()
#include <vcl_iostream.h>
#include <vil/vil_load.h>
#include <vgl/vgl_homg_point_2d.h>
#include <vgl/vgl_homg_line_2d.h>
#include <vdgl/vdgl_digital_curve.h>
#include <vdgl/vdgl_interpolator.h>
#include <vdgl/vdgl_edgel_chain.h>
#include <vsol/vsol_curve_2d.h>
#include <bxml/bxml_vtol_io.h>
#include <vgui/vgui.h>
#include <vgui/vgui_find.h>
#include <vgui/vgui_macro.h>
#include <vgui/vgui_dialog.h>
#include <vgui/vgui_soview.h>
#include <vgui/vgui_image_tableau.h>
#include <vgui/vgui_easy2D_tableau.h>
#include <vgui/vgui_viewer2D_tableau.h>
#include <vgui/vgui_grid_tableau.h>
#include <vgui/vgui_shell_tableau.h>
#include <bgui/bgui_image_tableau.h>
#include <bgui/bgui_vtol2D_tableau.h>
#include <bgui/bgui_picker_tableau.h>
#include <sdet/sdet_detector_params.h>
#include <sdet/sdet_detector.h>
#include <gevd/gevd_clean_edgels.h>
#include <vgui/vgui_key.h>
#include <vgui/vgui_modifier.h>
#include <vtol/vtol_face_2d_sptr.h>
#include <gevd/gevd_float_operators.h>
#include <sdet/sdet_region_proc_params.h>
#include <sdet/sdet_region_proc.h>

// pilou's line
#include <bdgl/bdgl_curve_description.h>
#include <bdgl/bdgl_curve_tracker.h>
#include <bdgl/bdgl_curve_matcher.h>

//static live_video_manager instance
bmvv_multiview_manager *bmvv_multiview_manager::instance_ = 0;
//===============================================================
//: The singleton pattern - only one instance of the manager can occur
//==============================================================
bmvv_multiview_manager *bmvv_multiview_manager::instance()
{
  if (!instance_)
    {
      instance_ = new bmvv_multiview_manager();
      instance_->init();
    }
  return bmvv_multiview_manager::instance_;
}

//==================================================================
//: constructors/destructor
//==================================================================
bmvv_multiview_manager::
bmvv_multiview_manager() : vgui_wrapper_tableau()
{
}

bmvv_multiview_manager::~bmvv_multiview_manager()
{
}

//======================================================================
//: set up the tableaux at each grid cell
//  the vtol2D_tableaux have been initialized in the constructor
//======================================================================
void bmvv_multiview_manager::init()
{
  grid_ = vgui_grid_tableau_new(2,1);
  grid_->set_grid_size_changeable(true);
  unsigned int col=0, row = 0;
  for (; col<2; col++)
    {
      vgui_image_tableau_sptr itab = bgui_image_tableau_new();
      bgui_vtol2D_tableau_sptr btab = bgui_vtol2D_tableau_new(itab);
      vtol_tabs_.push_back(btab);
      bgui_picker_tableau_new pcktab(btab);
      vgui_viewer2D_tableau_sptr v2d = vgui_viewer2D_tableau_new(pcktab);
      grid_->add_at(v2d, col, row);
    }
  vgui_shell_tableau_sptr shell = vgui_shell_tableau_new(grid_);
  this->add_child(shell);
}

//=========================================================================
//: make an event handler
// note that we have to get an adaptor and set the tableau to receive events
// this handler does nothing but is a place holder for future event processing
// For now, just pass the events down to the child tableaux
//=========================================================================
bool bmvv_multiview_manager::handle(const vgui_event &e)
{
  return this->child.handle(e);
}

//====================================================================
//: Gets the picker tableau at the given position.
//  Returns a null tableau if it fails
//====================================================================
bgui_picker_tableau_sptr
bmvv_multiview_manager::get_picker_tableau_at(unsigned col, unsigned row)
{
  vgui_tableau_sptr top_tab = grid_->get_tableau_at(col, row);
  if (top_tab)
    {
      bgui_picker_tableau_sptr tt;
      tt.vertical_cast(vgui_find_below_by_type_name(top_tab,
                                                    vcl_string("bgui_picker_tableau")));
      if (tt)
        return tt;
    }
  vgui_macro_warning << "Unable to get bgui_picker_tableau at (" << col << ", "
                     << row << ")\n";
  return 0;
}

//====================================================================
//: Gets the picker tableau at the currently selected grid position
//  Returns a null tableau if it fails
//====================================================================
bgui_picker_tableau_sptr bmvv_multiview_manager::get_selected_picker_tableau()
{
  unsigned int row=0, col=0;
  grid_->get_last_selected_position(&col, &row);
  return this->get_picker_tableau_at(col, row);
}

//====================================================================
//: Gets the picker tableau at the given position.
//  if col, row are out of bounds then null is returned
//  row is currently not used but may be when we have more than 2 cameras
//====================================================================
bgui_vtol2D_tableau_sptr bmvv_multiview_manager::get_vtol2D_tableau_at(unsigned col, unsigned row)
{
  if (row!=0)
    return 0;
  bgui_vtol2D_tableau_sptr btab = 0;
  if (col==0||col==1)
    btab = vtol_tabs_[col];
  return btab;
}


//=================================================================
//: Get the vtol_2D_tableau at the currently selected grid cell.
//=================================================================
bgui_vtol2D_tableau_sptr bmvv_multiview_manager::get_selected_vtol2D_tableau()
{
  unsigned int row =0, col=0;
  grid_->get_last_selected_position(&col, &row);
  return this->get_vtol2D_tableau_at(col, row);
}

void bmvv_multiview_manager::quit()
{
  vcl_exit(1);
}

//=========================================================================
//: load an image an put it in the currently selected grid cell
//=========================================================================
void bmvv_multiview_manager::load_image_file(vcl_string image_filename, bool greyscale, unsigned col, unsigned row)
{
  img_ = vil_load(image_filename.c_str());
  bgui_vtol2D_tableau_sptr btab = this->get_vtol2D_tableau_at(col, row);
  if (btab)
    {
      vgui_image_tableau_sptr itab = btab->get_image_tableau();
      itab->set_image(img_);
      return;
    }
  vcl_cout << "In bmvv_multiview_manager::load_image_file() - null tableau\n";
}

//=========================================================================
//: load an image an put it in the currently selected grid cell
//=========================================================================
void bmvv_multiview_manager::load_image()
{
  bool greyscale = false;
  vgui_dialog load_image_dlg("Load Image");
  static vcl_string image_filename = "";
  static vcl_string ext = "*.*";
  load_image_dlg.file("Image Filename:", ext, image_filename);
  load_image_dlg.checkbox("greyscale ", greyscale);
  if (!load_image_dlg.ask())
    return;
  img_ = vil_load(image_filename.c_str());
  bgui_vtol2D_tableau_sptr btab = this->get_selected_vtol2D_tableau();
  if (btab)
    {
      vgui_image_tableau_sptr itab = btab->get_image_tableau();
      itab->set_image(img_);
      return;
    }
  vcl_cout << "In bmvv_multiview_manager::load_image() - null tableau\n";
}

//===================================================================
//: Clear the display
//===================================================================
void bmvv_multiview_manager::clear_display()
{
  bgui_vtol2D_tableau_sptr btab = this->get_selected_vtol2D_tableau();
  if (btab)
    btab->clear();
  else
    vcl_cout << "In bmvv_multiview_manager::clear_display() - null tableau\n";
}

//===================================================================
//: clear all selections in both panes
//===================================================================
void bmvv_multiview_manager::clear_selected()
{
  for (vcl_vector<bgui_vtol2D_tableau_sptr>::iterator bit = vtol_tabs_.begin();
       bit != vtol_tabs_.end(); bit++)
    if (*bit)
      (*bit)->deselect_all();
}


//======================================================================
//: Draw a set of intensity faces on the currently selected grid cell
//======================================================================
void bmvv_multiview_manager::draw_regions(vcl_vector<vdgl_intensity_face_sptr>& regions,
                                          bool verts)
{
  vcl_vector<vtol_face_2d_sptr> faces;
  for (vcl_vector<vdgl_intensity_face_sptr>::iterator rit = regions.begin();
       rit != regions.end(); rit++)
    faces.push_back((*rit)->cast_to_face_2d());

  bgui_vtol2D_tableau_sptr btab = this->get_selected_vtol2D_tableau();
  if (btab)
    btab->add_faces(faces, verts);
  else
    vcl_cout << "In bmvv_multiview_manager::draw_regions() - null tableau\n";
}

//========================================================================
//: Compute Van Duc edges for the currently selected grid cell
//========================================================================
void bmvv_multiview_manager::vd_edges()
{
  static bool agr = true;
  static sdet_detector_params dp;
  vgui_dialog* vd_dialog = new vgui_dialog("VD Edges");
  vd_dialog->field("Gaussian sigma", dp.smooth);
  vd_dialog->field("Noise Threshold", dp.noise_multiplier);
  vd_dialog->checkbox("Automatic Threshold", dp.automatic_threshold);
  vd_dialog->checkbox("Agressive Closure", agr);
   vd_dialog->checkbox("Compute Junctions", dp.junctionp);
  if (!vd_dialog->ask())
    return;
  if (agr)
    dp.aggressive_junction_closure=1;
  else
    dp.aggressive_junction_closure=0;
  sdet_detector det(dp);

  bgui_vtol2D_tableau_sptr btab = this->get_selected_vtol2D_tableau();
  if (btab)
    {
      vgui_image_tableau_sptr itab = btab->get_image_tableau();
      img_ = itab->get_image();
    }
  else
    {
      vcl_cout << "In bmvv_multiview_manager::vd_edges() - null tableau\n";
      return;
    }
  det.SetImage(img_);
  det.DoContour();
  vcl_vector<vtol_edge_2d_sptr>* edges = det.GetEdges();

  //display the edges
  if (btab&&edges)
    btab->add_edges(*edges, true);
  else
    {
      vcl_cout << "In bmvv_multiview_manager::vd_edges() - null edges or null tableau\n";
      return;
    }
}

void bmvv_multiview_manager::regions()
{
  this->clear_display();
  static bool agr = true;
  static sdet_detector_params dp;
  vgui_dialog* vd_dialog = new vgui_dialog("VD Edges");
  vd_dialog->field("Gaussian sigma", dp.smooth);
  vd_dialog->field("Noise Threshold", dp.noise_multiplier);
  vd_dialog->checkbox("Automatic Threshold", dp.automatic_threshold);
  vd_dialog->checkbox("Agressive Closure", agr);
  if (!vd_dialog->ask())
    return;
  if (agr)
    dp.aggressive_junction_closure=1;
  else
    dp.aggressive_junction_closure=0;
  bool debug = false;
  sdet_region_proc_params rpp(dp);
  sdet_region_proc rp(rpp);
  rp.set_image(img_);
  rp.extract_regions();
  //this should be somewhere else
  if (debug)
    {
      bgui_vtol2D_tableau_sptr btab = this->get_selected_vtol2D_tableau();
      if (!btab)
        {
          vcl_cout << "In bmvv_multiview_manager::regions() - "
                   << "null tableau\n";
          return;
        }
      vil_image ed_img = rp.get_edge_image();
      vgui_image_tableau_sptr itab =  btab->get_image_tableau();
      if (!itab)
        {
          vcl_cout << "In bmvv_multiview_manager::regions() - "
                   << "null image tableau\n";
          return;
        }
      itab->set_image(ed_img);
      return;
    }
  vcl_vector<vdgl_intensity_face_sptr>& regions = rp.get_regions();
  this->draw_regions(regions, true);
}

#ifdef HAS_XERCES
void bmvv_multiview_manager::read_xml_edges()
{
  this->clear_display();
  vgui_dialog load_image_dlg("Load XML edges");
  static vcl_string xml_filename = "";
  static vcl_string ext = "*.*";
  load_image_dlg.file("XML filename:", ext, xml_filename);
  if (!load_image_dlg.ask())
    return;
  bgui_vtol2D_tableau_sptr btab = this->get_selected_vtol2D_tableau();
  if (!btab)
    {
      vcl_cout << "In bmvv_multiview_manager::regions() - "
               << "null tableau\n";
      return;
    }
  vcl_vector<vtol_edge_2d_sptr> edges;
  if (bxml_vtol_io::read_edges(xml_filename, edges))
    btab->add_edges(edges, true);
}
#endif

//====================================================================
//: a test FMatrix
// epipole at infinity so points map to horizontal lines
//===================================================================
FMatrix bmvv_multiview_manager::test_fmatrix()
{
  vnl_matrix<double> m(3, 3, 0.0);
  m.put(1,2, 1.0);
  m.put(2,1,-1.0);
  return m;
}

//===================================================================
//: pick a point in the left image and show the corresponding epipolar
//  line in the right image
//===================================================================
void bmvv_multiview_manager::show_epipolar_line()
{
  this->clear_display();
  vgui::out << "pick point in left image\n";
  unsigned int col=0, row=0;//left image
  bgui_picker_tableau_sptr pkt = this->get_picker_tableau_at(col, row);
  if (!pkt)
    {
      vcl_cout << "In bmvv_multiview_manager::show_epipolar_line() - "
               << "null tableau\n";
      return;
    }
  float x = 0, y=0;
  pkt->pick_point(&x, &y);
  vgui::out << "p(" << x << " " << y << ")\n";
  vcl_cout << "p(" << x << " " << y << ")\n";

  col = 1;//right image
  bgui_vtol2D_tableau_sptr v2D = this->get_vtol2D_tableau_at(col,row);

  //temporary test for FMatrix
  FMatrix f = this->test_fmatrix();
  vgl_homg_point_2d<double> pl(x,y);
  vgl_homg_line_2d<double> lr = f.image2_epipolar_line(pl);
  //end test
  if (v2D)
    {
      v2D->add_infinite_line(lr.a(), lr.b(), lr.c());
      v2D->post_redraw();
    }
}

//===================================================================
//: capture corresponding curves in left and right image
//===================================================================
void bmvv_multiview_manager::select_curve_corres()
{
  for (vcl_vector<bgui_vtol2D_tableau_sptr>::iterator bit = vtol_tabs_.begin();
       bit != vtol_tabs_.end(); bit++)
    if (*bit)
      {
        vcl_vector<vgui_soview*> sovs = (*bit)->get_selected_soviews();
        for (unsigned int i = 0; i<sovs.size(); i++)
          {
            vgui_soview* sov = sovs[i];
            int id = sov->get_id();
            vcl_cout << "id = " << id << "\n";
            vtol_edge_2d_sptr e = (*bit)->get_mapped_edge(id);
            vsol_curve_2d_sptr c = e->curve();
            vdgl_digital_curve_sptr dc = c->cast_to_digital_curve();
            // pilou's code
            vdgl_interpolator_sptr interp = dc->get_interpolator();
            vdgl_edgel_chain_sptr  ec = interp->get_edgel_chain();
            bdgl_curve_description descr(ec);
            descr.info();
            if (dc) (*bit)->add_digital_curve(dc);
          }
      }
  this->clear_selected();
}


// do the edge matching/tracking
// [PLB]
//-----------------------------------------------------------
void bmvv_multiview_manager::track_edges()
{
  // get parameters
  this->clear_display();
  static bdgl_curve_matcher_params mp(1.0, 10.0, 0.31416);
  static bdgl_curve_tracker_params tp(1e6);

  vgui_dialog* tr_dialog = new vgui_dialog("Edge Tracking");
  tr_dialog->field("Matching threshold", tp.match_thres_);
  tr_dialog->field("Pixel scale", mp.image_scale_);
  tr_dialog->field("Gradient scale", mp.grad_scale_);
  tr_dialog->field("Angle scale", mp.angle_scale_);
  if (!tr_dialog->ask())
    return;
  tp.match_params_ = mp;
  bdgl_curve_tracker tracker(tp);

  // embedded VD edges computations
  // VD parameters
  static bool agr = true;
  static sdet_detector_params dp;
  vgui_dialog* vd_dialog = new vgui_dialog("VD Edges");
  vd_dialog->field("Gaussian sigma", dp.smooth);
  vd_dialog->field("Noise Threshold", dp.noise_multiplier);
  vd_dialog->checkbox("Automatic Threshold", dp.automatic_threshold);
  vd_dialog->checkbox("Agressive Closure", agr);
  if (!vd_dialog->ask())
    return;
  if (agr)
    dp.aggressive_junction_closure=1;
  else
    dp.aggressive_junction_closure=0;
  sdet_detector det1(dp);
  sdet_detector det2(dp);

  // passing types
  vcl_vector<vtol_edge_2d_sptr>* edges;
  vsol_curve_2d_sptr c;
  vdgl_digital_curve_sptr dc;
  vdgl_interpolator_sptr interp;
  vdgl_edgel_chain_sptr  ec;
  vcl_vector< vdgl_edgel_chain_sptr > ecl;
  bgui_vtol2D_tableau_sptr btab;

  // first image
  det1.SetImage(this->get_image_at(0,0));
  det1.DoContour();
  edges = det1.GetEdges();
  //display the edges
#if 0
  btab = this->get_vtol2D_tableau_at(0,0);
  if (btab)
    btab->add_edges(*edges, true);
  else
    {
      vcl_cout << "In bmvv_multiview_manager::track_edges() - null tableau\n";
      return;
    }
#endif // 0
  // pass the edges
  ecl.clear();
  for (unsigned int i=0;i<edges->size();i++){
    c = (*edges)[i]->curve();
    dc = c->cast_to_digital_curve();
    interp = dc->get_interpolator();
    ec = interp->get_edgel_chain();
    ecl.push_back(ec);
  }
  tracker.input_curve_.push_back(ecl);

  // second image
  det2.SetImage(this->get_image_at(1,0));
  det2.DoContour();
  edges = det2.GetEdges();
  //display the edges
#if 0
  btab = this->get_vtol2D_tableau_at(1,0);
  if (btab)
    btab->add_edges(*edges, true);
  else
    {
      vcl_cout << "In bmvv_multiview_manager::track_edges() - null tableau\n";
      return;
    }
#endif // 0
  ecl.clear();
  for (unsigned int i=0;i<edges->size();i++){
    c = (*edges)[i]->curve();
    dc = c->cast_to_digital_curve();
    interp = dc->get_interpolator();
    ec = interp->get_edgel_chain();
    ecl.push_back(ec);
  }
  tracker.input_curve_.push_back(ecl);

  // compute the tracking/matching
  tracker.track();

  // output the results : generate links between the curves in both images

  //display the edges
  edges = det1.GetEdges();
  //display the edges
  btab = this->get_vtol2D_tableau_at(0,0);
  if (btab){
    tracker.draw_lines(0, btab);
  } else {
    vcl_cout << "In bmvv_multiview_manager::track_edges() - null tableau\n";
    return;
  }
  edges = det2.GetEdges();
  //display the edges
  btab = this->get_vtol2D_tableau_at(1,0);
  if (btab){
    tracker.draw_lines(1, btab);
  } else {
      vcl_cout << "In bmvv_multiview_manager::track_edges() - null tableau\n";
      return;
  }
  return;
}

//====================================================================
//: Gets the image at the given position.
//  if col, row are out of bounds then null is returned
//  row is currently not used but may be when we have more than 2 cameras
//====================================================================
vil_image bmvv_multiview_manager::get_image_at(unsigned col, unsigned row)
{
  bgui_vtol2D_tableau_sptr btab = this->get_vtol2D_tableau_at(col, row);
  if (btab)
    {
      vgui_image_tableau_sptr itab = btab->get_image_tableau();
      return itab->get_image();
    }
  vcl_cout << "In bmvv_multiview_manager::get_image_at() - null tableau\n";
  return 0;
}
