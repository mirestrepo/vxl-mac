//
// vsrl_manager.cxx
//
// This program was written to test the Dense Matching software
//
// G.W. Brooksby
// 02/13/03

#include <vcl_cstdlib.h>
#include <vcl_iostream.h>
#include <vil/vil_load.h>
#include <vil/vil_scale_intensities.h>
#include <vil/vil_save.h>
#include <vgui/vgui.h>
#include <vgui/vgui_dialog.h>
#include <vgui/vgui_menu.h>
#include <vgui/vgui_image_tableau.h>
#include <vgui/vgui_viewer2D_tableau.h>
#include <vgui/vgui_easy2D_tableau.h>
#include <vgui/vgui_soview2D.h>
#include <vgui/vgui_grid_tableau.h>
#include <vgui/vgui_shell_tableau.h>
#include <vgui/vgui_dialog.h>
#include <vsrl/vsrl_stereo_dense_matcher.h>
#include <sdet/sdet_region_proc_params.h>
#include <sdet/sdet_region_proc.h>
#include <vtol/vtol_one_chain_sptr.h>
#include <vtol/vtol_one_chain.h>
#include <vtol/vtol_vertex_2d.h>
#include <vtol/vtol_vertex_2d_sptr.h>
#include <vtol/vtol_vertex.h>
#include <vtol/vtol_edge.h>
#include <vtol/vtol_edge_2d.h>
#include <vtol/vtol_edge_2d_sptr.h>
#include "vsrl_menus.h"
#include "vsrl_manager.h"
#include "vsrl_point_picker.h"

// static manager instance
vsrl_manager* vsrl_manager::instance_=0;

//ensure only one instance is created
vsrl_manager *vsrl_manager::instance()
{
  if(!instance_)
    {
      instance_ = new vsrl_manager();
      instance_->init();
    }
  return vsrl_manager::instance_;
}

vsrl_manager::vsrl_manager():vgui_wrapper_tableau(){}

vsrl_manager::~vsrl_manager(){}

void vsrl_manager::init()
{

  // Load the image tableaux
  itabL_ = vgui_image_tableau_new();
  itabR_ = vgui_image_tableau_new(); 
  dimg_tab_ = vgui_image_tableau_new();  // disparity image tableau
  disparity_bias_=0;

  // Put the image tableax into an easy2D tableau
  e2d0_ = vgui_easy2D_tableau_new(itabL_);
  e2d1_ = vgui_easy2D_tableau_new(itabR_);
  e2d2_ = vgui_easy2D_tableau_new(dimg_tab_);

  // Set up characteristics of points to be drawn
  e2d0_->set_foreground(1,0,0);
  e2d1_->set_foreground(1,0,0);
  e2d2_->set_foreground(1,0,0);
  e2d0_->set_point_radius(5);
  e2d1_->set_point_radius(5);
  e2d2_->set_point_radius(5);
  e2d0_->set_line_width(2);
  e2d1_->set_line_width(2);
  e2d2_->set_line_width(2);

  // Put the easy2D tableaux into the viewers
  vgui_viewer2D_tableau_sptr viewer0 = vgui_viewer2D_tableau_new(e2d0_);
  vgui_viewer2D_tableau_sptr viewer1 = vgui_viewer2D_tableau_new(e2d1_);
  vgui_viewer2D_tableau_sptr viewer2 = vgui_viewer2D_tableau_new(e2d2_);

  // Put the viewers into tableaus for picking points 
  vpicker0_=new vsrl_point_picker(viewer0);
  vpicker1_=new vsrl_point_picker(viewer1);
  vpicker2_=new vsrl_point_picker(viewer2);

  //Put the viewers into a grid
  grid_ = new vgui_grid_tableau(3,1);
  grid_->add_at(vpicker0_, 0,0);
  grid_->add_at(vpicker1_, 1,0);
  grid_->add_at(vpicker2_, 2,0);
  grid_->set_selected(0,0);
  grid_->set_uses_paging_events(false); // disable paging
  grid_->set_grid_size_changeable(false); // disable adding panes

  // Put the grid into a shell tableau at the top the hierarchy
  vgui_shell_tableau_new shell(grid_);

  // Get a parameters object
  params_ = vsrl_parameters::instance();

  this->add_child(shell);
}

void vsrl_manager::quit()
{
  vcl_exit(1);
}

void vsrl_manager::load_left_image()
{
  vgui_dialog load_image_dlg("Load Image file");
  static vcl_string image_filename = "";
  static vcl_string ext = "*.*";
  load_image_dlg.file("Image Filename:", ext, image_filename);
  if (!load_image_dlg.ask()) return;
  imgL_ = vil_load(image_filename.c_str());
  itabL_->set_image(imgL_);
  return;
}

void vsrl_manager::load_right_image()
{
  vgui_dialog load_image_dlg("Load Image file");
  static vcl_string image_filename = "";
  static vcl_string ext = "*.*";
  load_image_dlg.file("Image Filename:", ext, image_filename);
  if (!load_image_dlg.ask()) return;
  imgR_ = vil_load(image_filename.c_str());
  itabR_->set_image(imgR_);
  return;
}

void vsrl_manager::load_disparity_image()
{
  vgui_dialog load_image_dlg("Load Disparity Image file");
  static vcl_string image_filename = "";
  static vcl_string ext = "*.*";
  load_image_dlg.file("Disparity Image Filename:", ext, image_filename);
  if (!load_image_dlg.ask()) return;
  disp_img_ = vil_load(image_filename.c_str());
  vil_memory_image_of<unsigned char> real_image(disp_img_);

  vil_image scaled_image = scale_image(real_image);

  // Display the scaled image
  dimg_tab_->set_image(scaled_image);
}

void vsrl_manager::save_disparity_image()
{
  vgui_dialog save_image_dlg("Save Disparity Image file");
  static vcl_string image_filename = "";
  static vcl_string ext = "*.tif";
  save_image_dlg.file("Disparity Image Filename", ext, image_filename);
  if (!save_image_dlg.ask()) return;
  vil_memory_image_of<unsigned char> disp(disp_img_);
  if (!vil_save(disp,image_filename.c_str())) {
    vcl_cout << "Error saving disparity image!" << vcl_endl;
  }
  return;
}

void vsrl_manager::load_params_file()
{
  //
  vgui_dialog load_params_dlg("Load Dense Matcher Parameters file");
  static vcl_string params_filename = "";
  static vcl_string ext = "*.*";
  load_params_dlg.file("Dense Matcher Parameter Filename:", ext, params_filename);
  if (!load_params_dlg.ask()) return;

  // Due to the need for a non-const char* being fed to the .load function
  // we have to construct a filename that is NOT const!
  char* filename = new char[params_filename.length()+1]; // get a new char* array
  params_filename.copy(filename,params_filename.length()); // copy the string into it
  filename[params_filename.length()]=0; //add null terminator
  params_->load(filename); // load the parameters file
  disparity_bias_ = params_->correlation_range;
  delete [] filename;  // delete the filename
}

void vsrl_manager::point_pick()
{
  vcl_cerr << "vsrl_manager::point_pick() not yet implemented" << vcl_endl;
  return;
}

void vsrl_manager::clear_all()
{
  e2d0_->clear();
  e2d1_->clear();
  e2d2_->clear();
  this->post_redraw();
  return;
}

bool vsrl_manager::handle(vgui_event const & event)
{

  this->child.handle(event);

  if (event.type == vgui_BUTTON_DOWN &&
      event.button == vgui_LEFT && 
      !event.modifier)
    {
      put_points();
    }
  else if (event.type == vgui_BUTTON_DOWN &&
           event.button == vgui_LEFT && 
           event.modifier == vgui_SHIFT)
    {
      put_lines();
    }
  return true;
}

bool vsrl_manager::validate_point(vnl_vector<float>* pt)
{
  if (pt->x() < 0 ||
      pt->y() < 0 ||
      pt->x() > disp_img_.cols() ||
      pt->y() > disp_img_.rows() )
    {
      vcl_cout << "Error: point out of range of disparity image."
                << vcl_endl;
      return false;
    }  
  return true;
}

int vsrl_manager::get_disparity(vnl_vector<float>* pt)
{
  int pixel_val=0;
  vil_memory_image_of<unsigned char> disp(disp_img_);
  pixel_val = disp(pt->x(),pt->y());
  if (pixel_val > 0) {
    // we subtract the disparity bias, plus 1 for the indexing offset
    //    pixel_val -= (disparity_bias_ + 1); 
    pixel_val -= (params_->correlation_range + 1); 
    vcl_cout << "Disparity: " << pixel_val << vcl_endl;
  }
  if (pixel_val == 0) vcl_cout << "No disparity mapping found for this point!" << vcl_endl;
  return pixel_val;
}

bool vsrl_manager::put_points()
{
  unsigned r=0, c=0;
  grid_->get_last_selected_position(&c, &r);
  
  // determine which grids to update
  vnl_vector<float>* pos;
  // Point Picked in Left Pane
  if (c==0)
    {
      pos = vpicker0_->point_;  // get the last point picked
      if (!validate_point(pos)) return true;
      int disp = get_disparity(pos);
      float x=pos->x()+disp;
      vpicker1_->put_point(x,pos->y());
      vpicker2_->put_point(pos->x(),pos->y());
    }
  // Point Picked in Right Pane
  if (c==1)
    {
      pos = vpicker1_->point_;  // get the last point picked
      if (!validate_point(pos)) return true;
      int disp = get_disparity(pos);
      float x=pos->x()- disp;
      vpicker0_->put_point(x,pos->y());
      vpicker2_->put_point(pos->x(),pos->y());
    }
  // Point Picked in Disparity Pane
  if (c==2)
    {
      pos = vpicker2_->point_;  // get the last point picked
      if (!validate_point(pos)) return true;
      int disp = get_disparity(pos);
      float x=pos->x()+disp;
      vpicker0_->put_point(pos->x(),pos->y());
      vpicker1_->put_point(x,pos->y());
    }
  this->post_redraw();
  return true;
}

bool vsrl_manager::put_lines()
{
  unsigned r=0, c=0;
  grid_->get_last_selected_position(&c, &r);
  
  // determine which grids to update
  vnl_vector<float>* pos;
  // Point Picked in Left Pane
  if (c==0)
    {
      pos = vpicker0_->point_;  // get the last point picked
      vpicker1_->put_H_line(pos->x(),pos->y());
      vpicker2_->put_H_line(pos->x(),pos->y());
    }
  // Point Picked in Right Pane
  if (c==1)
    {
      pos = vpicker1_->point_;  // get the last point picked
      vpicker0_->put_H_line(pos->x(),pos->y());
      vpicker2_->put_H_line(pos->x(),pos->y());
    }
  // Point Picked in Disparity Pane
  if (c==2)
    {
      pos = vpicker2_->point_;  // get the last point picked
      vpicker0_->put_H_line(pos->x(),pos->y());
      vpicker1_->put_H_line(pos->x(),pos->y());
    }
  this->post_redraw();
  return true;
}

bool vsrl_manager::do_dense_matching()
{

  if (!imgL_ || !imgR_) return false;

  // The parameters used will be the default parameters or
  // the parameters loaded manually from a file.
  // Now create a dense matcher with the images that are loaded.
  vcl_cout << "Begin Stereo Dense Matcher...";
  vsrl_stereo_dense_matcher matcher(imgL_,imgR_);
  vcl_cout << "Setting Correlation Range...";
  matcher.set_correlation_range(params_->correlation_range);
  vcl_cout << "Running Dense Matcher." << vcl_endl;
  // Run the dense matcher.
  matcher.execute();

  // Get & display the disparity image
  // Get a buffer the size of the left image
  vil_memory_image_of<unsigned char> buffer(imgL_);
  // Zero out the buffer
  for (int x=0;x<buffer.width();x++)
    for (int y=0;y<buffer.height();y++)
      buffer(x,y)=0;
  // Get the disparities into the buffer
  for (int y=0;y<buffer.height();y++) {
    for (int x=0;x<buffer.width();x++) {
      int disparity = matcher.get_disparity(x,y);
      int value = disparity + params_->correlation_range+1;
      if (value < 0)
        value = 0;
      if (value>2*params_->correlation_range+1)
        value=0;
      buffer(x,y)=value;
    }
  }
  // Display the disparity image
  disp_img_ = buffer;
  vil_image scaled_image = scale_image(buffer);
  dimg_tab_->set_image(scaled_image);
  
  vcl_cout << "Dense Matcher complete." << vcl_endl;
  this->post_redraw();
  return true;
}

vil_image vsrl_manager::scale_image(vil_memory_image_of<unsigned char> img)
{
  double maxval = 0;
  for (int x=0;x<img.width();x++) {
    for (int y=0;y<img.height();y++) {
      if (img(x,y) > maxval) {
        maxval = img(x,y);
      }
    }
  }
  double scale = 255.0/maxval;
  double shift = 0;
  vil_image scaled_image = vil_scale_intensities(img, scale, shift);
  return scaled_image;
}

void vsrl_manager::find_regions()
{
  this->clear_all();
  static bool debug = false;
  static bool agr = true;
  static bool residual = false;
  static sdet_detector_params dp;
  dp.noise_multiplier=1.0;
  vgui_dialog region_dialog("Edgel Regions");
  region_dialog.field("Gaussian sigma", dp.smooth);
  region_dialog.field("Noise Threshold", dp.noise_multiplier);
  region_dialog.checkbox("Automatic Threshold", dp.automatic_threshold);
  region_dialog.checkbox("Agressive Closure", agr);
  region_dialog.checkbox("Compute Junctions", dp.junctionp);
  region_dialog.checkbox("Debug", debug);
  region_dialog.checkbox("Residual Image", residual);
  if (!region_dialog.ask())
    return;
  if (agr)
    dp.aggressive_junction_closure=1;
  else
    dp.aggressive_junction_closure=0;
  
  sdet_region_proc_params rpp(dp, true, debug, 2);
  sdet_region_proc rp(rpp);
  rp.set_image(imgL_);
  rp.extract_regions();
  if (debug)
    {
      vil_image ed_img = rp.get_edge_image();
      vgui_image_tableau_sptr itab =  e2d0_->get_image_tableau();
      if (!itab)
        {
          vcl_cout << "In segv_segmentation_manager::regions() - null image tableau\n";
          return;
        }
      itab->set_image(ed_img);
    }
  if (!debug)
    {
      vcl_vector<vdgl_intensity_face_sptr>& regions = rp.get_regions();
      this->draw_regions(regions, true);
    }
  if (residual)
    {
      vil_image res_img = rp.get_residual_image();
      vgui_image_tableau_sptr itab =  e2d0_->get_image_tableau();
      if (!itab)
        {
          vcl_cout << "In segv_segmentation_manager::regions() - null image tableau\n";
          return;
        }
      itab->set_image(res_img);
    }
}

void vsrl_manager::draw_regions(vcl_vector<vdgl_intensity_face_sptr>& regions,
                                             bool verts)
{

  // This segment of code is ripped from various places in brl...bgui.

   for (vcl_vector<vdgl_intensity_face_sptr>::iterator rit = regions.begin();
        rit != regions.end(); rit++)
     {
       vtol_face_2d_sptr f = (*rit)->cast_to_face_2d();
       edge_list* edges = f->edges();

       vgui_soview2D_group* vsovg = new vgui_soview2D_group();

       for (edge_list::iterator eit = edges->begin(); eit != edges->end(); eit++) {

         vtol_edge_2d_sptr e = (*eit)->cast_to_edge_2d();

         vgui_soview2D_linestrip* e_line = new vgui_soview2D_linestrip();

         vsol_curve_2d_sptr c = e->curve();

         if (!c) {
           vcl_cout << "vsrl_manager::draw_regions - null curve." << vcl_endl;
           return;
         }
         if (c->cast_to_digital_curve())
           {
             vdgl_digital_curve_sptr dc = c->cast_to_digital_curve();
             //get the edgel chain
             vdgl_interpolator_sptr itrp = dc->get_interpolator();
             vdgl_edgel_chain_sptr ech = itrp->get_edgel_chain();
             
             //n, x, and y are in the parent class vgui_soview2D_linestrip
             e_line->n = ech->size();
             //offset the coordinates for display (may not be needed)
             e_line->x = new float[e_line->n], e_line->y = new float[e_line->n];
             for (unsigned int i=0; i<e_line->n;i++)
               {
                 vdgl_edgel ed = (*ech)[i];
                 e_line->x[i]=ed.get_x();
                 e_line->y[i]=ed.get_y();
               }
           }
         else {
           vcl_cout << "vsrl_manager::draw_regions -"
                    << " attempt to draw an edge with unknown curve geometry\n";           
         }

         vsovg->ls.push_back(e_line);

       }

       e2d0_->add(vsovg);

       if (verts)
         {
           vcl_vector<vtol_vertex_sptr>* vts = f->vertices();
           for (vcl_vector<vtol_vertex_sptr>::iterator vit = vts->begin();
                vit != vts->end(); vit++)
             {
               vtol_vertex_2d_sptr v = (*vit)->cast_to_vertex_2d();
               e2d0_->add_point(v->x(),v->y());
             }
           delete vts;
         }
     }
}

void
vsrl_manager::set_params()
{

  // establish the variables.  Should already have instantiated params_
  int corr_range=params_->correlation_range;
  double inner_cost=params_->inner_cost;
  double outer_cost=params_->outer_cost;
  double continuity_cost=params_->continuity_cost;
  int correlation_window_width=params_->correlation_window_width;
  int correlation_window_height=params_->correlation_window_height;
  double bias_cost=params_->bias_cost;
  double common_intensity_diff=params_->common_intensity_diff;

  vgui_dialog params_dialog("Dense Matcher Parameters");
  params_dialog.field("Correlation Range:", corr_range);
  params_dialog.field("Inner Cost:", inner_cost);
  params_dialog.field("Outer Cost:", outer_cost);
  params_dialog.field("Continuity Cost:",continuity_cost);
  params_dialog.field("Correlation Window Width:", correlation_window_width);
  params_dialog.field("Correlation Window Height:", correlation_window_height);
  params_dialog.field("Bias Cost:",bias_cost);
  params_dialog.field("Common Intensity Difference:",common_intensity_diff);
  if (!params_dialog.ask()) {
    return;
  }
  else {
    params_->correlation_range = corr_range;
    params_->inner_cost = inner_cost;
    params_->outer_cost = outer_cost;
    params_->continuity_cost = continuity_cost;
    params_->correlation_window_width = correlation_window_width;
    params_->correlation_window_height = correlation_window_height;
    params_->bias_cost = bias_cost;
    params_->common_intensity_diff = common_intensity_diff;
  }
  return;

}
