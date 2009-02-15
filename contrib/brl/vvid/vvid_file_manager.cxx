// This is brl/vvid/vvid_file_manager.cxx
#include "vvid_file_manager.h"
//:
// \file
// \author J.L. Mundy

#include <vcl_vector.h>
#include <vcl_iostream.h>
#include <vul/vul_timer.h>
#include <vil1/vil1_image.h>
#include <vil1/vil1_save.h>
#include <vil1/vil1_memory_image_of.h>
#include <vidl_vil1/vidl_vil1_movie.h>
#include <vidl_vil1/vidl_vil1_clip.h>
#include <vidl_vil1/vidl_vil1_io.h>
#include <vidl_vil1/vidl_vil1_frame.h>
#include <vgui/vgui.h>
#include <vgui/vgui_error_dialog.h>
#include <vgui/vgui_adaptor.h>
#include <vgui/vgui_dialog.h>
#include <vgui/vgui_utils.h>
#include <vgui/vgui_shell_tableau.h>
#include <vgui/vgui_style_sptr.h>
#include <vgui/vgui_style.h>
#include <bgui/bgui_image_tableau.h>
#include <bgui/bgui_vtol2D_tableau.h>
#include <bgui/bgui_picker_tableau.h>
#include <bgui/bgui_bargraph_clipon_tableau.h>
#include <vgui/vgui_viewer2D_tableau.h>
#include <vgui/vgui_grid_tableau.h>
#include <vgui/vgui_image_tableau.h>
#include <vgui/vgui_rubberband_tableau.h>
#include <bgui/bgui_vtol2D_rubberband_client.h>
#include <vgui/vgui_composite_tableau.h>
#include <vsol/vsol_point_2d.h>
#include <vtol/vtol_face_2d.h>
#include <brip/brip_vil1_float_ops.h>
#include <bsol/bsol_algs.h>
#include <btol/btol_face_algs.h>
#include <sdet/sdet_harris_detector_params.h>
#include <sdet/sdet_detector_params.h>
#include <sdet/sdet_fit_lines_params.h>
#include <sdet/sdet_grid_finder_params.h>
#include <sdet/sdet_vehicle_finder_params.h>
#include <sdet/sdet_vehicle_finder.h>
#include <strk/strk_tracker_params.h>
#include <strk/strk_info_tracker_params.h>
#include <strk/strk_info_model_tracker_params.h>
#include <strk/strk_art_info_model.h>
#include <vpro/vpro_frame_diff_process.h>
#include <vpro/vpro_motion_process.h>
#include <vpro/vpro_lucas_kanade_process.h>
#include <vpro/vpro_harris_corner_process.h>
#include <vpro/vpro_edge_process.h>
#include <vpro/vpro_edge_line_process.h>
#include <vpro/vpro_grid_finder_process.h>
#include <strk/strk_corr_tracker_process.h>
#include <strk/strk_info_model_tracker_process.h>
#include <strk/strk_info_tracker_process.h>
#include <strk/strk_track_display_process.h>
#include <strk/strk_feature_capture_process.h>
#include <strk/strk_snippet_extractor_process.h>
#include <vpro/vpro_basis_generator_process.h>
#include <vpro/vpro_fourier_process.h>
#include <vpro/vpro_spatial_filter_process.h>
#include <strk/strk_art_model_display_process.h>
#include <vpro/vpro_ihs_process.h>
#include <vpro/vpro_half_res_process.h>
#include <strk/strk_io.h>
//static manager instance
vvid_file_manager *vvid_file_manager::instance_ = 0;

//The vvid_file_manager is a singleton class
vvid_file_manager *vvid_file_manager::instance()
{
  if (!instance_)
  {
    instance_ = new vvid_file_manager();
    instance_->init();
  }
  return vvid_file_manager::instance_;
}

//======================================================================
//: set up the tableaux at each grid cell
//======================================================================
void vvid_file_manager::init()
{
  grid_ = vgui_grid_tableau_new(2,1);
  grid_->set_grid_size_changeable(true);

  //  itab0_ = vgui_image_tableau_new();
  itab0_ = bgui_image_tableau_new();
  easy0_ = bgui_vtol2D_tableau_new(itab0_);
  bgui_vtol2D_rubberband_client* cl0 =  new bgui_vtol2D_rubberband_client(easy0_);

  rubber0_ = vgui_rubberband_tableau_new(cl0);
  vgui_composite_tableau_new comp0(easy0_,rubber0_);
  picktab0_ = bgui_picker_tableau_new(comp0);
  v2D0_ = vgui_viewer2D_tableau_new(picktab0_);
  grid_->add_at(v2D0_, 0,0);

  itab1_ = bgui_image_tableau_new();
  easy1_ = bgui_vtol2D_tableau_new(itab1_);
  v2D1_ = vgui_viewer2D_tableau_new(easy1_);
  grid_->add_at(v2D1_, 1,0);
  vgui_shell_tableau_sptr shell = vgui_shell_tableau_new(grid_);
  this->add_child(shell);
  bargraph_ = 0;
  //add a clipon display for bargraph
  //  vgui_easy2D_tableau_sptr easy;
  //  easy.vertical_cast(easy1_);
  //  bargraph_ = new bgui_bargraph_clipon_tableau(easy);

  stem_ = 0;
  long_tip_ =0;
  short_tip_=0;
  art_model_=0;
  background_model_ = 0;
  display_frame_repeat_=1;
  display_frame_skip_=0;
  skip_counter_=0;
  start_frame_ = 0;
  end_frame_ = 0;
  on_style_  = vgui_style::new_style(0.0, 1.0, 0.0, 3.0, 4.0);
  off_style_ = vgui_style::new_style(1.0, 0.0, 0.0, 3.0, 4.0);
}

//-----------------------------------------------------------
// constructors/destructor
// start with a single pane
vvid_file_manager::vvid_file_manager(): vgui_wrapper_tableau()
{
  width_ = 512;
  height_ = 512;
  track_ = false;
  color_label_ = false;
  window_ = 0;
  frame_trail_.clear();
  my_movie_=(vidl_vil1_movie*)0;
  win_ = 0;
  cache_frames_ = false;
  save_display_ = false;
  play_video_ = true;
  pause_video_ = false;
  next_frame_ = false;
  prev_frame_ = false;
  save_display_ = false;
  overlay_pane_ = true;
  time_interval_ = 10.0;
  video_process_ = 0;
  art_model_ = 0;
  display_frame_repeat_=1;
  display_frame_skip_=0;
  skip_counter_=0;
}

vvid_file_manager::~vvid_file_manager()
{
}


// make an event handler
// note that we have to get an adaptor and set the tableau to receive events
bool vvid_file_manager::handle(const vgui_event &e)
{
  return this->child.handle(e);
}

//: quit the application
void vvid_file_manager::quit()
{
  vgui::quit();
}
//-------------------------------------------------------------
//: Display a processed image
//
void vvid_file_manager::display_image()
{
  if (!video_process_)
    return;
  if (itab1_)
    itab1_->set_image(video_process_->get_output_image());
}

//-------------------------------------------------------------
//: Display a set of spatial objects
//
void vvid_file_manager::display_spatial_objects()
{
  static bool toggle = false;
  if (!video_process_)
    return;
  vcl_vector<vsol_spatial_object_2d_sptr> const& sos =
    video_process_->get_output_spatial_objects();
  if (easy0_)
  {
    //easy0_->clear_all();
    if (color_label_)
    {
      float r,g,b;
      //If tracking is on then we maintain a queue of points
      if (track_)
      {
        frame_trail_.add_spatial_objects(sos);
        vcl_vector<vsol_spatial_object_2d_sptr> temp;
        frame_trail_.get_spatial_objects(temp);
        for (unsigned int i=0;i<temp.size();i++) {
          set_changing_colors( temp[i]->get_tag_id() , &r, &g, &b );
          vgui_style_sptr style = vgui_style::new_style(r, g, b, 1.0, 2.0);
          easy0_->set_vsol_spatial_object_2d_style(temp[i], style );
          easy0_->add_spatial_object(temp[i]);
        }
      }
      else
      {
        for (unsigned int i=0;i<sos.size();i++) {
          set_changing_colors( sos[i]->get_tag_id() , &r, &g, &b );
#ifdef DEBUG
          vcl_cout<<'('<<sos[i]->get_tag_id()<<")\n";
#endif
          vgui_style_sptr style = vgui_style::new_style(r, g, b, 1.0, 2.0);
          easy0_->set_vsol_spatial_object_2d_style(sos[i], style );
          easy0_->add_spatial_object(sos[i]);
        }
      }
    }
    else
    {
      //If tracking is on then we maintain a queue of points
      if (track_)
      {
        frame_trail_.add_spatial_objects(sos);
        vcl_vector<vsol_spatial_object_2d_sptr> temp;
        frame_trail_.get_spatial_objects(temp);
        easy0_->add_spatial_objects(temp);
      }
      else
      {
        if (toggle)
        {
          easy0_->add_spatial_objects(sos, off_style_);
          toggle = !toggle;
        }
        else
        {
          easy0_->add_spatial_objects(sos, on_style_);
          toggle = !toggle;
        }
      }
    }
  }
}

// set changing colors for labelling curves, points, etc
//-----------------------------------------------------------------------------
void vvid_file_manager::set_changing_colors(int num, float *r, float *g, float *b)
{
  int strength = num/6;
  int pattern  = num%6;
  strength %= 20;
  float s = 1.0f - strength * 0.05f;

  switch (pattern)
  {
    case 0 : (*r) = s; (*g) = 0; (*b) = 0; break;
    case 1 : (*r) = 0; (*g) = s; (*b) = 0; break;
    case 2 : (*r) = 0; (*g) = 0; (*b) = s; break;
    case 3 : (*r) = s; (*g) = s; (*b) = 0; break;
    case 4 : (*r) = 0; (*g) = s; (*b) = s; break;
    case 5 : (*r) = s; (*g) = 0; (*b) = s; break;
    default: (*r) = 0; (*g) = 0; (*b) = 0; break; // this will never happen
  }
#ifdef DEBUG
  vcl_cout<<"color : "<<(*r)<<" : "<<(*g)<<" : "<<(*b)<<'\n';
#endif

  return;
}


//-------------------------------------------------------------
//: Display topology objects
//
void vvid_file_manager::display_topology()
{
  if (!easy0_)
    return;
  vul_timer t;
  vcl_vector<vtol_topology_object_sptr> const & topos =
    video_process_->get_output_topology();
  easy0_->clear_all();
    //If tracking is on then we maintain a queue of points
  if (track_)
  {
    frame_trail_.add_topology_objects(topos);
    vcl_vector<vtol_topology_object_sptr> temp;
    frame_trail_.get_topology_objects(temp);
    easy0_->add_topology_objects(temp);
  }
  else
    easy0_->add_topology_objects(topos);
#ifdef DEBUG
  vcl_cout << "display " << topos.size()
           << " topology objs in " << t.real() << " msecs.\n";
#endif
  easy0_->post_redraw();
}

void vvid_file_manager::display_bargraph(vcl_vector<float> const& data)
{
  if (!bargraph_)
    return;
  bargraph_->update(data);
}

//-----------------------------------------------------------------------------
//: Loads a video file, e.g. avi into the viewer
//-----------------------------------------------------------------------------
void vvid_file_manager::load_video_file()
{
  play_video_ = true;
  pause_video_ = false;
  next_frame_ = false;
  prev_frame_ = false;
  save_display_ = false;
  video_process_ = 0;
  frame_trail_.clear();
  vgui_dialog load_video_dlg("Load video file");
  static vcl_string image_filename = "";
  static vcl_string ext = "";
  load_video_dlg.file("Filename:", ext, image_filename);
  load_video_dlg.checkbox("Cache Frames ", cache_frames_);
  if (!load_video_dlg.ask())
    return;

  my_movie_ = vidl_vil1_io::load_movie(image_filename.c_str());
  if (!my_movie_) {
    vgui_error_dialog("Failed to load movie file");
    return;
  }
  tabs_.clear();
  int n_frames = my_movie_->length();
  start_frame_ = 0;
  end_frame_ = n_frames-1;
  vidl_vil1_movie::frame_iterator pframe = my_movie_->first();
  vil1_image img = pframe->get_image();
  //  vil1_image second = (++pframe)->get_image();
  height_ = img.height();
  width_ = img.width();
  vcl_cout << "Video Height " << height_ << vcl_endl
           << "Video Width  " << width_ << vcl_endl;
  if (win_)
    win_->reshape(2*width_, height_);
  int i = 0;
  if (cache_frames_)
  {
    for (pframe = my_movie_->first(); pframe!=my_movie_->last(); ++pframe)
    {
      vil1_image img = pframe->get_image();
      vgui_image_tableau_sptr itab = vgui_image_tableau_new(img);
      bgui_vtol2D_tableau_new  e(itab);
      tabs_.push_back(e);
      vcl_cout << "Loading Frame [" << i << "]: (" <<width_ <<'x'<<height_ << ")\n";
      ++i;
    }
    v2D0_->child.assign(tabs_[0]);
    itab1_->set_image(tabs_[0]->get_image_tableau()->get_image());
  }
  else
  {
    //v2D0_->child.assign(easy0_);
    //don't display frame in process window
#if 0
    itab1_->set_image(img);
#endif
    itab0_->set_image(img);
  }
  grid_->post_redraw();
  vgui::run_till_idle();
}

//----------------------------------------------
void vvid_file_manager::cached_play()
{
  vul_timer t;
  for (vcl_vector<bgui_vtol2D_tableau_sptr>::iterator vit = tabs_.begin();
       vit != tabs_.end()&&play_video_; vit++)
  {
    //pause by repeating the same frame
    if (pause_video_&&play_video_)
    {
      if (next_frame_&&vit!=tabs_.end()-2)
      {
        vit+=2;
        next_frame_ = false;
      }
      if (prev_frame_&&vit!=tabs_.begin()+2)
      {
        vit--;
        prev_frame_ = false;
      }
      vit--;
    }

    v2D0_->child.assign(*vit);
      //Here we can put some stuff to control the frame rate. Hard coded to
      //a delay of 10 for now
    while (t.all()<time_interval_) ;
    //force the new frame to be displayed
    grid_->post_redraw();
    vgui::run_till_idle();
    t.mark();
  }
}

//----------------------------------------------
void vvid_file_manager::un_cached_play()
{
  if (!my_movie_)
  {
    vcl_cout << "No movie has been loaded\n";
    return;
  }
  if (video_process_)
    video_process_->set_n_frames(my_movie_->length());
  vidl_vil1_movie::frame_iterator pframe = my_movie_->begin();
  vidl_vil1_movie::frame_iterator lframe = pframe;
  pframe += start_frame_;
  lframe += end_frame_+1;
  for ( ; pframe!=my_movie_->end() && pframe!=lframe && play_video_; ++pframe)
  {
    int frame_index = pframe->get_real_frame_index();
    vgui::out << "frame["<< frame_index <<"]\n";
    vil1_image img = pframe->get_image();
    itab0_->set_image(img);
    // pause by repeating the same frame
    if (pause_video_)
    {
      if (next_frame_)
      {
        if (pframe!=my_movie_->last()) ++pframe;
        next_frame_ = false;
      }
      if (prev_frame_)
      {
        if (pframe!=my_movie_->first()) --pframe;
        prev_frame_ = false;
      }
      // repeat the same frame by undoing the subsequent ++pframe of the iteration
      --pframe;
    }
    else if (video_process_)
    {
      video_process_->set_frame_index(frame_index);
      vil1_memory_image_of<unsigned char> image(img);
      video_process_->add_input_image(image);
      if (video_process_->execute())
      {
        if (video_process_->get_output_type()==vpro_video_process::SPATIAL_OBJECT)
          display_spatial_objects();
        else if (video_process_->get_output_type()==vpro_video_process::IMAGE)
          display_image();
        else if (video_process_->get_output_type()==vpro_video_process::TOPOLOGY)
          display_topology();
        if (video_process_->graph_flag())
        {
          vcl_vector<float> const& data = video_process_->graph();
          this->display_bargraph(data);
        }
      }
    }
    grid_->post_redraw();
    vgui::run_till_idle();
    this->save_display(frame_index);
  }

  if (video_process_)
    video_process_->finish();
  bargraph_= 0;
  easy1_->clear();
  if (save_display_)
    this->end_save_display();
  save_display_ = false;
}

void vvid_file_manager::play_video()
{
  play_video_ = true;
  pause_video_ = false;
  time_interval_ = 10.0;
  easy0_->clear_all();
  frame_trail_.clear();
  //return the display to the first frame after the play is finished

  if (cache_frames_)
  {
    this->cached_play();
    v2D0_->child.assign(tabs_[0]);
  }
  else
  {
    this->un_cached_play();
    if (!my_movie_)
      return;

    if (video_process_&&video_process_->get_output_image())
      itab1_->set_image(video_process_->get_output_image());
    else//don't display first video frame in process window, instead show nothing
#if 0
      itab1_->set_image(my_movie_->get_image(0));
#endif
    {
      vil1_image null;
      itab1_->set_image(null);
    }
  }
  this->post_redraw();
}

//Player control functions

//Stop the video and return to the first frame
void vvid_file_manager::stop_video()
{
  play_video_ = false;
}

//Cycle the play at the current frame
void vvid_file_manager::pause_video()
{
  pause_video_ =!pause_video_;
  //make the time interval longer for paused state
  time_interval_ = 50.0;
}

void vvid_file_manager::start_frame()
{
  vgui_dialog frame_dlg("Start Frame");
  frame_dlg.field("Frame No.", start_frame_);
  if (!frame_dlg.ask())
    return;
}

void vvid_file_manager::end_frame()
{
  vgui_dialog frame_dlg("End Frame");
  frame_dlg.field("Frame No.", end_frame_);
  if (!frame_dlg.ask())
    return;
}


//While the video is paused go to the next frame
void vvid_file_manager::next_frame()
{
  next_frame_ = true;
}

//While the video is paused go to the previous frame
void vvid_file_manager::prev_frame()
{
  prev_frame_ = true;
}

void vvid_file_manager::set_speed()
{
  //not implemented yet
}

void vvid_file_manager::easy2D_tableau_demo()
{
  int inc = 40;
  for (vcl_vector<bgui_vtol2D_tableau_sptr>::iterator eit = tabs_.begin();
       eit != tabs_.end(); eit++, ++inc)
  {
    (*eit)->clear();
    (*eit)->set_foreground(0,1,1);
    (*eit)->set_point_radius(5);
    if (inc>60)
      inc = 40;
    for (unsigned int j = 0; j<=height_; j+=inc)
      for (unsigned int k=0; k<=width_; k+=inc)
        (*eit)->add_point(k,j);
  }
}

void vvid_file_manager::no_op()
{
  video_process_ = 0;
}

void vvid_file_manager::difference_frames()
{
  static vpro_frame_diff_params fdp;
  vgui_dialog frame_diff_dialog("Frame_Diff Params");
  frame_diff_dialog.field("Display Scale Range", fdp.range_);
  if (!frame_diff_dialog.ask())
    return;
  video_process_ = new vpro_frame_diff_process(fdp);
}

void vvid_file_manager::compute_motion()
{
  static vpro_motion_params vmp;
  vgui_dialog motion_dialog("Motion Params");
  motion_dialog.field("Low Range", vmp.low_range_);
  motion_dialog.field("High Range", vmp.high_range_);
  motion_dialog.field("Smooth Sigma", vmp.smooth_sigma_);
  if (!motion_dialog.ask())
    return;

  video_process_ = new vpro_motion_process(vmp);
}

void vvid_file_manager::compute_lucas_kanade()
{
  static bool downsample = false;
  static int windowsize=2;
  static double thresh=20000;
  vgui_dialog downsample_dialog("Lucas-Kanade Params");
  downsample_dialog.checkbox("Downsample", downsample);
  downsample_dialog.field("WindowSize(2n+1) n=",windowsize);
  downsample_dialog.field("Motion Factor Threshold", thresh);

  if (!downsample_dialog.ask())
    return;
  video_process_ = new vpro_lucas_kanade_process(downsample,windowsize,thresh);
}

void vvid_file_manager::compute_harris_corners()
{
  static int track_window;
  static sdet_harris_detector_params hdp;
  vgui_dialog harris_dialog("harris");
  harris_dialog.field("sigma", hdp.sigma_);
  harris_dialog.field("thresh", hdp.thresh_);
  harris_dialog.field("N = 2n+1, (n)", hdp.n_);
  harris_dialog.field("Max No.Corners(percent)", hdp.percent_corners_);
  harris_dialog.field("scale_factor", hdp.scale_factor_);
  harris_dialog.checkbox("Tracks", track_);
  harris_dialog.field("Window", track_window);

  if (!harris_dialog.ask())
    return;

  video_process_ = new vpro_harris_corner_process(hdp);
  if (track_)
  {
    frame_trail_.clear();
    frame_trail_.set_window(track_window);
  }
}

void vvid_file_manager::compute_vd_edges()
{
  static int track_window;
  static bool agr = false;
  static sdet_detector_params dp;
  vgui_dialog det_dialog("Video Edges");
  det_dialog.field("Gaussian sigma", dp.smooth);
  det_dialog.field("Noise Threshold", dp.noise_multiplier);
  det_dialog.checkbox("Automatic Threshold", dp.automatic_threshold);
  det_dialog.checkbox("Agressive Closure", agr);
  det_dialog.checkbox("Compute Junctions", dp.junctionp);
  det_dialog.checkbox("Tracks", track_);
  det_dialog.field("Window", track_window);

  if (!det_dialog.ask())
    return;

  if (agr)
    dp.aggressive_junction_closure=1;
  else
    dp.aggressive_junction_closure=0;

  video_process_  = new vpro_edge_process(dp);
  if (track_)
  {
    frame_trail_.clear();
    frame_trail_.set_window(track_window);
  }
}

void vvid_file_manager::compute_line_fit()
{
  static bool agr = false;
  static sdet_detector_params dp;
  dp.borderp = false;
  static sdet_fit_lines_params flp;
  vgui_dialog line_dialog("Video Line Segments");
  line_dialog.field("Gaussian sigma", dp.smooth);
  line_dialog.field("Noise Threshold", dp.noise_multiplier);
  line_dialog.checkbox("Automatic Threshold", dp.automatic_threshold);
  line_dialog.checkbox("Agressive Closure", agr);
  line_dialog.checkbox("Compute Junctions", dp.junctionp);
  line_dialog.field("Min Fit Length", flp.min_fit_length_);
  line_dialog.field("RMS Distance", flp.rms_distance_);
  if (!line_dialog.ask())
    return;

  if (agr)
    dp.aggressive_junction_closure=1;
  else
    dp.aggressive_junction_closure=0;

  video_process_  = new vpro_edge_line_process(dp, flp);
}

void vvid_file_manager::compute_grid_match()
{
  static bool agr = false;
  static sdet_detector_params dp;
  dp.borderp = false;
  static sdet_fit_lines_params flp;
  static sdet_grid_finder_params gfp;
//dp.automatic_threshold=true;
  dp.noise_multiplier=4;
  flp.min_fit_length_=10;
  flp.rms_distance_=0.05;
  gfp.angle_tol_ = 3.0;
  vgui_dialog grid_dialog("Grid Match");
  grid_dialog.field("Gaussian sigma", dp.smooth);
  grid_dialog.field("Noise Threshold", dp.noise_multiplier);
  grid_dialog.checkbox("Automatic Threshold", dp.automatic_threshold);
  grid_dialog.checkbox("Agressive Closure", agr);
  grid_dialog.checkbox("Compute Junctions", dp.junctionp);
  grid_dialog.field("Min Fit Length", flp.min_fit_length_);
  grid_dialog.field("RMS Distance", flp.rms_distance_);
  grid_dialog.field("Angle Tolerance", gfp.angle_tol_);
  grid_dialog.field("Line Count Threshold", gfp.thresh_);
//grid_dialog.checkbox("Debug Output", gfp.verbose_);

  if (!grid_dialog.ask())
    return;

  if (agr)
    dp.aggressive_junction_closure=1;
  else
    dp.aggressive_junction_closure=0;

  video_process_  = new vpro_grid_finder_process(dp, flp, gfp);
}

void vvid_file_manager::compute_corr_tracking()
{
  static strk_tracker_params tp;
  vgui_dialog tracker_dialog("Correlation Tracker ");
  tracker_dialog.field("Number of Samples", tp.n_samples_);
  tracker_dialog.field("Search Radius", tp.search_radius_);
  tracker_dialog.field("Smooth Sigma", tp.sigma_);
  if (!tracker_dialog.ask())
    return;
  vcl_cout << tp << '\n';
  vtol_topology_object_sptr to = easy0_->get_temp();
  if (!to)
    vcl_cout << "In vvid_file_manager::compute_corr_tracking() - no model\n";
  else
  {
    video_process_ = new strk_corr_tracker_process(tp);
    video_process_->add_input_topology_object(to);
  }
}

void vvid_file_manager::compute_info_tracking()
{
  static bool output_track = false;
  static bool output_hist = false;
  static bool display_hist = false;
  static strk_info_tracker_params tp;
  vgui_dialog tracker_dialog("Mutual Information Tracker V1.6");
  tracker_dialog.field("Number of Samples", tp.n_samples_);
  tracker_dialog.field("Search Radius", tp.search_radius_);
  tracker_dialog.field("Angle Range (radians)", tp.angle_range_);
  tracker_dialog.field("Scale Range (1+-s)", tp.scale_range_);
  tracker_dialog.field("Smooth Sigma", tp.sigma_);
  tracker_dialog.field("Min Gradient Mag", tp.min_gradient_);
  tracker_dialog.field("Parzen Sigma", tp.parzen_sigma_);
  tracker_dialog.checkbox("Model Background", tp.use_background_);
  tracker_dialog.checkbox("Add Gradient Info", tp.gradient_info_);
  tracker_dialog.checkbox("Add Color Info", tp.color_info_);
  tracker_dialog.checkbox("Renyi Joint Entropy", tp.renyi_joint_entropy_);
  tracker_dialog.checkbox("Output Track Data", output_track);
  tracker_dialog.checkbox("Output Feature Data", output_hist);
  tracker_dialog.checkbox("Display Histograms", display_hist);
  tracker_dialog.checkbox("Verbose", tp.verbose_);
  tracker_dialog.checkbox("Debug", tp.debug_);
  if (!tracker_dialog.ask())
    return;
  static vcl_string track_file, hist_file;
  if (output_track)
  {
    vgui_dialog output_dialog("Track Data File");
    static vcl_string ext = "*.*";
    output_dialog.file("Track File:", ext, track_file);
    if (!output_dialog.ask())
      return;
  }
  if (output_hist)
  {
    vgui_dialog hist_dialog("Feature Data File");
    static vcl_string ext = "*.*";
    hist_dialog.file("Feature File:", ext, hist_file);
    if (!hist_dialog.ask())
      return;
  }
  vcl_cout << tp << '\n';
  vtol_topology_object_sptr to = easy0_->get_temp();
  if (!to)
    vcl_cout << "In vvid_file_manager::compute_info_tracking() - no model\n";
  else
  {
    strk_info_tracker_process* vitp = new strk_info_tracker_process(tp);
    video_process_ = vitp;
    video_process_->add_input_topology_object(to);
    if (tp.use_background_) {
      if (!background_model_)
        vcl_cout << "In vvid_file_manager::compute_info_tracking() -"
                 << " no background model\n";
      else
        video_process_->add_input_topology_object(background_model_->cast_to_topology_object());
    }
    if (output_track)
      if (!vitp->set_track_output_file(track_file))
        return;
    if (output_hist)
      if (!vitp->set_hist_output_file(hist_file))
        return;
    if (display_hist)
    {
      vgui_easy2D_tableau_sptr easy;
      easy.vertical_cast(easy1_);
      bargraph_ = new bgui_bargraph_clipon_tableau(easy);
      bargraph_->set_color_vector(vitp->color_index());
    }
  }
}

void vvid_file_manager::save_display(int /* frame */)
{
  if (!save_display_)
    return;

  if (display_frame_skip_) {
    if (skip_counter_>0)
    {
      --skip_counter_;
      return;
    }
    else
      skip_counter_ = display_frame_skip_;
  }

  if (!overlay_pane_)
  {
    vil1_image image = itab1_->get_image();
    if (!image)
      return;
    display_output_frames_.push_back(image);
    return;
  }

  vil1_memory_image_of<vil1_rgb<unsigned char> >
    temp = vgui_utils::colour_buffer_to_image();
  for (int i = 0; i<display_frame_repeat_; i++)
    display_output_frames_.push_back(temp);
}

void vvid_file_manager::display_poly_track()
{
  start_frame_ = 0;
  if (my_movie_)
    end_frame_ = my_movie_->length()-1;
  else
    end_frame_=0;
  vgui_dialog output_dialog("Track Data File(sets start frame)");
  static vcl_string track_file;
  static vcl_string trk_ext = "trk", out_ext = "out";
  output_dialog.file("Track File:", trk_ext, track_file);
  if (!output_dialog.ask())
    return;
  strk_track_display_process* vtd = new strk_track_display_process();
  video_process_ = vtd;
  vtd->set_input_file(track_file);
  start_frame_ = vtd->start_frame();
  end_frame_ = vtd->end_frame();
}

void vvid_file_manager::generate_basis_sequence()
{
  vgui_dialog output_dialog("Image Basis Generator");
  static vcl_string basis_file;
  static vcl_string ext = "*.*";
  output_dialog.file("Basis File:", ext, basis_file);
  if (!output_dialog.ask())
    return;
  video_process_ = new vpro_basis_generator_process(basis_file);
}

void vvid_file_manager::compute_fourier_transform()
{
  static vpro_fourier_params vfp;
  vgui_dialog fourier_dialog("Fourier Params");
  fourier_dialog.field("Display Scale Range", vfp.range_);
  if (!fourier_dialog.ask())
    return;
  video_process_ = new vpro_fourier_process(vfp);
}

void vvid_file_manager::spatial_filter()
{
  static vpro_spatial_filter_params vsfp;
  vgui_dialog spatial_filter_dialog("Spatial_Filter Params");
  spatial_filter_dialog.field("X Dir", vsfp.dir_fx_);
  spatial_filter_dialog.field("Y Dir", vsfp.dir_fy_);
  spatial_filter_dialog.field("Center Freq", vsfp.f0_);
  spatial_filter_dialog.field("Filter Radius", vsfp.radius_);
  spatial_filter_dialog.checkbox("Show Filtered Fourier Magnitude",
                                 vsfp.show_filtered_fft_);
  if (!spatial_filter_dialog.ask())
    return;
  video_process_ = new vpro_spatial_filter_process(vsfp);
}

void vvid_file_manager::create_box()
{
  rubber0_->rubberband_box();
}

void vvid_file_manager::create_polygon()
{
  rubber0_->rubberband_polygon();
}

void vvid_file_manager::start_save_display()
{
  save_display_ = true;
  skip_counter_ = display_frame_skip_;
  display_output_frames_.clear();
  vgui_dialog output_dialog("Display Movie Output");
  static vcl_string ext = "avi";
  output_dialog.file("Movie File:", ext, display_output_file_);
  output_dialog.field("# Frame Repeats ", display_frame_repeat_);
  output_dialog.field("# Frame Skips ", display_frame_skip_);
  output_dialog.checkbox("Save Overlay Pane (left) (or Image Pane (right))", overlay_pane_);
  if (!output_dialog.ask())
    return;
}

void vvid_file_manager::end_save_display()
{
  if (!save_display_||!display_output_frames_.size())
    return;
  save_display_ = false;
  vidl_vil1_clip_sptr clip = new vidl_vil1_clip(display_output_frames_);
  vidl_vil1_movie_sptr mov= new vidl_vil1_movie();
  mov->add_clip(clip);
  vidl_vil1_io::save(mov.ptr(), display_output_file_.c_str(), "AVI");
  display_output_frames_.clear();
}

void vvid_file_manager::create_stem()
{
  vcl_cout << "Make the stem ...\n";
  art_model_ = 0;
  vtol_topology_object_sptr to = easy0_->get_temp();
  if (!to)
  {
    vcl_cout << "Failed\n";
    return;
  }
  else
    vcl_cout << "Stem complete\n";
  stem_ = to->cast_to_face()->cast_to_face_2d();
}

void vvid_file_manager::create_long_arm_tip()
{
  vcl_cout << "Make the long_arm_tip ...\n";
  art_model_ = 0;
  vtol_topology_object_sptr to = easy0_->get_temp();
  if (!to)
  {
    vcl_cout << "Failed\n";
    return;
  }
  else
    vcl_cout << "long_arm_tip complete\n";
  long_tip_ = to->cast_to_face()->cast_to_face_2d();
}

void vvid_file_manager::create_short_arm_tip()
{
  vcl_cout << "Make the short_arm_tip ...\n";
  art_model_ = 0;
  vtol_topology_object_sptr to = easy0_->get_temp();
  if (!to)
  {
    vcl_cout << "Failed\n";
    return;
  }
  else
    vcl_cout << "short_arm_tip complete\n";
  short_tip_ = to->cast_to_face()->cast_to_face_2d();
}

void vvid_file_manager::exercise_art_model()
{
  if (!stem_||!long_tip_||!short_tip_)
  {
    vcl_cout << "Not enough components to make the art model\n";
    return;
  }

  static bool refresh_model = false;
  static double stem_tx =0, stem_ty =0, stem_angle =0;
  static double long_arm_pivot_angle = 0, short_arm_pivot_angle =0;
  static double long_tip_angle = 0, short_tip_angle = 0;

  vgui_dialog trans_art_dialog("Transform Art Model");
  trans_art_dialog.field(" Stem Tx", stem_tx);
  trans_art_dialog.field(" Stem Ty", stem_ty);
  trans_art_dialog.field(" Stem Angle", stem_angle);
  trans_art_dialog.field("Long Arm Pivot Angle",long_arm_pivot_angle);
  trans_art_dialog.field("Short Arm Pivot Angle",short_arm_pivot_angle);
  trans_art_dialog.field("Long Tip Angle", long_tip_angle);
  trans_art_dialog.field("Short Tip Angle", short_tip_angle);
  trans_art_dialog.checkbox("Refresh Model", refresh_model);
  if (!trans_art_dialog.ask())
    return;

  if (refresh_model || !art_model_)
  {
    vcl_vector<vtol_face_2d_sptr> faces;
    vsol_point_2d_sptr pivot = btol_face_algs::centroid(stem_);
    faces.push_back(stem_);
    faces.push_back(long_tip_);
    faces.push_back(short_tip_);
    vil1_image img = itab0_->get_image();
    vil1_memory_image_of<float> image =
      brip_vil1_float_ops::convert_to_float(img);
    art_model_ = new strk_art_info_model(faces, pivot, image);

    vcl_vector<vtol_face_2d_sptr> vtol_faces = art_model_->vtol_faces();
    easy0_->clear();
    easy0_->add_faces(vtol_faces);
  }

  art_model_ = new strk_art_info_model(art_model_);//to simulate generation
  art_model_->transform(stem_tx, stem_ty, stem_angle, long_arm_pivot_angle,
                        short_arm_pivot_angle, long_tip_angle,
                        short_tip_angle);
  easy0_->clear();
  vcl_vector<vtol_face_2d_sptr> new_faces = art_model_->vtol_faces();
  easy0_->add_faces(new_faces);
  easy0_->post_redraw();
}


void vvid_file_manager::track_art_model()
{
  if (!stem_||!long_tip_||!short_tip_)
  {
    vcl_cout << "Not enough components to construct model\n";
    return;
  }
  vcl_vector<vtol_topology_object_sptr> faces;
  faces.push_back(stem_->cast_to_face());
  faces.push_back(long_tip_->cast_to_face());
  faces.push_back(short_tip_->cast_to_face());

  static bool output_track = false;
  static strk_info_model_tracker_params imtp;
  vgui_dialog trans_art_dialog("Articulated Model Tracking");
  trans_art_dialog.field(" Number of Samples ", imtp.n_samples_);
  trans_art_dialog.field(" Stem Translation Radius ", imtp.stem_trans_radius_);
  trans_art_dialog.field(" Stem Angle Range", imtp.stem_angle_range_);
  trans_art_dialog.field("Long Arm Pivot Angle Range",
                         imtp.long_arm_angle_range_);
  trans_art_dialog.field("Short Arm Pivot Angle Range",
                         imtp.short_arm_angle_range_);
  trans_art_dialog.field("Long Arm Tip Angle Range",
                         imtp.long_arm_tip_angle_range_);
  trans_art_dialog.field("Short Arm Tip Angle Range",
                         imtp.short_arm_tip_angle_range_);
  trans_art_dialog.checkbox("Compute Gradient Info", imtp.gradient_info_);
  trans_art_dialog.checkbox("Output Track Data", output_track);
  trans_art_dialog.checkbox("Output Debug Messages", imtp.verbose_);
  if (!trans_art_dialog.ask())
    return;
  static vcl_string track_file;
  if (output_track)
  {
    vgui_dialog output_dialog("Track Data File");
    static vcl_string ext = "*.*";
    output_dialog.file("Track File:", ext, track_file);
    if (!output_dialog.ask())
      return;
  }
  vcl_cout << imtp << '\n';
  strk_info_model_tracker_process* imitp =
    new strk_info_model_tracker_process(imtp);
  video_process_ = imitp;
  video_process_->add_input_topology(faces);
  if (output_track)
    if (!imitp->set_output_file(track_file))
      return;
}

void vvid_file_manager::display_art_model_track()
{
  vgui_dialog output_dialog("Track Data File");
  static vcl_string track_file;
  static vcl_string trk_ext = "trk", out_ext = "out";
  output_dialog.file("Track File:", trk_ext, track_file);
  if (!output_dialog.ask())
    return;
  strk_art_model_display_process* vtd = new strk_art_model_display_process();
  video_process_ = vtd;
  vtd->set_input_file(track_file);
}


void vvid_file_manager::display_ihs()
{
  video_process_  = new vpro_ihs_process();
}

void vvid_file_manager::create_background_model()
{
  vcl_cout << "Make the background model polygon ...\n";
  background_model_ = 0;
  vtol_topology_object_sptr to = easy0_->get_temp();
  if (!to)
  {
    vcl_cout << "Failed\n";
    return;
  }
  else
    vcl_cout << "Background model complete\n";
  background_model_ = to->cast_to_face()->cast_to_face_2d();
}

void vvid_file_manager::save_frame()
{
  vil1_image img = itab0_->get_image();
  if (!img)
  {
    vcl_cout << "In vvid_file_manager::save_frame() - no image\n";
    return;
  }
  vgui_dialog file_dialog("Frame Image File");
  static vcl_string image_file;
  static vcl_string ext = "tif";
  file_dialog.file("Track File:", ext, image_file);
  if (!file_dialog.ask())
    return;
  if (!vil1_save(img, image_file.c_str()))
    vcl_cout << "In vvid_file_manager::save_frame() - save failed\n";
}

void vvid_file_manager::save_half_res()
{
  vgui_dialog file_dialog("Half Resolution Output File");
  static vcl_string video_file;
  static vcl_string ext = "avi";
  file_dialog.file("Video File:", ext, video_file);
  if (!file_dialog.ask())
    return;
  vpro_half_res_process* hrp = new vpro_half_res_process(video_file);
  video_process_ = hrp;
  this->play_video();
}

void vvid_file_manager::create_c_and_g_tracking_face()
{
  easy0_->clear_all();
  static bool show_region_boxes = false;
  static sdet_vehicle_finder_params vfp;
  vgui_dialog vehicle_finder_dialog("Click and Go(Not Ready for Prime Time!)");
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
  vil1_image image = itab0_->get_image();
  sdet_vehicle_finder vf(vfp);
  vf.set_image(image);
  //User picks point
  vcl_cout << "\nPick vehicle...\n";
  float x0=0, y0=0;
  picktab0_->pick_point(&x0,&y0);
  vcl_cout << "Pick(" << x0 << ' ' << y0 << ")\n";
  //Display the search box
  vf.set_pick((int)x0, (int)y0);
  vsol_box_2d_sptr box = vf.search_box();
  vsol_polygon_2d_sptr poly = bsol_algs::poly_from_box(box);
  easy0_->add_vsol_polygon_2d(poly);
  //Get the tracking face
  if (!vf.detect_vehicle())
  {
    vcl_cout << "Vehicle Detection Failed\n";
    return;
  }
  vtol_face_2d_sptr f = vf.vehicle_track_face();
  easy0_->set_temp(f->cast_to_topology_object());
  easy0_->add_face(f);
  easy0_->post_redraw();
}
// Display an image of tracked histogram data
void vvid_file_manager::display_tracked_hist_data()
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
  easy1_->get_image_tableau()->set_image(image);
  easy1_->post_redraw();
  vgui::run_till_idle();
}

// capture histogram data from a prestored track
void vvid_file_manager::capture_feature_data()
{
  vgui_dialog file_dialog("Feature Capture From Track");
  static vcl_string track_file;
  static vcl_string hist_file;
  static vcl_string trk_ext = "trk", hist_ext = "hist";
  file_dialog.file("Track File(input):", trk_ext, track_file);
  file_dialog.file("Hist File(output):", hist_ext, hist_file);
  if (!file_dialog.ask())
    return;
  strk_info_tracker_params itp;
  itp.color_info_ = true;
  strk_feature_capture_process* fcp = new strk_feature_capture_process(itp);
  video_process_ = fcp;
  fcp->set_input_file(track_file);
  fcp->set_output_file(hist_file);
  start_frame_ = fcp->start_frame();
  end_frame_ = fcp->end_frame();
}
// capture a set of image snippets enclosing the vehicle in each frame
void vvid_file_manager::capture_snippets()
{
  vgui_dialog file_dialog("Snippet Capture From Track");
  static vcl_string track_file;
  static vcl_string snip_dir;
  static vcl_string trk_ext = "trk", snip_ext = "";
  static double frac = 0.1;
  file_dialog.file("Track File(input):", trk_ext, track_file);
  file_dialog.file("Snippet Directory(output):", snip_ext, snip_dir);
  file_dialog.field("Snippet ROI Margin (fraction of Bounding Box diameter)", frac);
  if (!file_dialog.ask())
    return;
  strk_snippet_extractor_process* sep = new strk_snippet_extractor_process();
  video_process_ = sep;
  sep->set_margin_fraction(frac);
  sep->set_input_file(track_file);
  sep->set_snippet_directory(snip_dir);
  start_frame_ = sep->start_frame();
  end_frame_ = sep->end_frame();
}
