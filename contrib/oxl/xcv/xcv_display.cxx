//
// This is xcv/xcv_display.cxx
// See xcv_display.h for a description of this file.
//
// \author K.Y.McGaul
// \verbatim
//  Modifications:
//    K.Y.McGaul  27-JAN-2000    Initial version.
// \endverbatim
//

#include <vcl_cstdio.h> // sprintf()
#include <vcl_string.h>
#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <vcl_cmath.h>

#include <vil/vil_memory_image_of.h>
#include <vil/vil_image_as.h>
#include <vnl/vnl_math.h>

#include <vgui/vgui.h>
#include <vgui/vgui_menu.h>
#include <vgui/vgui_window.h>
#include <vgui/vgui_tableau.h>
#include <vgui/vgui_adaptor.h>
#include <vgui/vgui_dialog.h>
#include <vgui/vgui_enhance.h>
#include <vgui/vgui_easy2D.h>
#include <vgui/vgui_viewer2D.h>
#include <vgui/vgui_composite.h>
#include <vgui/vgui_rubberbander.h>
#include <vgui/vgui_shell_tableau.h>
#include <vgui/vgui_text_tableau.h>
#include <vgui/vgui_roi_tableau.h>
#include <vgui/vgui_event_server.h>

#include <xcv/xcv_image_tableau.h>
#include <xcv/xcv_picker_tableau.h>
#include <xcv/xcv_axes_tableau.h>
#include <xcv/xcv_display.h>

//vgui_roi_tableau_make_roi *xcv_display::roi_tableau_client_ = 0;
extern void post_to_status_bar(const char*);
extern void get_current(unsigned*, unsigned*);
extern vcl_vector<xcv_image_tableau_sptr> get_image_list();
extern vgui_easy2D_sptr get_easy2D_at(unsigned, unsigned);
extern xcv_picker_tableau_sptr get_picker_tableau_at(unsigned, unsigned);
extern vgui_composite_sptr get_composite_at(unsigned, unsigned);
extern vgui_viewer2D_sptr get_viewer2D_at(unsigned, unsigned);
extern xcv_image_tableau_sptr get_image_tableau_at(unsigned, unsigned);
extern vgui_rubberbander_sptr get_rubberbander_at(unsigned, unsigned);
extern vgui_tableau_sptr get_top(unsigned,unsigned);
extern bool get_image_at(vil_image*, unsigned, unsigned);


static bool debug = true;
static bool is_magnifying = false;
static bool is_enhancing = false;
static vgui_composite_sptr comp;
static vgui_enhance_sptr enhance;
static xcv_image_tableau_sptr img;
static vgui_easy2D_sptr easy;

//-----------------------------------------------------------------------------
//: Centre the current image inside the frame.
//-----------------------------------------------------------------------------
void xcv_display::centre_image()
{
  unsigned col, row;
  get_current(&col, &row);
  vgui_viewer2D_sptr view = get_viewer2D_at(col, row);

  vil_image image;
  if (get_image_at(&image, col, row))
    //view->center_image(image.width(), image.height());
    view->center_event();
}

//-----------------------------------------------------------------------------
//: Turn the enhance lens on/off.
//  This is done by slotting in an enhance tableau above the image tableau.
//-----------------------------------------------------------------------------
void xcv_display::toggle_enhance()
{
  if (is_magnifying)
    toggle_magnify();

  if (is_enhancing == false)
  {
    unsigned col, row;
    get_current(&col, &row);
    img = get_image_tableau_at(col, row);
    if (!img) return;
    easy = get_easy2D_at(col, row);
    if (!easy) return;

    // Get the full list of images and get the user to select one:
    vcl_vector<xcv_image_tableau_sptr> img_tabs = get_image_list();
    static int selected_image = 1;
    vcl_vector<vcl_string> labels;
    for (unsigned int i=0; i<img_tabs.size(); i++)
      labels.push_back(img_tabs[i]->file_name());
    vgui_dialog dl("Images");
    dl.choice("Enhance image:", labels, selected_image);
    if (!dl.ask())
      return;

    // Replace the image with an enhance tableau containing the same image:
    enhance = vgui_enhance_new(img, img_tabs[selected_image]);
    easy->set_child(enhance);

    is_enhancing = true;

    char msg[100];
    sprintf(msg, "Enhance lens added to position (%d, %d).", col, row);
    post_to_status_bar(msg);
  }
  else
  {
    if(debug) vcl_cout << "removing enhance lens" << vcl_endl;

    vgui_slot::replace_child_everywhere(enhance, img);
    is_enhancing = false;
  }
}

//-----------------------------------------------------------------------------
//: Turn the magnify glass on/off.
//  This is done by slotting in an enhance tableau above the image tableau.
//-----------------------------------------------------------------------------
void xcv_display::toggle_magnify()
{
  if (is_enhancing)
    toggle_enhance();

  if (is_magnifying == false)
  {
    if (debug) vcl_cerr << "adding magnify lens" << vcl_endl;

    unsigned col, row;
    get_current(&col, &row);
    comp = get_composite_at(col, row);
    if (!comp) return;

    enhance = vgui_enhance_new();
    vgui_slot::replace_child_everywhere(comp, enhance);
    enhance->set_child(comp);

    is_magnifying = true;
  }
  else
  {
    if(debug) vcl_cout << "removing magnify lens" << vcl_endl;

    enhance->set_child(0);
    vgui_slot::replace_child_everywhere(enhance, comp);
    enhance = vgui_enhance_sptr(); //0;
    is_magnifying = false;
  }
}
//-----------------------------------------------------------------------------
//: Define a region of interest
//-----------------------------------------------------------------------------
void xcv_display::make_roi()
{
  unsigned col, row;

  get_current(&col, &row);
  xcv_image_tableau_sptr imt = get_image_tableau_at(col,row);
  vgui_rubberbander_sptr rubber = get_rubberbander_at(col, row);
  if(!rubber)
    return;

  //roi_tableau_client_ = new vgui_roi_tableau_make_roi(imt);
  vgui_roi_tableau_make_roi roi_tableau_client_(imt);

  vgui_event_server *es = new vgui_event_server(imt);
  vgui_rubberbander_client* old_client = rubber->get_client();  // save to put back in later
  rubber->set_client(&roi_tableau_client_);
  rubber->rubberband_box();
  while(!roi_tableau_client_.is_done())
    es->next();
  es = 0;
  rubber->set_client(old_client);
  //roi_tableau_client_ = 0;
}

//-----------------------------------------------------------------------------
//: Remove the region of interest
//-----------------------------------------------------------------------------
void xcv_display::remove_roi()
{
  unsigned col, row;
  get_current(&col,&row);
  vgui_rubberbander_sptr rubber = get_rubberbander_at(col, row);

  //if(rubber)
  //{
    //rubber->set_client(0);
    //delete roi_tableau_client_;
    //roi_tableau_client_ = 0;
  //}
  xcv_image_tableau_sptr imt = get_image_tableau_at(col,row);
  imt->unset_roi();
}

//-----------------------------------------------------------------------------
//: Generates intensity values along a line.
//  Given an image, a start point and an end point (both
//  in image coordinates), this method generates two arrays representing
//  the coordinates of the image pixels in the slice, and one array
//  containing the intensity values along the line.
//-----------------------------------------------------------------------------
void xcv_display::line_profile(const vil_image& src, float x0, float y0, float x1,float y1,
  int num_points, float* xvals, float* yvals, float* ivals)
{
  int sdepth = src.bits_per_component();

  if ((sdepth != 8) /*&& (sdepth != 16)*/)
  {
    vcl_cerr << sdepth << " bit profile not yet implemented." << vcl_endl;
    return;
  }

  float x_step = (x1 - x0)/(num_points-1);
  float y_step = (y1 - y0)/(num_points-1);

  // copy input image to byte buffer
  //vil_memory_image_of<vil_byte> memimg(src);
  vil_memory_image_of<vil_byte> memimg;
  memimg.resize(src.width(), src.height());
  vil_image_as_byte(src).get_section(memimg.get_buffer(), 0, 0, src.width(), src.height());

  for(int i = num_points-1; i>=0; i--)
  {
    xvals[i] = x0 + x_step*i;
    yvals[i] = y0 + y_step*i;

    if (sdepth == 8)
    {
      float imgval = (float)memimg(vnl_math_rnd(xvals[i]), vnl_math_rnd(yvals[i]));
      ivals[i] = imgval;
    }
#if 0
    else if (sdepth == 16)
      ivals[i]
        = (int)(*(unsigned short*)(memimg(vnl_math_rnd(xvals[i]), vnl_math_rnd(yvals[i]))));
#endif
  }
}

//-----------------------------------------------------------------------------
//: Show a profile of the image intensity on a user-selected line.
//-----------------------------------------------------------------------------
void xcv_display::show_line_slice()
{
  unsigned col, row;
  get_current(&col, &row);
  xcv_picker_tableau_sptr picker = get_picker_tableau_at(col, row);
  if (!picker) return;

  float fx0,fy0,fx1,fy1;
  picker->pick_line(&fx0, &fy0, &fx1, &fy1);

  vil_image img;
  if (!get_image_at(&img, col, row))
    return;

  int num_points
    = (int)(vnl_math_rnd(vcl_sqrt((double)((fx1-fx0)*(fx1-fx0)+ (fy1-fy0)*(fy1-fy0)))));

  // Avoid divide by zero
  if(!num_points) num_points = 10;

  float* x = new float[num_points+1];
  float* y = new float[num_points+1];
  float* val = new float[num_points+1];

  line_profile(img, fx0, fy0, fx1, fy1, num_points+1, x, y, val);

  char tmp_heading[200];
  sprintf(tmp_heading, "Image Intensity Profile across (%.0f, %.0f) to (%.0f, %.0f)",
    fx0, fy0, fx1, fy1);

  xcv_axes_tableau_new axes(tmp_heading, "X", "Intensity");
  for (int i=0; i<num_points+1; i++)
    axes->add_point(x[i], val[i]);
  axes->compute_axes();
  vgui_viewer2D_new viewer(axes);

  vgui_dialog profile_dialog("Image Line Profile");
  profile_dialog.inline_tableau(vgui_shell_tableau_new(viewer), 700, 500);
  profile_dialog.set_ok_button("close");
  profile_dialog.set_cancel_button(0);
  profile_dialog.ask();
  /*
  vgui_window *popup = vgui::produce_window(700, 500, "Image Line Profile");
  if (popup)
  {
    popup->set_statusbar(false);
    popup->get_adaptor()->set_tableau(vgui_shell_tableau_new(viewer));
    popup->show();
  }
  */
}

//-----------------------------------------------------------------------------
//: Creates a menu containing all the functions in this file.
//-----------------------------------------------------------------------------
vgui_menu xcv_display::create_display_menu()
{
  vgui_menu display_menu;
  //display_menu.add("Centre image", centre_image);
  //display_menu.separator();
  display_menu.add("Toggle magnify lens", toggle_magnify);
  display_menu.add("Toggle enhance lens", toggle_enhance);
  display_menu.separator();
  display_menu.add("Make ROI",make_roi);
  display_menu.add("Remove ROI",remove_roi);
  display_menu.add("Show line slice", show_line_slice);
  return display_menu;
}

