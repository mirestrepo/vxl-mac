#include "bwm_tableau_img.h"
#include "bwm_tableau_text.h"
#include "bwm_command_macros.h"
#include "bwm_observer_mgr.h"
#include "bwm_tableau_mgr.h"
#include "bwm_popup_menu.h"
#include <vsl/vsl_binary_io.h>
#include <vsl/vsl_vector_io.h>
#include <vil/vil_save.h>
#include <vsol/vsol_point_2d.h>
#include <vsol/vsol_box_2d.h>

#include <vgui/vgui_dialog.h>
#include <vgui/vgui_viewer2D_tableau.h>
#include <vgui/vgui_shell_tableau.h>
#include <vgui/vgui_command.h>

  
void bwm_tableau_img::get_popup(vgui_popup_params const &params, vgui_menu &menu)
{
  menu.clear();

  bwm_popup_menu pop(this);
  pop.get_menu(menu);
}
void bwm_tableau_img::lock()
{
  my_observer_->image_tableau()->lock_linenum(true);
  my_observer_->lock_vgui_status(true);
}

void bwm_tableau_img::unlock()
{
  my_observer_->lock_vgui_status(false);
  if(!my_observer_->vgui_status_on())
    my_observer_->image_tableau()->lock_linenum(false);
}

void bwm_tableau_img::create_box()
{
  // first lock the bgui_image _tableau
  this->lock();
  set_color(1, 0, 0);
  float x1=0, y1=0, x2=0, y2=0;
  pick_box(&x1, &y1, &x2, &y2);
  vsol_box_2d_sptr box2d = new vsol_box_2d();
  box2d->add_point(x1, y1);
  box2d->add_point(x2, y2);
  this->unlock();
  // add the box to the list
  my_observer_->create_box(box2d);
}

void bwm_tableau_img::create_polygon()
{
  // first lock the bgui_image _tableau
  bwm_observer_mgr::instance()->stop_corr();
  this->lock();
  vsol_polygon_2d_sptr poly2d;
  set_color(1, 0, 0);
  pick_polygon(poly2d);
  if (!poly2d)
    {
      vcl_cerr << "In bwm_tableau_img::create_polygon() - picking failed\n";
      return;
    }
  this->unlock();

  // add the polygon to the list
  my_observer_->create_polygon(poly2d);
}

void bwm_tableau_img::create_polyline()
{
  // first lock the bgui_image _tableau
  this->lock();
  bwm_observer_mgr::instance()->stop_corr();

  vsol_polyline_2d_sptr poly2d;
  set_color(1, 0, 0);
  this->pick_polyline(poly2d);
  if (!poly2d)
    {
      vcl_cerr << "In bwm_tableau_img::create_polyline() - picking failed\n";
      return;
    }

  this->unlock();
  // add the polygon to the list
  my_observer_->create_polyline(poly2d);
}

void bwm_tableau_img::create_point()
{
  float x, y;

  set_color(1, 0, 0);
  this->pick_point(&x, &y);
  my_observer_->create_point(new vsol_point_2d(x, y));
}

void bwm_tableau_img::create_pointset()
{
  vcl_vector<vsol_point_2d_sptr> pts;
  bool picked = this->pick_point_set(pts, 10);
  for (vcl_vector<vsol_point_2d_sptr>::iterator pit = pts.begin();
       pit != pts.end(); ++pit)
    my_observer_->create_point(*pit);
  this->post_redraw();
}

void bwm_tableau_img::copy()
{
  my_observer_->copy();
}

void bwm_tableau_img::paste()
{
  float x, y;
  this->pick_point(&x, &y);
  my_observer_->paste(x, y);
}

void bwm_tableau_img::deselect_all()
{
  my_observer_->deselect_all();
}

void bwm_tableau_img::clear_poly()
{
  my_observer_->delete_selected();
}

void bwm_tableau_img::clear_box()
{
  my_observer_->clear_box();
}

void bwm_tableau_img::clear_all()
{
  my_observer_->delete_all();
}

void bwm_tableau_img::intensity_profile()
{
  float x1, y1, x2, y2;
  this->lock();
  pick_line(&x1, &y1, &x2, &y2);
  vcl_cout << x1 << ',' << y1 << "-->" << x2 << ',' << y2 << vcl_endl;
  my_observer_->intensity_profile(x1, y1, x2, y2);
  this->unlock();
}

void bwm_tableau_img::range_map()
{
  my_observer_->range_map();
}

void bwm_tableau_img::toggle_show_image_path()
{
  my_observer_->toggle_show_image_path();
}

void bwm_tableau_img::zoom_to_fit()
{
  my_observer_->zoom_to_fit();
}

void bwm_tableau_img::scroll_to_point()
{
  my_observer_->scroll_to_point();
}

void bwm_tableau_img::save_mask()
{
  vil_image_view_base_sptr mask = my_observer_->mask();
  if(!mask)
    return;
  vgui_dialog save_dlg("Save Mask");
  vcl_string ext, file_path;
  save_dlg.file("Mask Filename", ext, file_path);
  if(!save_dlg.ask())
    return;
  if(file_path =="")
    return;
  bool result = vil_save(*mask,file_path.c_str());
  if( !result ) {
    vcl_cerr << "Failed to save image to" << file_path << vcl_endl;
  }
}

void bwm_tableau_img::save_spatial_objects_2d()
{
  vcl_vector<vsol_spatial_object_2d_sptr> sos = 
    my_observer_->get_spatial_objects_2d();
  if(!sos.size())
    return;
  vgui_dialog save_dlg("Save Spatial Objects");
  vcl_string ext, binary_filename;
  save_dlg.file("Binary Filename", ext, binary_filename);
  if(!save_dlg.ask())
    return;
  if(binary_filename == "")
    return;
  vsl_b_ofstream ostr(binary_filename);
  if(!ostr){
    vcl_cerr << "Failed to open output stream "
             << binary_filename << vcl_endl;
    return;
  }
  vsl_b_write(ostr, sos);
}


void bwm_tableau_img::help_pop()
{
bwm_tableau_text* text = new bwm_tableau_text(500, 500);

text->set_text("C:\\lems\\lemsvxlsrc\\lemsvxlsrc\\contrib\\bwm\\doc\\doc\\HELP_cam.txt");
vgui_tableau_sptr v = vgui_viewer2D_tableau_new(text);
vgui_tableau_sptr s = vgui_shell_tableau_new(v);
vgui_dialog popup("CAMERA TABLEAU HELP");
popup.inline_tableau(s, 550, 550);
if (!popup.ask())
  return;
}

void bwm_tableau_img::step_edges_vd()
{
  my_observer_->step_edges_vd();
}

void bwm_tableau_img::lines_vd()
{
  my_observer_->lines_vd();
}

void bwm_tableau_img::recover_edges()
{
  my_observer_->recover_edges();
}

void bwm_tableau_img::recover_lines()
{
  my_observer_->recover_lines();
}

bool bwm_tableau_img::handle(const vgui_event& e)
{
#if 0 // commented out
  vcl_cout << "Key:" << e.key << " modif: " << e.modifier << vcl_endl;
  if (e.key == 'p' && e.modifier == vgui_SHIFT) {
    create_polygon_mesh();
    return true;
  } else if (e.key == 't' && e.modifier == vgui_SHIFT) {
    this->triangulate_mesh();
    return true;
  } else if ( e.key == 'm' && e.modifier == vgui_SHIFT) {
    this->move_obj_by_vertex();
    return true;
  } else if ( e.key == 'e' && e.modifier == vgui_SHIFT) {
    this->extrude_face();
    return true;
  } else if ( e.key == 's' && e.modifier == vgui_SHIFT) {
    this->save();
    return true;
  } else if ( e.key == '-' && e.modifier == vgui_SHIFT) {
    this->deselect_all();
    return true;
  } else if ( e.key == 'd' && e.modifier == vgui_SHIFT) {
    this->clear_object();
    return true;
  } else if ( e.key == 'a' && e.modifier == vgui_SHIFT) {
    this->clear_all();
    return true;
  } else if ( e.key == 'h' && e.modifier == vgui_SHIFT) {
    this->help_pop();
    return true;
  }
#endif // 0
  return bgui_picker_tableau::handle(e);
}

void bwm_tableau_img::init_mask()
{
  my_observer_->init_mask();
}

void bwm_tableau_img::add_poly_to_mask()
{
  my_observer_->add_poly_to_mask();
}

/*void bwm_tableau_img::add_dontcare_poly_to_mask()
{
  my_observer_->add_dontcare_poly_to_mask();
}*/

void bwm_tableau_img::remove_poly_from_mask()
{
  my_observer_->remove_poly_from_mask();
}
/*void bwm_tableau_img::create_mask()
{
  my_observer_->create_mask();
}*/
