/*
  fsm
*/
#include <vcl_iostream.h>

#include <vil1/vil1_load.h>
#include <vil1/vil1_image.h>
#include <vil1/vil1_pyramid.h>

#include <vgui/vgui.h>
#include <vgui/vgui_gl.h>
#include <vgui/vgui_event.h>
#include <vgui/vgui_viewer2D.h>
#include <vgui/vgui_image_tableau.h>

struct example_tabby : public vgui_tableau
{
  int level;
  vil1_pyramid pyr;
  vgui_image_tableau_new image_tab;
  vgui_slot slot;

  example_tabby(vil1_image const &image) : level(0), pyr(image), slot(this, image_tab) {
    image_tab->set_image(image);
  }

  bool handle(vgui_event const &e) {
    // compensate for size change by scaling by 2^level :
    glMatrixMode(GL_MODELVIEW);
    for (int i=0; i<level; ++i)
      glScalef(2, 2, 2);

    //
    if (e.type == vgui_KEY_PRESS && e.key == vgui_PGUP) {
      ++level;
      if (level <= 5) {
        image_tab->set_image(pyr[level]);
        post_redraw();
        vcl_cerr << "level " << level << vcl_endl;
      }
      else
        level = 5;
      return true;
    }
    else if (e.type == vgui_KEY_PRESS && e.key == vgui_PGDN) {
      --level;
      if (level >= 0) {
        image_tab->set_image(pyr[level]);
        post_redraw();
        vcl_cerr << "level " << level << vcl_endl;
      }
      else
        level = 0;
      return true;
    }
    else
      return slot.handle(e);
  }
};

int main(int argc, char **argv)
{
  vgui::init(argc, argv);

  if (argc != 2) {
    vcl_cerr << "need name of image" << vcl_endl;
    return 1;
  }

  vil1_image image = vil1_load(argv[1]);
  if (!image) {
    vcl_cerr << "load failed -- invalid image?" << vcl_endl;
    return 1;
  }
  vcl_cerr << image << vcl_endl;

  vgui_tableau_sptr tab(new example_tabby(image));
  vgui_viewer2D_new zoom(tab);
  return vgui::run(zoom, image.width(), image.height());
}
