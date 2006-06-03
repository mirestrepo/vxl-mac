#include <vcl_compiler.h>
#include <vgui/vgui.h>
#include <vgui/vgui_adaptor.h>
#include <vgui/vgui_window.h>
#include <vidl/vidl_io.h>
#include "vidfpl_vil_menus.h"
#include <vvid/vvid_vil_file_manager.h>

#include <vidl/vidl_image_list_codec.h>

#ifdef HAS_MPEG2
# include <vidl/vidl_mpegcodec.h>
#endif

#ifdef VCL_WIN32
#include <vidl/vidl_avicodec.h>
#endif

int main(int argc, char** argv)
{
  // Register video codecs
  vidl_io::register_codec(new vidl_image_list_codec);

#ifdef VCL_WIN32
  vidl_io::register_codec(new vidl_avicodec);
#endif

#ifdef HAS_MPEG2
  vcl_cout << " Has MPEG\n";
  vidl_io::register_codec(new vidl_mpegcodec);
#endif
#if defined(VCL_WIN32)
vcl_cout << '\n'<< "Max number of open files has been reset from " << _getmaxstdio();
_setmaxstdio(2048);
#endif

   // Initialize the toolkit.
  vgui::init(argc, argv);
  vgui_menu menubar = vidfpl_vil_menus::get_menu();
  unsigned w = 400, h = 340;

  vcl_string title = "Video (vil) File Player 1.0";
  vgui_window* win = vgui::produce_window(w, h, menubar, title);
  win->get_adaptor()->set_tableau(vvid_vil_file_manager::instance());
  win->set_statusbar(true);
  win->enable_vscrollbar(true);
  win->enable_hscrollbar(true);
  win->show();
  vvid_vil_file_manager::instance()->set_window(win);
  vvid_vil_file_manager::instance()->post_redraw();
  vgui::run();
  return 0;
}
