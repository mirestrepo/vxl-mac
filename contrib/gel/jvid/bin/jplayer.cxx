#include <vcl_ios.h>
#include <vcl_iostream.h>
#include <vcl_cstdlib.h> // for vcl_exit()

#include <vgui/vgui.h>
#include <vgui/vgui_adaptor.h>
#include <vgui/vgui_window.h>
#include <vgui/vgui_command.h>
#include <vgui/vgui_image_tableau.h>
#include <vgui/internals/vgui_accelerate.h>
#include <vidl/vidl_io.h>
#include <jvid/jvx_manager.h>

#ifdef HAS_MPEG
# include <vidl/vidl_mpegcodec.h>
#endif

#ifdef HAS_X11
# include <vgui/internals/vgui_accelerate_x11.h>
#endif

#ifdef VCL_WIN32
#include <vidl/vidl_avicodec.h>
#endif


static jvx_manager* jvm = new jvx_manager();

void quit_callback()
{
  vcl_exit(1);
}

void file_callback()
{
  jvm->load_video_file();
}
void play_callback()
{
  jvm->play_video();
}
int main(int argc, char** argv)
{
  // Register video codecs
#ifdef VCL_WIN32
  vidl_io::register_codec(new vidl_avicodec);
#endif

#ifdef HAS_MPEG
  vidl_io::register_codec(new vidl_mpegcodec);
#endif

   // turn off the mfc acceleration since this seems to stop
   // us from doing the double buffering. It also seems to add
   // a certain amount of overhead
#ifdef HAS_MFC
  vgui_accelerate::vgui_mfc_acceleration = false;
#endif

  vcl_cout << "Joe Video\n";

   // Initialize the toolkit.
  vgui::init(argc, argv);

  vgui_menu menubar;
  vgui_menu menufile;

  menufile.add( "Quit", new vgui_command_cfunc( quit_callback ));
  menufile.add( "Load", new vgui_command_cfunc( file_callback ));
  menufile.add( "Play", new vgui_command_cfunc( play_callback ));
  menubar.add( "File", menufile);
  // Initialize the window
  unsigned w = 400, h = 340;

  jvm->set_grid_size_changeable(true);
  vcl_string title = "Joe Video Player";
  vgui_window *win = vgui::produce_window(w, h, menubar, title);
  win->get_adaptor()->set_tableau(jvm);
  win->set_statusbar(true);
  win->enable_vscrollbar(true);
  win->enable_hscrollbar(true);
  win->show();
  jvm->post_redraw();
  return  vgui::run();
}
