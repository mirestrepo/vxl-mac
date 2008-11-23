#include <vidl2/v4l2_pixel_format.h>
#include <vidl2/vidl2_color.h>
#include <vidl2/vidl2_convert.h>
#include <vidl2/vidl2_exception.h>
#include <vidl2/vidl2_frame.h>
#include <vidl2/vidl2_frame_sptr.h>
#include <vidl2/vidl2_pixel_format.h>
#include <vidl2/vidl2_pixel_iterator.h>
#include <vidl2/vidl2_istream.h>
#include <vidl2/vidl2_istream_sptr.h>
#include <vidl2/vidl2_istream_image_resource.h>
#include <vidl2/vidl2_image_list_istream.h>
#include <vidl2/vidl2_ostream.h>
#include <vidl2/vidl2_ostream_sptr.h>
#include <vidl2/vidl2_image_list_ostream.h>
#include <vidl2/vidl2_iidc1394_params.h>
#if VIDL2_HAS_VIDEODEV2
#include <vidl2/vidl2_v4l2_device.h>
#include <vidl2/vidl2_v4l2_device_sptr.h>
#include <vidl2/vidl2_v4l2_devices.h>
#include <vidl2/vidl2_v4l2_istream.h>
#include <vidl2/vidl2_v4l2_control.h>
#endif
#if VIDL2_HAS_FFMPEG
#include <vidl2/vidl2_ffmpeg_init.h>
#include <vidl2/vidl2_ffmpeg_istream.h>
#include <vidl2/vidl2_ffmpeg_ostream.h>
#include <vidl2/vidl2_ffmpeg_ostream_params.h>
#include <vidl2/vidl2_ffmpeg_convert.h>
#endif
#if VIDL2_HAS_DC1394
#include <vidl2/vidl2_dc1394_istream.h>
#endif
#if VIDL2_HAS_DIRECTSHOW
#include <vidl2/vidl2_dshow.h>
#include <vidl2/vidl2_dshow_file_istream.h>
#include <vidl2/vidl2_dshow_live_istream.h>
#include <vidl2/vidl2_dshow_istream_params.h>
#if VIDL2_HAS_DIRECTSHOW_ESF
#include <vidl2/vidl2_dshow_istream_params_esf.h>
#endif // HAS_DIRECTSHOW_ESF
#endif // HAS_DIRECTSHOW
#if 0 // FIXME
#include <vidl2/vidl2_v4l_istream.h>
#include <vidl2/vidl2_v4l_params.h>
#endif // 0

#include <vidl2/gui/vidl2_gui_param_dialog.h>
#include <vidl2/gui/vidl2_capture_tableau.h>
#include <vidl2/gui/vidl2_capture_tableau_sptr.h>

int main() { return 0; }
