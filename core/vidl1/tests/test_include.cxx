#ifdef HAS_AVI
#include <vidl/vidl_avicodec.h>
# if defined(_MSC_VER) || defined(__BORLANDC__) || defined(__MINGW32__)
#include <vidl/vidl_win_avicodec.h>
# else
#include <vidl/vidl_avifile_avicodec.h>
# endif
#endif
#include <vidl/vidl_clip.h>
#include <vidl/vidl_clip_sptr.h>
#include <vidl/vidl_codec.h>
#include <vidl/vidl_codec_sptr.h>
#ifdef HAS_FFMPEG
#include <vidl/vidl_ffmpeg_codec.h>
#endif
#include <vidl/vidl_file_sequence.h>
#include <vidl/vidl_frame.h>
#include <vidl/vidl_frame_sptr.h>
#include <vidl/vidl_frame_resource.h>
#include <vidl/vidl_image_list_codec.h>
#include <vidl/vidl_image_list_codec_sptr.h>
#include <vidl/vidl_io.h>
#include <vidl/vidl_movie.h>
#include <vidl/vidl_movie_sptr.h>
#ifdef HAS_MPEG2
#include <vidl/vidl_mpegcodec.h>
#include <vidl/vidl_mpegcodec_helper.h>
#include <vidl/vidl_mpegcodec_sptr.h>
#endif
#include <vidl/vidl_vob_frame_index.h>
#include <vidl/vidl_yuv_2_rgb.h>

int main() { return 0; }
