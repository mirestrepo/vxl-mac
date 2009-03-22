// This is an example of how to use vidl1_mpegcodec,
// written by Ming Li(ming@mpi-sb.mpg.de),
// Max-Planck-Institut fuer Informatik, Germany, 29 Jan 2003.

// may not work for some format mpeg2 files due to the fixed
// load_mpegcodec_callback function !!

#include <vcl_cassert.h>
#include <vcl_cstdlib.h>
#include <vil/vil_save.h>
#include <vidl1/vidl1_io.h>
#include <vidl1/vidl1_mpegcodec.h>
#include <vidl1/vidl1_movie.h>

static void my_load_mpegcodec_callback(vidl1_codec * vc)
{
    bool grey_scale = false;
    bool demux_video = true;
    vcl_string pid = "0x00";
    int numframes = -1;

    vidl1_mpegcodec * mpegcodec = vc->castto_vidl1_mpegcodec();
      if (!mpegcodec) return;

    mpegcodec->set_grey_scale(grey_scale);
    if (demux_video) mpegcodec->set_demux_video();
      mpegcodec->set_pid(pid.c_str());
    mpegcodec->set_number_frames(numframes);
    mpegcodec->init();
}

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    vcl_cerr << "Please specify an MPEG movie file as first command line argument.\n"
             << "The middle frame will then be saved to a file named test.ppm\n";
    return 1;
  }

  vidl1_io::load_mpegcodec_callback=&my_load_mpegcodec_callback;

  vidl1_movie_sptr movie = vidl1_io::load_movie(argv[1]);
  assert( movie );
  assert( movie->length()>0 );
  vcl_cout << "Length = " << movie->length() << vcl_endl
           << "Width  = " << movie->width() << vcl_endl
           << "Height = " << movie->height() << vcl_endl;


  //traverse the movie sequence
  int i=0;
  for (vidl1_movie::frame_iterator pframe = movie->first();
       pframe <= movie->last();
       ++pframe,i++)
  {
    vil_image_view_base_sptr im = pframe->get_view();
    vcl_cout << "decode frame " << i << vcl_endl;
  }

  //random frame access
  vil_image_view_base_sptr im=movie->get_view(movie->length()/2);
  vil_save(*im,"test.ppm");

  vcl_exit (0);
  return 0;
}
