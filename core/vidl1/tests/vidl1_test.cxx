#include <vidl1/vidl1_movie.h>
#include <vidl1/vidl1_frame.h>
#include <vidl1/vidl1_io.h>
#include <vcl_iostream.h>
#include <vcl_cstdlib.h>

int main ()
{
  vidl1_movie_sptr movie = new vidl1_movie;
  for (vidl1_movie::frame_iterator pframe = movie->begin();
       pframe != movie->end(); ++pframe)
  {
    vidl1_frame_sptr frame = pframe;
    if (!frame) return 1;
  }

  // A better way to do it
  for (vidl1_movie::frame_iterator pframe = movie->first();
       pframe <= movie->last(); ++pframe)
  {
    vil_image_view_base_sptr im = pframe->get_view();
    vcl_cout << "Got frame: " << im->ni() << 'x' << im->nj() << '\n';
  }

  // Running through the frames 2 images at a time
  for (vidl1_movie::frame_iterator pframe = movie->first();
       pframe <= movie->last(); pframe += 2)
  {
    vil_image_view_base_sptr im = pframe->get_view();
    vcl_cout << "Got frame: " << im->ni() << 'x' << im->nj() << '\n';
  }

  // Running backwards throught the image
  for (vidl1_movie::frame_iterator pframe = movie->last();
       pframe >= movie->first(); --pframe)
  {
    vil_image_view_base_sptr im = pframe->get_view();
    vcl_cout << "Got frame: " << im->ni() << 'x' << im->nj() << '\n';
  }

  // Backwards two at a time
  for (vidl1_movie::frame_iterator pframe = movie->last();
       pframe >= movie->first(); pframe -= 2)
  {
    vil_image_view_base_sptr im = pframe->get_view();
    vcl_cout << "Got frame: " << im->ni() << 'x' << im->nj() << '\n';
  }

  // Run over all pairs of images
  for (vidl1_movie::frame_iterator pframe1 = movie->first();
       pframe1 <= movie->last(); ++pframe1)
    for (vidl1_movie::frame_iterator pframe2 = movie->first();
         pframe2 < pframe1; ++pframe2)
    {
      // Run some test on the two images
      vil_image_view_base_sptr im1 = pframe1->get_view();
      vil_image_view_base_sptr im2 = pframe2->get_view();
      if (im1 == im2)
        vcl_cout << "Frames " << pframe1.current_frame_number()
                 << " and " << pframe2.current_frame_number() << " are equal\n";
    }

  // Running over frames 10 to 20
  int p=0;
  for (vidl1_movie::frame_iterator pframe = movie->begin();
       pframe != movie->end() && p<=20; ++p,++pframe)
    if (p>=10)
      pframe->get_view();

  vcl_exit(0);

  // I want to test the signature, but couldn't be bothered checking
  // it works. IMS
  vidl1_io::save(movie, "image_directory" , "ImageList");
}
