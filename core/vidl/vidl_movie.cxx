#include "vidl_movie.h"

#include <vcl_iostream.h>

#include <vidl/vidl_frame.h>
#include <vidl/vidl_clip.h>

//=========================================================================
//  Methods for vidl_movie.
//_________________________________________________________________________

//------------------------------------------------------------------------
// CONSTRUCTOR(S) AND DESTRUCTOR

// -- Constructor
vidl_movie::vidl_movie() {}

// -- Constructor, build a movie with the single given clip
vidl_movie::vidl_movie(vidl_clip_ref clip)
{
  // Check validity of initialisation
  if (!clip_.empty())
    vcl_cerr << "Bad inittialisation of the movie."<< vcl_endl;

  add_clip(clip);
}

// -- destructor
vidl_movie::~vidl_movie()
{

}

// -- Get the frame numbered n (frames are numbered from 0 to total-1)
vidl_frame_ref vidl_movie::get_frame(int n)
{
  vidl_frame_ref ret_frame = NULL;

  vcl_list<vidl_clip_ref>::iterator i = clip_.begin();

  while ((i!=clip_.end()) && 
         (!(ret_frame=(*i)->get_frame(n))))
    {
    n = n - (**i).length();
    i++;
    }

  // one can note that if the frame was not in the clip (n too big),
  // Then, NULL is returned.
  return ret_frame;
  
}

// -- Add a clip at the end of the movie
void vidl_movie::add_clip(vidl_clip_ref clip)
{
  clip_.push_back(clip);
}


// -- Give back the number of frames of the movie
int vidl_movie::length() const
{
  int number = 0; 
  for (vcl_list<vidl_clip_ref>::const_iterator i=clip_.begin(); i!= clip_.end(); ++i)
    number += (*i)->length();

  return number;
}

// -- Return the horizontal size of the frames in the movie
// Check that all the movies do have the same size, output
// an error if not
int vidl_movie::width() const
{ 
  // Get the size X of the first clip
  int sizeX = 0;
  vcl_list<vidl_clip_ref>::const_iterator i=clip_.begin();
  sizeX = (*i)->width();

  // Check that the (eventually) other clips have the same size
  for (; i!= clip_.end(); ++i)
    if ((*i)->width() != sizeX)
      {
        vcl_cerr << "SizeX of the movie asked. But the different clips have different sizes." 
             << vcl_endl;
        return 0;
      }

  // Return the size X
  return sizeX;

}

// -- Return the vertical size of the frames in the movie
// Check that all the movies do have the same size, output
// an error if not
int vidl_movie::height() const
{ 
  // Get the size Y of the first clip
  int sizeY = 0;
  vcl_list<vidl_clip_ref>::const_iterator i = clip_.begin();
  sizeY = (*i)->height();

  // Check that the (eventually) other clips have the same size
  for (; i!= clip_.end(); ++i)
    if ((*i)->height() != sizeY)
      {
        vcl_cerr << "SizeY of the movie asked. But the different clips have different sizes." 
             << vcl_endl;
        return 0;
      }

  // Return the size Y
  return sizeY;
}

