// This is mul/vil2/algo/vil2_algo_gauss_reduce.cxx
#include "vil2_algo_gauss_reduce.h"
//:
//  \file
//  \brief Functions to smooth and sub-sample image in one direction
//  \author Tim Cootes


//: Smooth and subsample single plane src_im in x to produce dest_im
//  Applies 1-5-8-5-1 filter in x, then samples
//  every other pixel.  Fills [0,(ni+1)/2-1][0,nj-1] elements of dest
void vil2_algo_gauss_reduce(vil2_byte* dest_im,
                            int d_x_step, int d_y_step,
                            const vil2_byte* src_im,
                            int src_ni, int src_nj,
                            int s_x_step, int s_y_step)
{
    vil2_byte* d_row = dest_im;
    const vil2_byte* s_row = src_im;
    int sxs2 = s_x_step*2;
    int ni2 = (src_ni-3)/2;
    for (int y=0;y<src_nj;++y)
    {
        // Set first element of row
        *d_row = *s_row;
        vil2_byte * d = d_row + d_x_step;
        const vil2_byte* s = s_row + sxs2;
        for (int x=0;x<ni2;++x)
        {
            // The 0.5 offset in the following ensures rounding
            *d = vil2_byte(0.5+ 0.05*s[-sxs2] +0.05 *s[sxs2]
                          +0.25*s[-s_x_step]+0.25*s[s_x_step]
                          +0.4*s[0]);

            d += d_x_step;
            s += sxs2;
        }
        // Set last elements of row
        *d = *s;

        d_row += d_y_step;
        s_row += s_y_step;
    }
}

//: Smooth and subsample single plane src_im in x to produce dest_im
//  Applies 1-5-8-5-1 filter in x, then samples
//  every other pixel.  Fills [0,(ni+1)/2-1][0,nj-1] elements of dest
void vil2_algo_gauss_reduce(float* dest_im,
                            int d_x_step, int d_y_step,
                            const float* src_im,
                            int src_ni, int src_nj,
                            int s_x_step, int s_y_step)
{
    float* d_row = dest_im;
    const float* s_row = src_im;
    int sxs2 = s_x_step*2;
    int ni2 = (src_ni-3)/2;
    for (int y=0;y<src_nj;++y)
    {
        // Set first element of row
        *d_row = *s_row;
        float * d = d_row + d_x_step;
        const float* s = s_row + sxs2;
        for (int x=0;x<ni2;++x)
        {
            *d = 0.05f*(s[-sxs2] + s[sxs2])
                +0.25f*(s[-s_x_step]+ s[s_x_step])
                +0.40f*s[0];

            d += d_x_step;
            s += sxs2;
        }
        // Set last elements of row
        *d = *s;

        d_row += d_y_step;
        s_row += s_y_step;
    }
}


//: Smooth and subsample single plane src_im in x to produce dest_im
//  Applies 1-5-8-5-1 filter in x, then samples
//  every other pixel.  Fills [0,(ni+1)/2-1][0,nj-1] elements of dest
void vil2_algo_gauss_reduce(int* dest_im,
                            int d_x_step, int d_y_step,
                            const int* src_im,
                            int src_ni, int src_nj,
                            int s_x_step, int s_y_step)
{
    int* d_row = dest_im;
    const int* s_row = src_im;
    int sxs2 = s_x_step*2;
    int ni2 = (src_ni-3)/2;
    for (int y=0;y<src_nj;++y)
    {
        // Set first element of row
        *d_row = *s_row;
        int * d = d_row + d_x_step;
        const int* s = s_row + sxs2;
        for (int x=0;x<ni2;++x)
        {
            // The 0.5 offset in the following ensures rounding
            *d = int(0.5+ 0.05*s[-sxs2] +0.05 *s[sxs2]
                          +0.25*s[-s_x_step]+0.25*s[s_x_step]
                          +0.4*s[0]);

            d += d_x_step;
            s += sxs2;
        }
        // Set last elements of row
        *d = *s;

        d_row += d_y_step;
        s_row += s_y_step;
    }
}


//: Smooth and subsample single plane src_im in x to produce dest_im using 121 filter in x and y
//  Smooths with a 3x3 filter and subsamples
void vil2_algo_gauss_reduce_121(vil2_byte* dest_im,
                                int d_x_step, int d_y_step,
                                const vil2_byte* src_im,
                                int src_ni, int src_nj,
                                int s_x_step, int s_y_step)
{
  int sxs2 = s_x_step*2;
  int sys2 = s_y_step*2;
  vil2_byte* d_row = dest_im+d_y_step;
  const vil2_byte* s_row1 = src_im + s_y_step;
  const vil2_byte* s_row2 = s_row1 + s_y_step;
  const vil2_byte* s_row3 = s_row2 + s_y_step;
  int ni2 = (src_ni-2)/2;
  int nj2 = (src_nj-2)/2;
  for (int y=0;y<nj2;++y)
  {
      // Set first element of row
      *d_row = *s_row2;
      vil2_byte * d = d_row + d_x_step;
      const vil2_byte* s1 = s_row1 + sxs2;
      const vil2_byte* s2 = s_row2 + sxs2;
      const vil2_byte* s3 = s_row3 + sxs2;
      for (int x=0;x<ni2;++x)
      {
          // The following is a little inefficient - could group terms to reduce arithmetic
          // Add 0.5 so that truncating effetively rounds
          *d = vil2_byte( 0.0625f * s1[-s_x_step] + 0.125f * s1[0] + 0.0625f * s1[s_x_step]
                       + 0.1250f * s2[-s_x_step] + 0.250f * s2[0] + 0.1250f * s2[s_x_step]
                       + 0.0625f * s3[-s_x_step] + 0.125f * s3[0] + 0.0625f * s3[s_x_step] +0.5);

          d += d_x_step;
          s1 += sxs2;
          s2 += sxs2;
          s3 += sxs2;
      }
      // Set last elements of row
      if (src_ni%2==1)
        *d = *s2;

      d_row += d_y_step;
      s_row1 += sys2;
      s_row2 += sys2;
      s_row3 += sys2;
  }

  // Need to set first and last rows as well

  // Dest image should be (src_ni+1)/2 x (src_nj+1)/2
  const vil2_byte* s0 = src_im;
  int ni=(src_ni+1)/2;
  for (int i=0;i<ni;++i)
  {
    dest_im[i]= *s0;
    s0+=sxs2;
  }

  if (src_nj%2==1)
  {
    int yhi = (src_nj-1)/2;
    vil2_byte* dest_last_row = dest_im + yhi*d_y_step;
    const vil2_byte* s_last = src_im + yhi*sys2;
    for (int i=0;i<ni;++i)
    {
      dest_last_row[i]= *s_last;
      s_last+=sxs2;
    }
  }
}

//: Smooth and subsample single plane src_im in x to produce dest_im using 121 filter in x and y
//  Smooths with a 3x3 filter and subsamples
void vil2_algo_gauss_reduce_121(float* dest_im,
                                int d_x_step, int d_y_step,
                                const float* src_im,
                                int src_ni, int src_nj,
                                int s_x_step, int s_y_step)
{
  int sxs2 = s_x_step*2;
  int sys2 = s_y_step*2;
  float* d_row = dest_im+d_y_step;
  const float* s_row1 = src_im + s_y_step;
  const float* s_row2 = s_row1 + s_y_step;
  const float* s_row3 = s_row2 + s_y_step;
  int ni2 = (src_ni-2)/2;
  int nj2 = (src_nj-2)/2;
  for (int y=0;y<nj2;++y)
  {
      // Set first element of row
      *d_row = *s_row2;
      float * d = d_row + d_x_step;
      const float* s1 = s_row1 + sxs2;
      const float* s2 = s_row2 + sxs2;
      const float* s3 = s_row3 + sxs2;
      for (int x=0;x<ni2;++x)
      {
          // The following is a little inefficient - could group terms to reduce arithmetic
          *d =   0.0625f * s1[-s_x_step] + 0.125f * s1[0] + 0.0625f * s1[s_x_step]
               + 0.1250f * s2[-s_x_step] + 0.250f * s2[0] + 0.1250f * s2[s_x_step]
               + 0.0625f * s3[-s_x_step] + 0.125f * s3[0] + 0.0625f * s3[s_x_step];

          d += d_x_step;
          s1 += sxs2;
          s2 += sxs2;
          s3 += sxs2;
      }
      // Set last elements of row
      if (src_ni%2==1)
        *d = *s2;

      d_row += d_y_step;
      s_row1 += sys2;
      s_row2 += sys2;
      s_row3 += sys2;
  }

  // Need to set first and last rows as well

  // Dest image should be (src_ni+1)/2 x (src_nj+1)/2
  const float* s0 = src_im;
  int ni=(src_ni+1)/2;
  for (int i=0;i<ni;++i)
  {
    dest_im[i]= *s0;
    s0+=sxs2;
  }

  if (src_nj%2==1)
  {
    int yhi = (src_nj-1)/2;
    float* dest_last_row = dest_im + yhi*d_y_step;
    const float* s_last = src_im + yhi*sys2;
    for (int i=0;i<ni;++i)
    {
      dest_last_row[i]= *s_last;
      s_last+=sxs2;
    }
  }
}


//: Smooth and subsample single plane src_im in x to produce dest_im using 121 filter in x and y
//  Smooths with a 3x3 filter and subsamples
void vil2_algo_gauss_reduce_121(int* dest_im,
                                int d_x_step, int d_y_step,
                                const int* src_im,
                                int src_ni, int src_nj,
                                int s_x_step, int s_y_step)
{
  int sxs2 = s_x_step*2;
  int sys2 = s_y_step*2;
  int* d_row = dest_im+d_y_step;
  const int* s_row1 = src_im + s_y_step;
  const int* s_row2 = s_row1 + s_y_step;
  const int* s_row3 = s_row2 + s_y_step;
  int ni2 = (src_ni-2)/2;
  int nj2 = (src_nj-2)/2;
  for (int y=0;y<nj2;++y)
  {
      // Set first element of row
      *d_row = *s_row2;
      int * d = d_row + d_x_step;
      const int* s1 = s_row1 + sxs2;
      const int* s2 = s_row2 + sxs2;
      const int* s3 = s_row3 + sxs2;
      for (int x=0;x<ni2;++x)
      {
          // The following is a little inefficient - could group terms to reduce arithmetic
          // Add 0.5 so that truncating effetively rounds
          *d = int( 0.0625f * s1[-s_x_step] + 0.125f * s1[0] + 0.0625f * s1[s_x_step]
                  + 0.1250f * s2[-s_x_step] + 0.250f * s2[0] + 0.1250f * s2[s_x_step]
                  + 0.0625f * s3[-s_x_step] + 0.125f * s3[0] + 0.0625f * s3[s_x_step] +0.5);

          d += d_x_step;
          s1 += sxs2;
          s2 += sxs2;
          s3 += sxs2;
      }
      // Set last elements of row
      if (src_ni%2==1)
        *d = *s2;

      d_row += d_y_step;
      s_row1 += sys2;
      s_row2 += sys2;
      s_row3 += sys2;
  }

  // Need to set first and last rows as well

  // Dest image should be (src_ni+1)/2 x (src_nj+1)/2
  const int* s0 = src_im;
  int ni=(src_ni+1)/2;
  for (int i=0;i<ni;++i)
  {
    dest_im[i]= *s0;
    s0+=sxs2;
  }

  if (src_nj%2==1)
  {
    int yhi = (src_nj-1)/2;
    int* dest_last_row = dest_im + yhi*d_y_step;
    const int* s_last = src_im + yhi*sys2;
    for (int i=0;i<ni;++i)
    {
      dest_last_row[i]= *s_last;
      s_last+=sxs2;
    }
  }
}
