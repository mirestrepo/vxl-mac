// <begin copyright notice>
// ---------------------------------------------------------------------------
//
//                   Copyright (c) 1997 TargetJr Consortium
//               GE Corporate Research and Development (GE CRD)
//                             1 Research Circle
//                            Niskayuna, NY 12309
//                            All Rights Reserved
//              Reproduction rights limited as described below.
//
//      Permission to use, copy, modify, distribute, and sell this software
//      and its documentation for any purpose is hereby granted without fee,
//      provided that (i) the above copyright notice and this permission
//      notice appear in all copies of the software and related documentation,
//      (ii) the name TargetJr Consortium (represented by GE CRD), may not be
//      used in any advertising or publicity relating to the software without
//      the specific, prior written permission of GE CRD, and (iii) any
//      modifications are clearly marked and summarized in a change history
//      log.
//
//      THE SOFTWARE IS PROVIDED "AS IS" AND WITHOUT WARRANTY OF ANY KIND,
//      EXPRESS, IMPLIED OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY
//      WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
//      IN NO EVENT SHALL THE TARGETJR CONSORTIUM BE LIABLE FOR ANY SPECIAL,
//      INCIDENTAL, INDIRECT OR CONSEQUENTIAL DAMAGES OF ANY KIND OR ANY
//      DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
//      WHETHER OR NOT ADVISED OF THE POSSIBILITY OF SUCH DAMAGES, OR ON
//      ANY THEORY OF LIABILITY ARISING OUT OF OR IN CONNECTION WITH THE
//      USE OR PERFORMANCE OF THIS SOFTWARE.
//
// ---------------------------------------------------------------------------
// <end copyright notice>
#ifndef _xpixel_h_
#define _xpixel_h_

//:
// \file

#include <vnl/vnl_vector.h>
#include "gevd_bufferxy.h"

//: Get reference to pixel as a pointer to a vnl_vector<float>, at indexes (x, y).

inline vnl_vector<float>*&
fvectorPixel(gevd_bufferxy& buf, int x, int y)
{
  return (*((vnl_vector<float>**) buf.GetElementAddr(x,y)));
}

inline vnl_vector<float>*
fvectorPixel(const gevd_bufferxy& buf, int x, int y)
{
  return (*((vnl_vector<float>*const *) buf.GetElementAddr(x,y)));
}

inline void freeFVectors(gevd_bufferxy& buf)
{
  for (int x = 0; x < buf.GetSizeX(); x++)
    for (int y = 0; y < buf.GetSizeY(); y++)
      delete fvectorPixel(buf, x, y);
}

#endif
