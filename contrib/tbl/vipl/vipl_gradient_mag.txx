#ifndef vipl_gradient_mag_txx_
#define vipl_gradient_mag_txx_

#include "vipl_gradient_mag.h"
#include <vcl_cmath.h> // for sqrt()

template <class ImgIn,class ImgOut,class DataIn,class DataOut,class PixelItr>
bool vipl_gradient_mag <ImgIn,ImgOut,DataIn,DataOut,PixelItr> :: section_applyop(){
  const ImgIn &in = in_data(0);
  ImgOut &out = *out_data_ptr();

  DataIn dummy = DataIn(); register double dx, dy;
  int startx = start(X_Axis());
  int starty = start(Y_Axis());
  int stopx = stop(X_Axis());
  int stopy = stop(Y_Axis());
  for (int j = starty; j < stopy; ++j)
    for (int i = startx; i < stopx; ++i) {
      dx = fgetpixel(in, i, j, dummy) - getpixel(in, i-1, j, dummy);
      dy = fgetpixel(in, i, j, dummy) - getpixel(in, i, j-1, dummy);
      dx = (vcl_sqrt( dx*dx + dy*dy ) + shift()) * scale();
      fsetpixel(out, i, j, (DataOut)dx);
    }
  return true;
}

#endif // vipl_gradient_mag_txx_
