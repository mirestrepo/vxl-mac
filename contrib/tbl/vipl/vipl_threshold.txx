#ifndef vipl_threshold_txx_
#define vipl_threshold_txx_

#include "vipl_threshold.h"

template <class ImgIn,class ImgOut,class DataIn,class DataOut,class PixelItr>
bool vipl_threshold <ImgIn,ImgOut,DataIn,DataOut,PixelItr> :: section_applyop(){
  const DataIn dummy = DataIn(0);
  const ImgIn &in = in_data();
  ImgOut &out = out_data();

  int startx = start(X_Axis());
  int starty = start(Y_Axis());
  int stopx = stop(X_Axis());
  int stopy = stop(Y_Axis());
  for (int j = starty; j < stopy; ++j)
    for (int i = startx; i < stopx; ++i) {
      DataIn p = fgetpixel(in, i, j, dummy);
      if (p <= threshold()) {fsetpixel(out, i, j, (DataOut)below());}
      else if (aboveset()) {fsetpixel(out, i, j, (DataOut)above());}
      else {fsetpixel(out, i, j, p);}
    }
  return true;
}

#endif // vipl_threshold_txx_
