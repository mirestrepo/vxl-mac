#include "vipl_median.h"

template <class ImgIn,class ImgOut,class DataIn,class DataOut,class PixelItr>
bool vipl_median <ImgIn,ImgOut,DataIn,DataOut,PixelItr> :: section_applyop(){
  const ImgIn &in = in_data(0);
  ImgOut &out = *out_data_ptr();
  int size = (radius() < 0) ? 0 : int(radius());

  // circular mask was generated in preop(), we just use it here

  // apply filter:
  DataIn dummy;
  DataIn* v = new DataIn[(2*size+1)*(2*size+1)];
  for(int j = start(Y_Axis()); j < stop(Y_Axis()); ++j)
    for(int i = start(X_Axis(),j); i < stop(X_Axis(),j); ++i) {
      register int count = 0;
      v[count++] = fgetpixel(in, i, j, dummy);
      for (int x=0; x<=size; ++x) for (int y=0; y<=size; ++y) if (mask()[x][y]) {
        v[count++] = getpixel(in, i+x, j+y, dummy);
        v[count++] = getpixel(in, i-x, j+y, dummy);
        v[count++] = getpixel(in, i+x, j-y, dummy);
        v[count++] = getpixel(in, i-x, j-y, dummy);
      }
      // qsort:
      for (int d=count/2; d>0; d=d/2)
        for (int ii=d; ii<count; ii++)
          for (int jj=ii-d; jj>=0 && v[jj]>v[jj+d]; jj-=d) {
            DataIn t = v[jj]; v[jj] = v[jj+d]; v[jj+d] = t;
          }

      fsetpixel(out, i, j, (DataOut)v[count/2]);
    }
  delete[] v;
  return true;
}

// it is important that the mask be computed in preop, if it was done in
// section_applyop then on a large image it would be computed many times.
template <class ImgIn,class ImgOut,class DataIn,class DataOut,class PixelItr>
bool vipl_median <ImgIn,ImgOut,DataIn,DataOut,PixelItr> :: preop(){
  // create circular mask:
  int size = (radius() < 0) ? 0 : int(radius());
  float rs = (radius() < 0) ? 0 : radius() * radius();
  typedef bool* boolptr;
  if(mask() == 0)
	 ref_mask() = new boolptr[1+size];
  else {
	for (int x=0; x<=size; ++x)
	    if(mask()[x]) delete ref_mask()[x];
	delete ref_mask();
	ref_mask() = new boolptr[1+size];
	}
  for (int x=0; x<=size; ++x) {
    ref_mask()[x] = new bool[size+1];
    for (int y=0; y<=size; ++y)
      ref_mask()[x][y] = (x*x + y*y <= rs);
  }
  return true;
}

// Since we will know if radius changes between calls to filter, we
// destroy the mask in postop, after we are all done filtering
template <class ImgIn,class ImgOut,class DataIn,class DataOut,class PixelItr>
bool vipl_median <ImgIn,ImgOut,DataIn,DataOut,PixelItr> :: postop(){
  int size = (radius() < 0) ? 0 : int(radius());
  if(mask()){
    for (int x=0; x<=size; ++x)
      if(mask()[x]) delete ref_mask()[x];
    delete ref_mask();
    ref_mask()=0;
  }
  return true;
}
