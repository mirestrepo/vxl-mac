#ifndef vipl_moment_h_
#define vipl_moment_h_

// .NAME vipl_moment - computation of n-th order moment
// .LIBRARY vipl
// .HEADER vxl package
// .INCLUDE vipl/vipl_moment.h
// .FILE vipl_moment.txx
//
// .SECTION Description
//   This Image Processing class is implemented using the vipl filters,
//   which means that it can be used with any image class (IUE or not,
//   TargetJr or not, vil or not) of any pixel data type.
//
//   The n-th order moment operation replaces a pixel with the expected value
//   of x^n of its surrounding pixels, in a certain neighbourhood. Here the
//   neighbourhood is a arbitrary rectangular mask, the height and width of which
//   are passed to the constructor.
//
//   A fast computation method is used which needs only seven + or - operations per
//   pixel (except for initialization of first row and column) because it uses
//   computed values of previous pixels
//
// .SECTION Author
//   Maarten Vergauwen, K.U.Leuven (ESAT/PSI), 21 September 1999.
//
// .SECTION Modifications
//   Peter Vanroose, Aug.2000 - adapted to vxl
//
// .EXAMPLE examples/example_std_dev.cxx

#include <vipl/filter/vipl_filter_2d.h> // parent class

// must be on single line for perceps:
template <class ImgIn,class ImgOut,class DataIn,class DataOut,VCL_DFL_TYPE_PARAM_STLDECL(PixelItr, vipl_trivial_pixeliter) >
class vipl_moment : public vipl_filter_2d<ImgIn,ImgOut,DataIn,DataOut,PixelItr> {

  // -+-+- data members: -+-+-
private:
  int order_;
  int width_;
  int height_;
  int& ref_order(){return order_;}
  int& ref_width(){return width_;}
  int& ref_height(){return height_;}
  void put_order(int v){order_=v;}
  void put_width(int v){width_=v;}
  void put_height(int v){height_=v;}
public:
  int order() const {return order_;}
  int width() const {return width_;}
  int height() const {return height_;}

  // -+-+- constructors/destructors: -+-+-
public:
  inline vipl_moment(int n,int w=3,int h=3)
    : vipl_filter_2d<ImgIn,ImgOut,DataIn,DataOut,PixelItr>(), order_(n), width_(w), height_(h) {}
  inline vipl_moment(vipl_moment const& A)
    : vipl_filter_2d<ImgIn,ImgOut,DataIn,DataOut,PixelItr>(A), order_(A.order()), width_(A.width()), height_(A.height()) {}
  inline ~vipl_moment() {}

  // -+-+- required method for filters: -+-+-
  bool section_applyop();

};


#endif // vipl_moment_h_
