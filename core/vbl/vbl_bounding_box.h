#ifndef vbl_bounding_box_h_
#define vbl_bounding_box_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME vbl_bounding_box - A class to hold and update a bounding box
// .LIBRARY vbl
// .HEADER vxl package
// .INCLUDE vbl/vbl_bounding_box.h
// .FILE vbl_bounding_box.txx
// .SECTION Author
//    awf@robots.ox.ac.uk
// Created: 17 Mar 00
//
// .SECTION Modifications
//     970807 AWF Initial version.
//     07 Mar 01 stewart@cs.rpi.edu added "inside" functions

#include <vcl_iosfwd.h>

//: A class to hold and update a bounding box.
//  Save valuable time not writing
//    if (x > xmax).....
template <class T, int DIM>
class vbl_bounding_box {
public:
  // -- Construct an empty bounding box.
  vbl_bounding_box() : initialized_(false) { }

  //: Incorporate 2d point x, y
  void update(T const& x, T const& y) {
    T tmp[2] = {x,y};
    update(tmp);
  }

  //: Incorporate 3d point x, y, z
  void update(T const& x, T const& y, T const& z) {
    T tmp[3] = {x,y,z};
    update(tmp);
  }

  //: Incorporate DIM-d point
  void update(T const* point) {
    if (!initialized_) {
      initialized_ = true;
      for(int i = 0; i < DIM; ++i)
        min_[i] = max_[i] = point[i];
    } else {
      for(int i = 0; i < DIM; ++i) {
        if (point[i] < min_[i]) min_[i] = point[i];
        if (point[i] > max_[i]) max_[i] = point[i];
      }
    }
  }

  //: Reset to empty
  void reset() {
    initialized_ = false;
  }

  //:
  bool empty() const {
    return initialized_ == false;
  }

  //:  is a 2D point inside the bounding box
  bool inside( const T& x, const T& y) const {
    return
      min_[0] <= x && x <= max_[0] &&
      min_[1] <= y && y <= max_[1];
  }

  //:  is a 3D point inside the bounding box
  bool inside( const T& x, const T& y, const T& z) const {
    return
      min_[0] <= x && x <= max_[0] &&
      min_[1] <= y && y <= max_[1] &&
      min_[2] <= z && z <= max_[2];
  }

  //:  inside test for arbitrary dimension
  bool inside(T const* point) {
    for( int i=0; i<DIM; ++i )
      if( point[i] < min_[i] || max_[i] < point[i] )
        return false;
    return true;
  }

  //: return "volume".
  T volume() const {
    if (!initialized_) return T(0);
    T vol = 1;
    for (int i=0; i<DIM; ++i)
      vol *= max_[i] - min_[i];
    return vol;
  }

  vcl_ostream& print(vcl_ostream& s) const;

  T const* get_min() const { return min_; }
  T const* get_max() const { return max_; }

  T      * get_min() { return min_; }
  T      * get_max() { return max_; }

  T const& get_xmin() const { return min_[0]; }
  T const& get_xmax() const { return max_[0]; }
  T const& get_ymin() const { return min_[1]; }
  T const& get_ymax() const { return max_[1]; }

  //#pragma dogma
  // There is no need for the data members to be private to
  // this class. After all, the STL pair<> template has its
  // two main members first, second public without causing
  // any problems.
//protected:
  bool initialized_;
  T min_[DIM];
  T max_[DIM];
};

//------------------------------------------------------------

#if defined(VCL_SUNPRO_CC_50)
// Error: Non-type template parameters are not allowed for function templates.
#else
template <class T, int DIM>
inline // this is "operator \subseteq"
bool nested(vbl_bounding_box<T,DIM> const &a, vbl_bounding_box<T,DIM> const &b)
{
  for (int i=0; i<DIM; ++i)
    if (a.min_[i] < b.min_[i] || a.max_[i] > b.max_[i])
      return false;
  return true;
}

template <class T, int DIM>
inline
bool disjoint(vbl_bounding_box<T,DIM> const &a, vbl_bounding_box<T,DIM> const &b)
{
  for (int i=0; i<DIM; ++i)
    if (a.min_[i] > b.max_[i] || a.max_[i] < b.min_[i])
      return true;
  return false;
}

template <class T, int DIM>
inline
bool meet(vbl_bounding_box<T,DIM> const &a, vbl_bounding_box<T,DIM> const &b)
{ return ! disjoint(a, b); }
#endif

// VC50 has trouble with this
#if !defined (VCL_WIN32) && !defined(VCL_SUNPRO_CC)
template <class T, int DIM>
vcl_ostream& operator << (vcl_ostream& s, const vbl_bounding_box<T,DIM>& bbox) { return bbox.print(s); }
#else
template <class T>
vcl_ostream& operator << (vcl_ostream& s, const vbl_bounding_box<T,2>& bbox) { return bbox.print(s); }
template <class T>
vcl_ostream& operator << (vcl_ostream& s, const vbl_bounding_box<T,3>& bbox) { return bbox.print(s); }
#endif

#endif // vbl_bounding_box_h_
