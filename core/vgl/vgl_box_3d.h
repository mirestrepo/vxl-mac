#ifndef vgl_box_3d_h
#define vgl_box_3d_h
#ifdef __GNUC__
#pragma interface
#endif

// This is vxl/vgl/vgl_box_3d.h

//:
// \file
// \brief Describe a 3D box.
// \author Don Hamilton, Peter Tu
//
// \verbatim
// Modifications
//  Peter Vanroose, Oct  7 2001: Removed deprecated get_*() functions
//  Peter Vanroose, Oct  6 2001: Added method add(vgl_point_3d<T>) to enlarge a box
//  Peter Vanroose, Oct  5 2001: Added operator==() and methods is_empty() and contains()
//  Peter Vanroose, Jul 10 2001: Deprecated get_*() in favour of *(), and explicit casts
//  NPC (Manchester) 14/03/2001: Tidied up the documentation + added binary_io
//  Peter Vanroose, Feb 28 2000: lots of minor corrections
// \endverbatim

#include <vcl_iosfwd.h>
#include <vcl_string.h>
#include <vgl/vgl_fwd.h> // forward declare vgl_point_3d

//: Represents a 3D box
//  A 3d box with sides aligned with x, y and z axes. Supports operations
//  required of a bounding box for geometric volume tests.
//  A box can be empty; this is the case when MaxPosition is to the left or
//  below or before MinPosition.
//
//  \verbatim
//                                 MaxPosition
//                       |<--width-->|
//                       O-----------O  ---
//                      /           /|   ^
//                     /           / |   |
//                    O-----------O  | depth
//                    |           |  |   |
//                    |  centroid |  |   v
//                    |     o     |  O  ---
//     Z              |           | /   /_____height
//     |   Y          |           |/   /
//     |  /           O-----------O  ---
//     | /         MinPosition
//     O-----X
// \endverbatim

template <class Type>
class vgl_box_3d {

   // PUBLIC INTERFACE----------------------------------------------------------

public:

  //: Default constructor (creates empty box)
  vgl_box_3d();

  //: Construct from min corner and max corner
  vgl_box_3d(Type const min_position[3],
             Type const max_position[3] );

  //: Construct from min corner and max corner
  vgl_box_3d(vgl_point_3d<Type> const& min_pos,
             vgl_point_3d<Type> const& max_pos);

  //: Construct from ranges in x,y,z (take care with order of inputs)
  vgl_box_3d(Type xmin, Type ymin, Type zmin,
             Type xmax, Type ymax, Type zmax);

  //: Construct width x height x depth box centred at centroid
  vgl_box_3d(const Type centroid[3],
             Type width, Type height, Type depth);

  //: Construct width x height x depth box centred at centroid
  vgl_box_3d(vgl_point_3d<Type> const& centroid,
             Type width, Type height, Type depth);

#if 0
  // default copy constructor:
  inline vgl_box_3d(vgl_box_3d const& that) {
    this->min_pos_[0] = that.min_pos_[0]; this->max_pos_[0] = that.max_pos_[0];
    this->min_pos_[1] = that.min_pos_[1]; this->max_pos_[1] = that.max_pos_[1];
    this->min_pos_[2] = that.min_pos_[2]; this->max_pos_[2] = that.max_pos_[2];
  }
  // default destructor:
  inline ~vgl_box_3d() {}
  // Default assignment operator:
  inline vgl_box_3d& operator=(vgl_box_3d const& that){
    this->min_pos_[0] = that.min_pos_[0]; this->max_pos_[0] = that.max_pos_[0];
    this->min_pos_[1] = that.min_pos_[1]; this->max_pos_[1] = that.max_pos_[1];
    this->min_pos_[2] = that.min_pos_[2]; this->max_pos_[2] = that.max_pos_[2];
    return *this;
  }
#endif

  //: Equality test
  inline bool operator==(vgl_box_3d<Type> const& b) const {
    // All empty boxes are equal:
    if (b.is_empty() ) return is_empty();
    return (min_x() == b.min_x() && min_y() == b.min_y() && min_z() == b.min_z()
         && max_x() == b.max_x() && max_y() == b.max_y() && max_z() == b.max_z());
  }

  // Data Access---------------------------------------------------------------

  //: Width (x dimension)
  Type width() const;
  //: Height (y dimension)
  Type height()const;
  //: Depth (z dimension)
  Type depth() const;

  //: Get volume of this box
  inline Type volume() const { return width()*height()*depth(); }

  //: min x
  inline Type min_x() const { return min_pos_[0]; }
  //: min y
  inline Type min_y() const { return min_pos_[1]; }
  //: min z
  inline Type min_z() const { return min_pos_[2]; }

  //: max x
  inline Type max_x() const { return max_pos_[0]; }
  //: max y
  inline Type max_y() const { return max_pos_[1]; }
  //: max z
  inline Type max_z() const { return max_pos_[2]; }

  //: Get x component of centroid
  inline Type centroid_x() const { return Type(0.5*(min_pos_[0]+max_pos_[0])); }
  //: Get y component of centroid
  inline Type centroid_y() const { return Type(0.5*(min_pos_[1]+max_pos_[1])); }
  //: Get z component of centroid
  inline Type centroid_z() const { return Type(0.5*(min_pos_[2]+max_pos_[2])); }

  //: Get the centroid point
  vgl_point_3d<Type> centroid() const;

  //: (min_x,min_y,min_z)
  vgl_point_3d<Type> min_point() const;
  
  //: (max_x,max_y,max_z)
  vgl_point_3d<Type> max_point() const;

  // Data Control--------------------------------------------------------------

  //: Return true if this box is empty
  inline bool is_empty() const {
    return (min_x() > max_x() || min_y() > max_y() || min_z() > max_z());
  }

  //: Add a point to this box, by possibly enlarging the box
  // so that the point just falls within the box.
  // Adding a point to an empty box makes it a size zero box only containing p.
  void add(vgl_point_3d<Type> const& p);

  //: Return true iff the point p is inside this box
  bool contains(vgl_point_3d<Type> const& p) const;

  //: Return true if (x,y,z) is inside this box, ie x_min<=x<=x_max etc
  inline bool contains(Type const& x, Type const& y, Type const& z) const {
    return x >= min_x() && x <= max_x() &&
           y >= min_y() && y <= max_y() &&
           z >= min_z() && z <= max_z();
  }

  //: Make the box empty
  void empty();

  //: Set min x ordinate of box (other sides unchanged)
  inline void set_min_x(Type min_x) {min_pos_[0]=min_x;}
  //: Set min y ordinate of box (other sides unchanged)
  inline void set_min_y(Type min_y) {min_pos_[1]=min_y;}
  //: Set min z ordinate of box (other sides unchanged)
  inline void set_min_z(Type min_z) {min_pos_[2]=min_z;}

  //: Set max x ordinate of box (other sides unchanged)
  inline void set_max_x(Type max_x) {max_pos_[0]=max_x;}
  //: Set max y ordinate of box (other sides unchanged)
  inline void set_max_y(Type max_y) {max_pos_[1]=max_y;}
  //: Set max z ordinate of box (other sides unchanged)
  inline void set_max_z(Type max_z) {max_pos_[2]=max_z;}

  //: Move box so centroid lies at cx (size unchanged)
  void set_centroid_x(Type cx);
  //: Move box so centroid lies at cy (size unchanged)
  void set_centroid_y(Type cy);
  //: Move box so centroid lies at cz (size unchanged)
  void set_centroid_z(Type cz);

  //: Set width (x), centroid unchanged
  void set_width( Type width);
  //: Set height (y), centroid unchanged
  void set_height(Type height);
  //: Set depth (z), centroid unchanged
  void set_depth( Type depth);

  //: Set min corner (max corner unchanged)
  inline void set_min_position(Type const m[3]) { min_pos_[0]=m[0]; min_pos_[1]=m[1]; min_pos_[2]=m[2]; }
  //: Set max corner (min corner unchanged)
  inline void set_max_position(Type const m[3]) { max_pos_[0]=m[0]; max_pos_[1]=m[1]; max_pos_[2]=m[2]; }
  //: Set min corner (max corner unchanged)
  void set_min_point(vgl_point_3d<Type> const& min_pt);
  //: Set max corner (min corner unchanged)
  void set_max_point(vgl_point_3d<Type> const& max_pt);
  //: Move box so centroid lies at c (size unchanged)
  inline void set_centroid(Type const c[3]) { set_centroid_x(c[0]); set_centroid_y(c[1]); set_centroid_z(c[2]); }
  //: Move box so centroid lies at centroid (size unchanged)
  void set_centroid(vgl_point_3d<Type> const& centroid);

  //: Write "<vgl_box_3d x0,y0,z0 to x1,y1,z1>" to stream
  vcl_ostream& print(vcl_ostream&) const;

  //: Write "x0 y0 z0 x1 y1 z1(endl)" to stream
  vcl_ostream& write(vcl_ostream&) const;

  //: Read x0,y0,z0,x1,y1,z1 from stream
  vcl_istream& read(vcl_istream&);

  // INTERNALS-----------------------------------------------------------------
protected:
  // Data Members--------------------------------------------------------------
  Type min_pos_[3];
  Type max_pos_[3];
};


//: Write box to stream
template <class Type>
vcl_ostream&  operator<<(vcl_ostream& s, vgl_box_3d<Type> const& p);

//: Read box from stream
template <class Type>
vcl_istream&  operator>>(vcl_istream& is,  vgl_box_3d<Type>& p);

#define VGL_BOX_3D_INSTANTIATE(T) extern "please include vgl/vgl_box_3d.txx first"

#endif // vgl_box_3d_h
