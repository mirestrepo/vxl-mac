// This is gel/vsol/vsol_spatial_object_2d.h
#ifndef vsol_spatial_object_2d_h_
#define vsol_spatial_object_2d_h_
//-----------------------------------------------------------------------------
//:
// \file
// \brief Base class of 2D spatial entities (topology geometry group)
//
//   vsol_spatial_object_2d is the base class of all 2d spatial entities.
//   It provides ref counting, timestamps, a bounding box, ...
//
// \author
//     Patricia A. Vrobel
//
// \verbatim
//  Modifications
//   1995/12/xx JLM
//           Replaced the SpatialGroup pointer by an IUBox bounding volume.
//           See the discussion in SpatialGroup.h for the rationale.
//           The bounding volume is updated according to a timestamp
//           mechanism similar to that used in 3D display. That is,
//           if the vsol_spatial_object_2d is modified it is "touched" to
//           set the timestamp.  Then, any request to bounding information
//           automatically updates the bounding volume if necessary.
//   1996/12/16 Peter VANROOSE  made destructor protected
//   2000/05/03 Peter TU        ported to vxl
//   2000/05/10 Fran�ois BERTEL add cast_to_group() because VXL is not compiled
//                              with -frtti :-(
//   2002/12/12 Peter Vanroose added cast_to_region()
//   2003/01/08 Peter Vanroose made bounding box "mutable" and changed interface
//   2003/01/09 Peter Vanroose deprecated set_min_x() etc. and replaced with
//                       more safe set_bounding_box() and add_to_bounding_box().
//                     (The old setup only worked correctly when (0,0) in bbox.)
// \endverbatim
//-----------------------------------------------------------------------------

#include <vsol/vsol_spatial_object_2d_sptr.h>

#include <vcl_string.h>
#include <vcl_iostream.h>
#include <vul/vul_timestamp.h>
#include <vbl/vbl_ref_count.h>
#include <vsl/vsl_fwd.h>
#include <vsol/vsol_box_2d_sptr.h>

class vsol_box_2d;
class vsol_point_2d;
class vsol_curve_2d;
class vsol_region_2d;
class vsol_group_2d;
class vtol_topology_object;
class vsol_spatial_object_2d;
extern void iu_delete(vsol_spatial_object_2d *);

#ifndef vsol_spatial_object_flags_
#define vsol_spatial_object_flags_

// system flags
const unsigned int VSOL_UNIONBIT       = 0x80000000;
const unsigned int VSOL_SYSTEM_FLAG1   = 0x01000000;
// user flags
const unsigned int VSOL_FLAG1          = 0x40000000;
const unsigned int VSOL_FLAG2          = 0x20000000;
const unsigned int VSOL_FLAG3          = 0x1000000;
const unsigned int VSOL_FLAG4          = 0x08000000;
const unsigned int VSOL_FLAG5          = 0x04000000;
const unsigned int VSOL_FLAG6          = 0x02000000;

// mask for last three bytes of tag field
const unsigned int VSOL_DEXID_BITS     = 0x00FFFFFF;
const unsigned int VSOL_FLAG_BITS      = 0xFF000000;

#endif // vsol_spatial_object_flags_


class vsol_spatial_object_2d : public vul_timestamp, public vbl_ref_count
{
  // Data Members--------------------------------------------------------------
 protected:
  mutable vsol_box_2d_sptr bounding_box_; // rectangular bounding area
  unsigned int tag_;
  int id_;
  static int tagcount_;// global count of all spatial objects.

 public:
  enum vsol_spatial_object_2d_type
  {
    SPATIAL_NO_TYPE=0,
    TOPOLOGYOBJECT,
    POINT,
    CURVE,
    REGION,
    SPATIALGROUP,
    NUM_SPATIALOBJECT_TYPES
  };
  static const char *SpatialTypes[];
  static const float eps;

  // Constructors/Destructors--------------------------------------------------

  virtual ~vsol_spatial_object_2d();

 protected:
  //: constructor initializes basic vsol_spatial_object_2d attributes.
  //   bounding_box is set to NULL.
  vsol_spatial_object_2d();
  vsol_spatial_object_2d(vsol_spatial_object_2d const& other);
  void not_applicable(vcl_string const& message) const
  { vcl_cerr<<message<<"() function call not applicable\tfor spatial object "
            <<get_name()<<" !\n";
  }

 public:
  // Data Access---------------------------------------------------------------

  //: get the spatial type
  virtual vsol_spatial_object_2d_type spatial_type() const=0;

  const char *get_name() const; // derived from spatial_type()

  //: compute bounding box, do nothing in this case except touching the box
  virtual void compute_bounding_box() const;

  vsol_box_2d_sptr get_bounding_box() const { check_update_bounding_box(); return bounding_box_; }

  //: get id of object
  int get_id() const { return id_; }
  //: set id of object
  void set_id(int i) { id_ = i; }

  //: unprotect the object
  void un_protect() { ref_count--; iu_delete(this); }

  //---------------------------------------------------------------------------
  //: Clone `this': creation of a new object and initialization
  //  See Prototype pattern
  //---------------------------------------------------------------------------
  virtual vsol_spatial_object_2d* clone() const=0;

  // Binary I/O------------------------------------------------------------------

  //: Return a platform independent string identifying the class
  virtual vcl_string is_a() const=0;

  //: Return IO version number;
  short version() const;

  //: Binary save self to stream.
  virtual void b_write(vsl_b_ostream &os) const;

  //: Binary load self from stream.
  virtual void b_read(vsl_b_istream &is);

  // Tag Flag and ID methods

  //: set user flag 1-6
  inline void set_user_flag(unsigned int flag);
  inline bool get_user_flag(unsigned int flag);
  inline void unset_user_flag(unsigned int flag);
  inline void set_tagged_union_flag();
  inline bool get_tagged_union_flag();
  inline void unset_tagged_union_flag();
  inline int get_tag_id();
  inline void set_tag_id(int id);

  // Data Control--------------------------------------------------------------

 public:
  void protected_destroy();

  // bounding box accessors

  void check_update_bounding_box() const;  // Test consistency of bound
  //: grow to the largest dim. of this and comp_box, i.e., take the convex union
  void grow_minmax_bounds(vsol_box_2d & comp_box) const; // mutable const

  double get_min_x() const;
  double get_max_x() const;
  double get_min_y() const;
  double get_max_y() const;

  //: set the bounding box to a single point, discarding the old bounding box
  // This is a "const" method since the bounding box is a "mutable" data member:
  // calculating the bounding box does not change the object.
  void set_bounding_box(double x, double y) const;

  //: add a point to the bounding box and take the convex union
  // This is a "const" method since the bounding box is a "mutable" data member:
  // calculating the bounding box does not change the object.
  void add_to_bounding_box(double x, double y) const;

  // Operators

  virtual bool operator==(vsol_spatial_object_2d const& obj) const { return this==&obj; }
  bool operator!=(vsol_spatial_object_2d const& obj) { return !(*this==obj); }

  //-------------------------------------------------------------------------

  //---------------------------------------------------------------------------
  //: The same behavior than dynamic_cast<>.
  // Needed because VXL is not necessarily compiled with -frtti
  //---------------------------------------------------------------------------
  vsol_spatial_object_2d* cast_to_spatial_object_2d() {return this;}
  vsol_spatial_object_2d const* cast_to_spatial_object_2d() const {return this;}
  virtual vtol_topology_object* cast_to_topology_object() {return 0;}
  virtual vtol_topology_object const* cast_to_topology_object()const{return 0;}
  virtual vsol_group_2d *cast_to_group() {return 0;}
  virtual vsol_group_2d const* cast_to_group() const {return 0;}
  virtual vsol_curve_2d *cast_to_curve() {return 0;}
  virtual vsol_curve_2d const* cast_to_curve() const {return 0;}
  virtual vsol_region_2d* cast_to_region() { return 0; }
  virtual vsol_region_2d const* cast_to_region() const { return 0; }
  virtual vsol_point_2d* cast_to_point() { return 0; }
  virtual vsol_point_2d const* cast_to_point() const { return 0; }

  virtual void print(vcl_ostream &strm=vcl_cout) const { describe(strm); }
  virtual void describe(vcl_ostream& =vcl_cout, int /*blanking*/=0) const { not_applicable("describe"); }

  friend inline vcl_ostream &operator<<(vcl_ostream &, vsol_spatial_object_2d const&);
  friend inline vcl_ostream &operator<<(vcl_ostream &, vsol_spatial_object_2d const*);
};

// inline member functions

inline void vsol_spatial_object_2d::set_tag_id(int id)
{
  //     ( set the new id bits)  or (save just the flag bits from the tag_)
  tag_ = ( (id & VSOL_DEXID_BITS)     |  ( tag_ & VSOL_FLAG_BITS ));
}

//: set a flag for a spatial object; flag can be VSOL_FLAG[1-6]
inline void vsol_spatial_object_2d::set_user_flag(unsigned int flag)
{
  tag_ =  (tag_ | flag);
}

//: check if a flag is set for a spatial object; flag can be VSOL_FLAG[1-6]
inline bool vsol_spatial_object_2d::get_user_flag(unsigned int flag)
{
  return (tag_ & flag) != 0;
}

//: un-set a flag for a spatial object; flag can be VSOL_FLAG[1-6]
inline void vsol_spatial_object_2d::unset_user_flag(unsigned int flag)
{
  tag_ = ( tag_ & (~flag) );
}

//: set the flag used by TAGGED_UNION.
inline void vsol_spatial_object_2d::set_tagged_union_flag()
{
  set_user_flag(VSOL_UNIONBIT);
}

//: check if the flag used by TAGGED_UNION is set.
inline bool vsol_spatial_object_2d::get_tagged_union_flag()
{
  return get_user_flag(VSOL_UNIONBIT);
}

//: un-set the flag used by TAGGED_UNION.
inline void vsol_spatial_object_2d::unset_tagged_union_flag()
{
  unset_user_flag(VSOL_UNIONBIT);
}

inline int vsol_spatial_object_2d::get_tag_id()
{
  return tag_ & VSOL_DEXID_BITS;
}

inline vcl_ostream &operator<<(vcl_ostream &strm, vsol_spatial_object_2d const& so)
{
  ((vsol_spatial_object_2d const*)&so)->print(strm);
  return strm;
}

inline vcl_ostream &operator<<(vcl_ostream &strm, vsol_spatial_object_2d const* so)
{
  if (so)
    ((vsol_spatial_object_2d const*)so)->print(strm);
  else
    strm << "NULL Spatial Object.\n";
  return strm;
}

//: Stream output operator for class pointer
inline void vsl_print_summary(vcl_ostream& os, vsol_spatial_object_2d const* so)
{
  os << so;
}

//: Allows derived class to be loaded by base-class pointer
//  A loader object exists which is invoked by calls
//  of the form "vsl_b_read(os,base_ptr)".  This loads derived class
//  objects from the disk, places them on the heap and
//  returns a base class pointer.
//  In order to work the loader object requires
//  an instance of each derived class that might be
//  found.  This function gives the model class to
//  the appropriate loader.
void vsl_add_to_binary_loader(vsol_spatial_object_2d const& b);

#endif // vsol_spatial_object_2d_h_
