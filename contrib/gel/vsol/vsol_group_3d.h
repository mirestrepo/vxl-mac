// This is gel/vsol/vsol_group_3d.h
#ifndef vsol_group_3d_h_
#define vsol_group_3d_h_
//*****************************************************************************
//:
// \file
// \brief Group of spatial objects in a 3D space
//
// \author Fran�ois BERTEL
// \date   2000/05/03
//
// \verbatim
//  Modifications
//   2000/05/03 Fran�ois BERTEL Creation
//   2000/06/17 Peter Vanroose  Implemented all operator==()s and type info
// \endverbatim
//*****************************************************************************

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vsol/vsol_spatial_object_3d.h>
#include <vcl_list.h>

class vsol_group_3d : public vsol_spatial_object_3d
{
  //***************************************************************************
  // Data members
  //***************************************************************************

  //---------------------------------------------------------------------------
  // Description: Set of objects that `this' contains
  //---------------------------------------------------------------------------
  vcl_list<vsol_spatial_object_3d_sptr> *storage_;

 public:
  //***************************************************************************
  // Initialization
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Default Constructor: group with no child
  //---------------------------------------------------------------------------
  vsol_group_3d(void);

  //---------------------------------------------------------------------------
  //: Copy constructor.
  //  The objects of the group are not duplicated
  //---------------------------------------------------------------------------
  vsol_group_3d(const vsol_group_3d &other);

  //---------------------------------------------------------------------------
  //: Destructor
  //  The objects of the group are not deleted
  //---------------------------------------------------------------------------
  virtual ~vsol_group_3d();

  //---------------------------------------------------------------------------
  //: Clone `this': creation of a new object and initialization
  //  See Prototype pattern
  //---------------------------------------------------------------------------
  virtual vsol_spatial_object_3d_sptr clone(void) const;

  //***************************************************************************
  // Access
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Return the object `i'
  //  REQUIRE: i>=0 and i<size()
  //---------------------------------------------------------------------------
  vsol_spatial_object_3d_sptr object(int i) const;

  //***************************************************************************
  // Status report
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Return the real type of a group. It is a SPATIALGROUP
  //---------------------------------------------------------------------------
  enum vsol_spatial_object_3d_type spatial_type(void) const;

  //---------------------------------------------------------------------------
  //: Compute the bounding box of `this'
  //  REQUIRE: size()>0
  //---------------------------------------------------------------------------
  virtual void compute_bounding_box(void) const; // virtual of vsol_spatial_object_3d

  //---------------------------------------------------------------------------
  //: Return the number of direct children of the group
  //---------------------------------------------------------------------------
  unsigned int size(void) const { return storage_->size(); }

  //---------------------------------------------------------------------------
  //: Return the number of objects of the group
  //---------------------------------------------------------------------------
  unsigned int deep_size(void) const;

  //---------------------------------------------------------------------------
  //: Is `new_object' a child (direct or not) of `this' ?
  //---------------------------------------------------------------------------
  bool is_child(const vsol_spatial_object_3d_sptr &new_object) const;

  //***************************************************************************
  // Element change
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Add an object `new_object'to `this'
  //---------------------------------------------------------------------------
  void add_object(const vsol_spatial_object_3d_sptr &new_object);

  //***************************************************************************
  // Removal
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Remove object `i' of `this' (not delete it)
  //  REQUIRE: i>=0 and i<size()
  //---------------------------------------------------------------------------
  void remove_object(const int i);

  //---------------------------------------------------------------------------
  //: The same behavior than dynamic_cast<>.
  //  Needed because VXL is not compiled with -frtti :-(
  //---------------------------------------------------------------------------
  virtual const vsol_group_3d *cast_to_group(void) const { return this; }
  virtual vsol_group_3d *cast_to_group(void) { return this; }
};

#endif // vsol_group_3d_h_
