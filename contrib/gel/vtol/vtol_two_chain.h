#ifndef vtol_two_chain_H
#define vtol_two_chain_H
//-----------------------------------------------------------------------------
//
// .NAME        vtol_two_chain - Represents a set of vtol_face
// .LIBRARY     vtol
// .HEADER      gel  package
// .INCLUDE     vtol/vtol_two_chain.h
// .FILE        vtol_two_chain.cxx
//
// .SECTION Description
//  The vtol_two_chain class is used to represent a set of vtol_faces on a topological
//  structure.  A vtol_two_chain consists of its inferior onechains and the superiors
//  on which it lies.  A vtol_two_chain may or may not be an ordered cycle.  If
//  the chain of vtol_face encloses a volume, then the vtol_two_chain may be used as
//  the boundary of a topological vtol_block in a 3D structure.
//
// .SECTION Author
//     Patricia A. Vrobel
//
// .SECTION Modifications:
//               JLM December 1995, Added timeStamp (Touch) to
//                   operations which affect bounds.
//               JLM December 1995, no local method for ComputeBoundingBox
//                   Should use vtol_face geometry recursively to be proper.
//                   Currently reverts to bounds on vertices from
//                   vtol_topology_object::ComputeBoundingBox()
//               PTU ported to vxl May 2000
//-----------------------------------------------------------------------------
#include <vtol/vtol_two_chain_sptr.h>

#include <vcl_vector.h>
//#include <vtol/vtol_topology_object.h>
//#include <vtol/vtol_hierarchy_node.h>
#include <vtol/vtol_chain.h>

class vtol_vertex;
class vtol_edge;
class vtol_zero_chain;
class vtol_one_chain;
class vtol_face;
class vtol_block;

class vtol_two_chain
//: public vtol_topology_object,
  : public vtol_chain
{
public:
  //***************************************************************************
  // Initialization
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Default constructor
  //---------------------------------------------------------------------------
  explicit vtol_two_chain(void);

  //---------------------------------------------------------------------------
  //: Constructor
  //---------------------------------------------------------------------------
  explicit vtol_two_chain(int num_face);

  //---------------------------------------------------------------------------
  //: Constructor
  //---------------------------------------------------------------------------
  explicit vtol_two_chain(face_list &, bool new_is_cycle=false);

  //---------------------------------------------------------------------------
  //: Constructor
  //---------------------------------------------------------------------------
  explicit vtol_two_chain(face_list &,
                          vcl_vector<signed char> &,
                          bool new_is_cycle=false);

  //---------------------------------------------------------------------------
  //: Copy constructor
  //---------------------------------------------------------------------------
  vtol_two_chain(vtol_two_chain const& other);

  //---------------------------------------------------------------------------
  //: Destructor
  //---------------------------------------------------------------------------
  virtual ~vtol_two_chain();

  //---------------------------------------------------------------------------
  //: Clone `this': creation of a new object and initialization
  //: See Prototype pattern
  //---------------------------------------------------------------------------
  virtual vsol_spatial_object_3d_sptr clone(void) const;

  virtual vtol_two_chain *
  copy_with_arrays(vcl_vector<vtol_topology_object_sptr> &verts,
                   vcl_vector<vtol_topology_object_sptr> &edges) const;
  // Accessors

  //---------------------------------------------------------------------------
  //: Return the topology type
  //---------------------------------------------------------------------------
  virtual vtol_topology_object_type topology_type(void) const;

  //: get the direction of the face

  signed char direction(vtol_face const& f) const;

  virtual vtol_face *face(int i) { return (vtol_face *)(_inferiors[i].ptr()); }

  //---------------------------------------------------------------------------
  //: Shallow copy with no links
  //---------------------------------------------------------------------------
  virtual vtol_topology_object *shallow_copy_with_no_links(void) const;

  virtual void add_superiors_from_parent(topology_list &);
  virtual void remove_superiors_of_parent(topology_list &);
  virtual void remove_superiors(void);
  virtual void update_superior_list_p_from_hierarchy_parent(void);

  virtual void add_face(vtol_face &,signed char);
  virtual void remove_face(vtol_face &);
  virtual void add_block(vtol_block &);
  virtual void remove_block(vtol_block &);

  //***************************************************************************
  // Replaces dynamic_cast<T>
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Return `this' if `this' is a two_chain, 0 otherwise
  //---------------------------------------------------------------------------
  virtual const vtol_two_chain *cast_to_two_chain(void) const;

  //---------------------------------------------------------------------------
  //: Return `this' if `this' is a two_chain, 0 otherwise
  //---------------------------------------------------------------------------
  virtual vtol_two_chain *cast_to_two_chain(void);

  //***************************************************************************
  // Status report
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Is `inferior' type valid for `this' ?
  //---------------------------------------------------------------------------
  virtual bool valid_inferior_type(vtol_topology_object const& inferior) const;

  //---------------------------------------------------------------------------
  //: Is `superior' type valid for `this' ?
  //---------------------------------------------------------------------------
  virtual bool valid_superior_type(vtol_topology_object const& superior) const;

  //---------------------------------------------------------------------------
  //: Is `chain_inf_sup' type valid for `this' ?
  //---------------------------------------------------------------------------
  virtual bool valid_chain_type(vtol_chain const& chain_inf_sup) const;

  //: network access methods

  virtual vertex_list *outside_boundary_vertices(void);
  virtual zero_chain_list *outside_boundary_zero_chains(void);
  virtual edge_list *outside_boundary_edges(void);
  virtual one_chain_list *outside_boundary_one_chains(void);
  virtual face_list *outside_boundary_faces(void);
  virtual two_chain_list *outside_boundary_two_chains(void);

  virtual two_chain_list *inferior_two_chains(void);
  virtual two_chain_list *superior_two_chains(void);

  //: Warning these methods should not be used by clients

  virtual vcl_vector<vtol_vertex*> *compute_vertices(void);
  virtual vcl_vector<vtol_edge*> *compute_edges(void);
  virtual vcl_vector<vtol_zero_chain*> *compute_zero_chains(void);
  virtual vcl_vector<vtol_one_chain*> *compute_one_chains(void);
  virtual vcl_vector<vtol_face*> *compute_faces(void);
  virtual vcl_vector<vtol_two_chain*> *compute_two_chains(void);
  virtual vcl_vector<vtol_block*> *compute_blocks(void);

  virtual vcl_vector<vtol_vertex*> *outside_boundary_compute_vertices(void);
  virtual vcl_vector<vtol_zero_chain*> *outside_boundary_compute_zero_chains(void);
  virtual vcl_vector<vtol_edge*> *outside_boundary_compute_edges(void);
  virtual vcl_vector<vtol_one_chain*> *outside_boundary_compute_one_chains(void);
  virtual vcl_vector<vtol_face*> *outside_boundary_compute_faces(void);
  virtual vcl_vector<vtol_two_chain*> *outside_boundary_compute_two_chains(void);

  virtual int num_faces(void) const { return numinf(); }

  virtual void correct_chain_directions(void);

  virtual bool operator==(vtol_two_chain const& other) const;
  bool operator==(vsol_spatial_object_3d const& obj) const; // virtual of vsol_spatial_object_3d

  virtual void print(vcl_ostream &strm=vcl_cout) const;
  virtual void describe_directions(vcl_ostream &strm=vcl_cout, int blanking=0) const;
  virtual void describe(vcl_ostream &strm=vcl_cout, int blanking=0) const;

  virtual bool break_into_connected_components(vcl_vector<vtol_topology_object *> &components);
};

#endif   // vtol_two_chain.h
