#ifndef vtol_zero_chain_h
#define vtol_zero_chain_h
//-----------------------------------------------------------------------------
//
// .NAME    vtol_zero_chain - Represents a set of Vertices
// .LIBRARY vtol
// .HEADER  gel package
// .INCLUDE vtol/vtol_zero_chain.h
// .FILE    vtol_zero_chain.cxx
//
// .SECTION Description
//  The vtol_zero_chain class is used to represent a set of Vertices on
//  a topological structure. A vtol_zero_chain maintains only the inferiors and
//  superiors. It is the topological inferior of an Edge.
//
// .SECTION Author
//     Patricia A. Vrobel
//     PTU - ported may 2000
//
// .SECTION Modifications:
//   JLM Dec 1995, Added timeStamp (Touch) to operations which affect bounds.
//   02-26-97 Added implementation for virtual Transform() - Peter Vanroose
//
//-----------------------------------------------------------------------------

#include <vtol/vtol_topology_object.h>
#include <vcl_vector.h>
#include <vtol/vtol_vertex.h>

class vtol_vertex;
class vtol_edge;
class vtol_one_chain;
class vtol_face;
class vtol_two_chain;
class vtol_block;

class vtol_zero_chain
  : public vtol_topology_object
{
public:
  //***************************************************************************
  // Initialization
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Default constructor. Empty zero-chain
  //---------------------------------------------------------------------------
  explicit vtol_zero_chain(void);

  //---------------------------------------------------------------------------
  //: Constructor from two vertices (to make an edge creation easier)
  //  REQUIRE: v1.ptr()!=0 and v2.ptr()!=0 and v1.ptr()!=v2.ptr()
  //---------------------------------------------------------------------------
  explicit vtol_zero_chain(vtol_vertex &v1,
                           vtol_vertex &v2);

  //---------------------------------------------------------------------------
  //: Constructor from an array of vertices
  //  REQUIRE: new_vertices.size()>0
  //---------------------------------------------------------------------------
  explicit
  vtol_zero_chain(const vcl_vector<vtol_vertex_sptr> &new_vertices);

  //---------------------------------------------------------------------------
  //: Copy constructor. Copy the vertices and the links
  //---------------------------------------------------------------------------
  vtol_zero_chain(const vtol_zero_chain &other);

  //---------------------------------------------------------------------------
  //: Destructor
  //---------------------------------------------------------------------------
  virtual ~vtol_zero_chain();

  //---------------------------------------------------------------------------
  //: Clone `this': creation of a new object and initialization
  //  See Prototype pattern
  //---------------------------------------------------------------------------
  virtual vsol_spatial_object_3d_sptr clone(void) const;

  //---------------------------------------------------------------------------
  //: Return the topology type
  //---------------------------------------------------------------------------
  virtual vtol_topology_object_type topology_type(void) const;

  //---------------------------------------------------------------------------
  //: Return the first vertex of `this'. If it does not exist, return 0
  //---------------------------------------------------------------------------
  virtual vtol_vertex_sptr v0(void) const;

  //***************************************************************************
  // Replaces dynamic_cast<T>
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Return `this' if `this' is a zero_chain, 0 otherwise
  //---------------------------------------------------------------------------
  virtual const vtol_zero_chain *cast_to_zero_chain(void) const;

  //---------------------------------------------------------------------------
  //: Return `this' if `this' is a zero_chain, 0 otherwise
  //---------------------------------------------------------------------------
  virtual vtol_zero_chain *cast_to_zero_chain(void);

  //***************************************************************************
  // Status report
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Is `inferior' type valid for `this' ?
  //---------------------------------------------------------------------------
  virtual bool
  valid_inferior_type(const vtol_topology_object &inferior) const;

  //---------------------------------------------------------------------------
  //: Is `superior' type valid for `this' ?
  //---------------------------------------------------------------------------
  virtual bool
  valid_superior_type(const vtol_topology_object &superior) const;

  //---------------------------------------------------------------------------
  //: Return the length of the zero-chain
  //---------------------------------------------------------------------------
  virtual int length(void) const;

  //---------------------------------------------------------------------------
  //: Is `this' equal to `other' ?
  //---------------------------------------------------------------------------
  virtual bool operator==(const vtol_zero_chain &other) const;
  bool operator==(const vsol_spatial_object_3d& obj) const; // virtual of vsol_spatial_object_3d

  virtual void print(vcl_ostream &strm=vcl_cout) const;
  virtual void describe(vcl_ostream &strm=vcl_cout,
                        int blanking=0) const;

  // : Warning - should not be used by clients
protected:
  virtual vcl_vector<vtol_vertex*> *compute_vertices(void);
  virtual vcl_vector<vtol_edge*> *compute_edges(void);
  virtual vcl_vector<vtol_zero_chain*> *compute_zero_chains(void);
  virtual vcl_vector<vtol_one_chain*> *compute_one_chains(void);
  virtual vcl_vector<vtol_face*> *compute_faces(void);
  virtual vcl_vector<vtol_two_chain*> *compute_two_chains(void);
  virtual vcl_vector<vtol_block*> *compute_blocks(void);
};

inline bool operator!=(vtol_zero_chain const &a, vtol_zero_chain const &b) { return !(a == b); }

#endif // vtol_zero_chain_h
