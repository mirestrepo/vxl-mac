#include "vtol_edge_2d.h"

#include <vcl_cassert.h>
#include <vtol/vtol_zero_chain.h>
#include <vtol/vtol_one_chain.h>
#include <vtol/vtol_macros.h>
#include <vtol/vtol_list_functions.h>
#include <vsol/vsol_curve_2d.h>
#include <vsol/vsol_line_2d.h>

//***************************************************************************
// Initialization
//***************************************************************************

//---------------------------------------------------------------------------
//: Default constructor. Empty edge. Not a valid edge.
//---------------------------------------------------------------------------
vtol_edge_2d::vtol_edge_2d(void)
{
  _curve=0;
  _v1=0;
  _v2=0;
}

//---------------------------------------------------------------------------
//: Constructor from the two endpoints `new_v1', `new_v2' and from a
//    curve `new_curve'. If `new_curve' is 0, a line is created from
//    `new_v1' and `new_v2'.
//---------------------------------------------------------------------------
vtol_edge_2d::vtol_edge_2d(vtol_vertex_2d &new_v1,
                           vtol_vertex_2d &new_v2,
                           const vsol_curve_2d_sptr &new_curve)
{
  vtol_topology_object *zc;

  if (!new_curve)
    _curve=new vsol_line_2d(new_v1.point(),new_v2.point());
  else
    _curve=new_curve;
  _v1=&new_v1;
  _v2=&new_v2;

  zc=new vtol_zero_chain(new_v1,new_v2);

  link_inferior(*zc);
}

//---------------------------------------------------------------------------
// Copy constructor
//---------------------------------------------------------------------------

//:
// Copy constructor for an vtol_edge_2d. This methods performs a deep copy of
// the elements of the old vtol_edge_2d, olde, and sets the corresponding member
// data of the new vtol_edge_2d.

vtol_edge_2d::vtol_edge_2d(const vtol_edge_2d &other)
{
  _curve=0;
  vsol_curve_2d_sptr tmp_curve;

  vcl_vector<vtol_topology_object_sptr>::const_iterator i;

  for(i=other.inferiors()->begin();i!=other.inferiors()->end();++i){
    vtol_zero_chain_sptr zc = (*i)->clone()->cast_to_topology_object()->cast_to_zero_chain();
    link_inferior(*zc);
  }

  set_vertices_from_zero_chains();
  if (other._curve)
    {
      vsol_spatial_object_2d_sptr sr = other._curve->clone();
      _curve=(vsol_curve_2d *)(sr.ptr());
      // make sure the geometry and Topology are in sync
      if (_v1)
        {
          if(_v1->cast_to_vertex_2d()){
            _curve->set_p0(_v1->cast_to_vertex_2d()->point());
            _curve->touch();
          }
        }
      if (_v2)
        {
          if(_v1->cast_to_vertex_2d()){
            _curve->set_p1(_v2->cast_to_vertex_2d()->point());
            _curve->touch();
          }
        }
    }
  touch();
}

//---------------------------------------------------------------------------
//: Constructor from a zero-chain.
//---------------------------------------------------------------------------
//
// Constructor for an vtol_edge_2d. If the vtol_zero_chain has two vertices , then the
// first and last vertices of the vtol_zero_chain are used for endpoints and
// an ImplicitLine is assumed to be the curve.  Otherwise, the all data
// (_v1, _v2, _curve) are set to NULL.  The vtol_zero_chain, newchain, becomes
// the Inferior of the vtol_edge_2d.

vtol_edge_2d::vtol_edge_2d(vtol_zero_chain &new_zero_chain)
{
  link_inferior(new_zero_chain);
  set_vertices_from_zero_chains();
  if(new_zero_chain.numinf()==2 && _v1->cast_to_vertex_2d()
     && _v2->cast_to_vertex_2d())
    // Safe to assume that it is an Implicit Line.
    _curve=new vsol_line_2d(_v1->cast_to_vertex_2d()->point(),
                            _v2->cast_to_vertex_2d()->point());
  else
    // User must set the type of curve needed.
    // Since guessing could get confusing.
    // So NULL indicates an edge of unknown type.
    _curve=0;
  touch();
}

//:
// Constructor for an vtol_edge_2d. The list of zero_chains, newchains, is
// assumed to be ordered along an edge. This method assigns the first
// vertex in the chain list to _v1, and assigns the last vertex in the
// chain list to _v2. No assumptions are made as to the curve type. The
// data member, _curve is left to be NULL.

vtol_edge_2d::vtol_edge_2d(zero_chain_list &newchains)
{
  // 1) Link the inferiors.
  zero_chain_list::iterator i;

  for (i=newchains.begin();i!= newchains.end();++i )
    link_inferior(*(*i));

  // 2) Set _v1 and _v2;

  set_vertices_from_zero_chains();
  _curve=0;
}

//:
// Constructor for a Linear vtol_edge_2d.  The coordinates, (x1, y1, z1),
// determine vtol_vertex_2d, _v1.  The coordinates, (x2, y2, z2), determine _v2.
// If curve is NULL, an ImplicitLine is generated for the vtol_edge_2d.

vtol_edge_2d::vtol_edge_2d(double x1,
                           double y1,
                           double x2,
                           double y2,
                           vsol_curve_2d_sptr curve)
{
  _v1=new vtol_vertex_2d(x1,y1);
  _v2=new vtol_vertex_2d(x2,y2);
  if (!curve)
    if(_v1->cast_to_vertex_2d() && _v2->cast_to_vertex_2d()){
      _curve=new vsol_line_2d(_v1->cast_to_vertex_2d()->point(),
                              _v2->cast_to_vertex_2d()->point());
    }
  else
    _curve=(vsol_curve_2d*)(curve->clone().ptr());

  vtol_zero_chain *inf=new vtol_zero_chain(*_v1,*_v2);
  link_inferior(*inf);
}

//:
// Constructor for an vtol_edge_2d from a Curve. If edgecurve is of ImplicitLine
// type, vertex locations for endpoints, _v1 and _v2, are computed from
// the ImplicitLine parameters.  If edgecurve is of any other type, _v1
// and _v2 are left as NULL.
// (Actually, this description is wrong. The endpoints are retreived
// from the curve, regardless of its type. -JLM)

vtol_edge_2d::vtol_edge_2d(vsol_curve_2d &edgecurve)
{
  vtol_zero_chain *newzc;
  if (_curve)
    {
      //  _v1 = new vtol_vertex_2d(&_curve->get_start_point());
      // _v2 = new vtol_vertex_2d(&_curve->get_end_point());
      newzc=new vtol_zero_chain(*_v1,*_v2);
    } else {
      _v1=0;
      _v2=0;
      _curve=0;
      newzc=new vtol_zero_chain();
    }
  link_inferior(*newzc);
}

//---------------------------------------------------------------------------
//: Clone `this': creation of a new object and initialization
// See Prototype pattern
//---------------------------------------------------------------------------
vsol_spatial_object_3d_sptr vtol_edge_2d::clone(void) const
{
  return new vtol_edge_2d(*this);
}

//---------------------------------------------------------------------------
//: Return the curve associated to `this'
//---------------------------------------------------------------------------
vsol_curve_2d_sptr vtol_edge_2d::curve(void) const
{
  return _curve;
}


//---------------------------------------------------------------------------
//: Set the curve with `new_curve'
//---------------------------------------------------------------------------
void vtol_edge_2d::set_curve(vsol_curve_2d &new_curve)
{
  _curve=&new_curve;
  touch(); //Update timestamp
}

//---------------------------------------------------------------------------
// Destructor
//---------------------------------------------------------------------------
vtol_edge_2d::~vtol_edge_2d()
{
}


//---------------------------------------------------------------------------
//: Return `this' if `this' is an edge, 0 otherwise
//---------------------------------------------------------------------------
const vtol_edge_2d * vtol_edge_2d::cast_to_edge_2d(void) const
{
  return this;
}

//---------------------------------------------------------------------------
//: Return `this' if `this' is an edge, 0 otherwise
//---------------------------------------------------------------------------
vtol_edge_2d * vtol_edge_2d::cast_to_edge_2d(void)
{
  return this;
}

// ******************************************************
//
//    Operators
//

// operators

bool vtol_edge_2d::operator==(const vtol_edge_2d &other) const
{
  if (this==&other) return true;

  if ( (_curve && !other._curve) ||
       (!_curve && other._curve) )
    return false;

  if (_curve && (*_curve)!=(*other._curve))
    return false;

  if (!(*_v1==*(other._v1)) || !(*_v2==*(other._v2))) // ((*_v1!=*(other._v1)) || (*_v2!=*(other._v2)))
    return false;

  vtol_zero_chain_sptr zc1=zero_chain();
  vtol_zero_chain_sptr zc2=other.zero_chain();
  if (!zc1||!zc2)
    return false;
  return *zc1==*zc2;
}

//: edge equality
bool vtol_edge_2d::operator==(const vtol_edge &other) const
{
  if (other.cast_to_edge_2d())
    return *this == (vtol_edge_2d const&) other;
  else
    return false;
}

//: spatial object equality

bool vtol_edge_2d::operator==(const vsol_spatial_object_3d& obj) const
{
  return
   obj.spatial_type() == vsol_spatial_object_3d::TOPOLOGYOBJECT &&
   ((vtol_topology_object const&)obj).topology_type() == vtol_topology_object::EDGE
  ? *this == (vtol_edge_2d const&) (vtol_topology_object const&) obj
  : false;
}

// ******************************************************
//
//    Inferior/Superior Accessor Functions
//
// ******************************************************
//
//    I/O methods
//

//:
// This method outputs all edge information to the vcl_ostream, strm.  It
// indents various levels of output by the number given in blanking.
void vtol_edge_2d::describe(vcl_ostream &strm,
                            int blanking) const
{
  for (int i1=0; i1<blanking; ++i1) strm << ' ';
  print(strm);
  for (int i2=0; i2<blanking; ++i2) strm << ' ';
  if(_v1) {
    _v1->print(strm);
  } else {
    strm << "Null vertex 1" << vcl_endl;
  }
  for (int i3=0; i3<blanking; ++i3) strm << ' ';
  if(_v2) {
    _v2->print(strm);
  } else {
    strm << "Null vertex 2" << vcl_endl;
  }
}

//:
// This method outputs a brief vtol_edge_2d info with vtol_edge_2d object address.
void vtol_edge_2d::print(vcl_ostream &strm) const
{
   strm<<"<vtol_edge_2d  "<<"  "<<(void const *)this <<"> with id "<<get_id()<<vcl_endl;
}

bool vtol_edge_2d::compare_geometry(const vtol_edge &other) const
{
  // we want to compare geometry

  if(other.cast_to_edge_2d()){
    bool result = (*_curve)== *(other.cast_to_edge_2d()->curve());
    return result;
  }
  return false;
}
