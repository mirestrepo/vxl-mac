//-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation "PairMatchSet2D3D.h"
#endif
//
// Class: PairMatchSet2D3D
// Author: Andrew W. Fitzgibbon, Oxford RRG
// Created: 19 Sep 96
// Modifications:
//
//-----------------------------------------------------------------------------

#include "PairMatchSet2D3D.h"
#include <mvl/HomgInterestPointSet.h>
#include <mvl/PairMatchSetCorner.h>
#include <mvl/ProjStructure.h>

// Default ctor
PairMatchSet2D3D::PairMatchSet2D3D()
{
  _corners = 0;
  _structure = 0;
}

// Copy ctor
PairMatchSet2D3D::PairMatchSet2D3D(const PairMatchSet2D3D& that)
{
  operator=(that);
}

PairMatchSet2D3D::PairMatchSet2D3D(const HomgInterestPointSet* corners, vcl_vector<HomgPoint3D>* structure)
{
  set(corners, structure);
}

// Assignment
PairMatchSet2D3D& PairMatchSet2D3D::operator=(const PairMatchSet2D3D& )
{
  cerr << "PairMatchSet2D3D::operator= not implemented\n";
  return *this;
}

// Destructor
PairMatchSet2D3D::~PairMatchSet2D3D()
{
}

void PairMatchSet2D3D::set(const HomgInterestPointSet* corners, vcl_vector<HomgPoint3D>* structure)
{
  _corners = corners;
  _structure = structure;
  set_size(_corners->size());
}

void PairMatchSet2D3D::set(int num_corners, vcl_vector<HomgPoint3D>* structure)
{
  _corners = 0;
  _structure = structure;
  set_size(num_corners);
}

void PairMatchSet2D3D::set_from(const PairMatchSet2D3D& otherframe_to_3d, const PairMatchSetCorner& otherframe_to_this)
{
  _corners = otherframe_to_this.get_corners2();
  _structure = otherframe_to_3d.get_structure();
  set_size(otherframe_to_this.size());

  clear();
  for(PairMatchSetCorner::iterator match = otherframe_to_this; match; match.next()) {
    int corner1 = match.get_i1();
    int corner2 = match.get_i2();
    int structure1 = otherframe_to_3d.get_match_12(corner1);
    add_match(corner2, structure1);
  }

}

HomgMetric PairMatchSet2D3D::get_conditioner() const
{
  if (!_corners) {
    cerr << "PairMatchSet2D3D::get_conditioner() WARNING _corners not set!\n";
    return 0;
  }
  return _corners->get_conditioner();
}

const HomgPoint2D& PairMatchSet2D3D::get_point_2d(int i1) const
{
  if (!_corners) {
    static HomgPoint2D dummy;
    cerr << "PairMatchSet2D3D::get_point_2d() WARNING _corners not set!\n";
    return dummy;
  }
  return _corners->get_homg(i1);
}

const HomgPoint3D& PairMatchSet2D3D::get_point_3d(int i2) const
{
  return _structure->operator[](i2);
}

const HomgInterestPointSet* PairMatchSet2D3D::get_corners() const
{
  if (!_corners) {
    cerr << "PairMatchSet2D3D::get_point_2d() WARNING _corners not set!\n";
    return 0;
  }
  return _corners;
}
