#ifndef PairMatchSetCorner_h_
#define PairMatchSetCorner_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME    PairMatchSetCorner - Matches between corners
// .LIBRARY MViewBasics
// .HEADER  MultiView Package
// .INCLUDE mvl/PairMatchSetCorner.h
// .FILE    PairMatchSetCorner.cxx
//
// .SECTION Description
//    PairMatchSetCorner is a subclass of PairMatchSet that stores matches
//    between corner features.
//
// .SECTION Author
//     Andrew W. Fitzgibbon, Oxford RRG, 09 Aug 96
//
// .SECTION Modifications:
//     <none yet>
//
//-----------------------------------------------------------------------------

//////////////#include <cool/decls.h>
#include <mvl/HomgPoint2D.h>
#include "PairMatchSet.h"

class HomgInterestPointSet;
class PairMatchSetCorner : public PairMatchSet {
public:
  // Constructors/Destructors--------------------------------------------------

  PairMatchSetCorner();
  // deprecated PairMatchSetCorner(HomgInterestPointSet const* corners1, HomgInterestPointSet const* corners2);
  PairMatchSetCorner(HomgInterestPointSet const* corners1, HomgInterestPointSet const* corners2);
  PairMatchSetCorner(const PairMatchSetCorner& that);
  PairMatchSetCorner& operator=(const PairMatchSetCorner& that);
 ~PairMatchSetCorner();

  // Computations--------------------------------------------------------------

  // Data Access---------------------------------------------------------------
  void extract_matches(vcl_vector <HomgPoint2D>& points1, vcl_vector <int>& corner_index_1,
                       vcl_vector <HomgPoint2D>& points2, vcl_vector <int>& corner_index_2) const;

  void extract_matches(vcl_vector <HomgPoint2D>& points1, vcl_vector <HomgPoint2D>& points2) const;

  //: Clear all matches and then set only those for which the corresponding
  // inliers flag is set.
  void set(const vcl_vector<bool>& inliers,
           const vcl_vector<int>&  corner_index_1,
           const vcl_vector<int>&  corner_index_2);

  // Data Control--------------------------------------------------------------
  void set(HomgInterestPointSet const* corners1, HomgInterestPointSet const* corners2);

//: Return the set of corners within which the i1 indices point
  HomgInterestPointSet const* get_corners1() const { return _corners1; }
  // HomgInterestPointSet* get_corners1() { return _corners1; }

//: Return the set of corners within which the i2 indices point
  HomgInterestPointSet const* get_corners2() const { return _corners2; }
  //   HomgInterestPointSet* get_corners2() { return _corners2; }

private:
  HomgInterestPointSet const* _corners1;
  HomgInterestPointSet const* _corners2;
};

#endif // PairMatchSetCorner_h_
