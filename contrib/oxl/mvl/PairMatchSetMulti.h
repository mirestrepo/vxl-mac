#ifndef PairMatchSetMulti_h_
#define PairMatchSetMulti_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME	PairMatchSetMulti - Multimap of ints
// .LIBRARY	MViewBasics
// .HEADER	MultiView Package
// .INCLUDE	mvl/PairMatchSetMulti.h
// .FILE	PairMatchSetMulti.cxx
//
// .SECTION Description
//    PairMatchSetMulti is a list of tuples (i1, i2) which allows
//    efficient O(log n) indexing by I1, and O(n) by i2.
//
// .SECTION Author
//     Andrew W. Fitzgibbon, Oxford RRG, 16 Sep 96
//
//-----------------------------------------------------------------------------

class PairMatchSetMulti {
public:
  // Constructors/Destructors--------------------------------------------------

  PairMatchSetMulti();
  PairMatchSetMulti(const PairMatchSetMulti& that);
 ~PairMatchSetMulti();

  PairMatchSetMulti& operator=(const PairMatchSetMulti& that);

  // Operations----------------------------------------------------------------
  void add_match(int i1, int i2);
};

#endif // PairMatchSetMulti_h_
