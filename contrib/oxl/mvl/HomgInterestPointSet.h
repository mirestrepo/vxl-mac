#ifndef HomgInterestPointSet_h_
#define HomgInterestPointSet_h_
#ifdef __GNUC__
#pragma interface
#endif
//
// .NAME	HomgInterestPointSet - Set of interest points on an image
// .LIBRARY	MViewBasics
// .HEADER	MultiView Package
// .INCLUDE	mvl/HomgInterestPointSet.h
// .FILE	HomgInterestPointSet.cxx
//
// .SECTION Description
//    HomgInterestPointSet holds a set of corners (or points of interest) computed
//    from an image.  The current implementation provides a moderately
//    abstract interface, but assumes that it is efficient to associate
//    an index with each corner.  I *know* this ought to be elsewhere.
//
// .SECTION Author
//     Andrew W. Fitzgibbon, Oxford RRG, 17 Aug 96
//
// .SECTION Modifications:
//     Peter Vanroose - 27 aug.97 - moved vcl_vector<HomgInterestPoint> instantiation to Templates package
//
//-----------------------------------------------------------------------------

#include <vcl_vector.h>

#include <vnl/vnl_int_2.h>
#include <vnl/vnl_double_2.h>

#include <mvl/HomgPoint2D.h>
#include <mvl/HomgMetric.h>

class vil_image;
class HomgPoint2D;
class ImageMetric;
class HomgInterestPointSetData;
class HomgInterestPoint;

class HomgInterestPointSet {
public:
  // Constructors/Destructors--------------------------------------------------

  HomgInterestPointSet();
  HomgInterestPointSet(const HomgMetric&);
  HomgInterestPointSet(const char* filename, const HomgMetric& = 0);
  HomgInterestPointSet(const vcl_vector<HomgPoint2D>&, ImageMetric* conditioner);
  HomgInterestPointSet(const HomgInterestPointSet& that);
 ~HomgInterestPointSet();

  HomgInterestPointSet& operator=(const HomgInterestPointSet& that);

  // Operations----------------------------------------------------------------
  // void set(const IUPointGroup&, const HomgMetric& c);

  // Computations--------------------------------------------------------------

  // Data Access---------------------------------------------------------------
  unsigned size() const;

  // -- Return i'th interest pt.
  const HomgPoint2D& operator[](int i) const { return get_homg(i); }

  HomgInterestPoint& get(int i);
  HomgInterestPoint const& get(int i) const;
  vnl_double_2 const& get_2d(int i) const;
  vnl_vector_fixed<int,2> const& get_int(int i) const;
  HomgPoint2D const& get_homg(int i) const;
  float              get_mean_intensity(int i) const;

  vcl_vector<HomgPoint2D> const & get_homg_points() const;

  const ImageMetric* get_conditioner() const { return (const ImageMetric*)_conditioner; }
  void set_conditioner(const HomgMetric& c);

  // Data Control--------------------------------------------------------------
  bool add(double x, double y); // image coords
  bool add(const HomgPoint2D&);
  bool add(const HomgInterestPoint&);
  bool add_preconditioned(const HomgPoint2D&);
  void set_image(vil_image const& image);
  void clear();

  // Input/Output--------------------------------------------------------------
  bool read(const char* filename, const HomgMetric& c = 0);
  bool read(const char* filename, vil_image const& src, const HomgMetric& c = 0);
  bool write(const char* filename) const;

  bool read(istream& f, const ImageMetric* c);
  bool write(ostream& f, const ImageMetric* c) const;

protected:
  // Data Members--------------------------------------------------------------
  HomgInterestPointSetData* _data;
  HomgMetric _conditioner;

  void init_conditioner(const HomgMetric& c = 0);
  void delete_conditioner();
};

#endif // HomgInterestPointSet_h_
