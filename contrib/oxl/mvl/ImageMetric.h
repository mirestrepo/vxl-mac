//-*- c++ -*-------------------------------------------------------------------
#ifndef ImageMetric_h_
#define ImageMetric_h_
#ifdef __GNUC__
#pragma interface
#endif
//
// Class : ImageMetric
//
// .SECTION Description
//    ImageMetric is the baseclass for classes that define how points in image
//    coordinates are converted to conditioned frames.  For simple systems this
//    will represent the mapping of the image plane to the unit square, but for
//    a fully calibrated camera, for example, this might include corrections for
//    radial lens distortion and other nonlinear effects.
//
//    The default implementation in this baseclass simply assumes an identity
//    mapping between the two coordinate systems.
//
//    A general convention that is sometimes useful is that points in homogeneous
//    coordinates have been conditioned, while nonhomogeneous primitives remain in
//    image coordinates. 
//
// .NAME        ImageMetric - Converting between image and metric coordinates.
// .LIBRARY     MViewBasics
// .HEADER	MultiView Package
// .INCLUDE     mvl/ImageMetric.h
// .FILE        ImageMetric.h
// .FILE        ImageMetric.C
// .SECTION Author
//     Andrew W. Fitzgibbon, Oxford RRG, 17 Aug 96
//
// .SECTION Modifications:
//     <none yet>
//
//-----------------------------------------------------------------------------

#include <vnl/vnl_matrix.h>
#if defined(VCL_GCC_27)
# include <vnl/vnl_double_2.h>
# include <mvl/HomgPoint2D.h>
# include <mvl/HomgLineSeg2D.h>
# include <mvl/HomgLine2D.h>
# include <mvl/FMatrix.h>
#else
class vnl_double_2;
class HomgPoint2D;
class HomgLineSeg2D;
class HomgLine2D;
class FMatrix;
#endif

class ImageMetric {
public:
  // Constructors/Destructors--------------------------------------------------
  
  ImageMetric();
  virtual ~ImageMetric();

  // Operations----------------------------------------------------------------
  
  // Computations--------------------------------------------------------------

  virtual HomgPoint2D homg_to_imagehomg(const HomgPoint2D&);
  virtual HomgPoint2D imagehomg_to_homg(const HomgPoint2D&);

  // The following virtuals may be overridden if desired.
  // By default they are implemented in terms of the previous two
  virtual vnl_double_2 homg_to_image(const HomgPoint2D&);
  virtual HomgPoint2D image_to_homg(const vnl_double_2&);
  virtual HomgPoint2D image_to_homg(double x, double y);

  virtual HomgLine2D homg_to_image_line(const HomgLine2D&);
  virtual HomgLine2D image_to_homg_line(const HomgLine2D&);

  virtual HomgLineSeg2D image_to_homg(const HomgLineSeg2D&);
  virtual HomgLineSeg2D homg_to_image(const HomgLineSeg2D&);  

  virtual FMatrix image_to_homg_deprecated(const FMatrix&);
  virtual FMatrix homg_to_image_deprecated(const FMatrix&);

  virtual double perp_dist_squared(const HomgPoint2D&, const HomgLine2D&);
  virtual HomgPoint2D perp_projection(const HomgLine2D & l, const HomgPoint2D & p);
  virtual double distance_squared(const HomgPoint2D&, const HomgPoint2D&);
  virtual double distance_squared(const HomgLineSeg2D& segment, const HomgLine2D& line); // ca_distance_squared_lineseg_to_line

  virtual bool is_within_distance(const HomgPoint2D&, const HomgPoint2D&, double distance);

  // Data Access---------------------------------------------------------------
  virtual const vnl_matrix<double>& get_C() const;
  virtual const vnl_matrix<double>& get_C_inverse() const;

  virtual bool is_linear() const;
  virtual bool can_invert_distance() const;
  virtual double image_to_homg_distance(double image_distance) const;
  virtual double homg_to_image_distance(double image_distance) const;

  virtual ostream& print(ostream& s) const;
  
  // Data Control--------------------------------------------------------------

  // Static functions to condition/decondition image relations
  static FMatrix decondition(const FMatrix& F, const ImageMetric* c1, const ImageMetric* c2);
  
  inline friend ostream& operator <<(ostream& o, const ImageMetric& m) { return m.print(o); }
  
protected:
  // Data Members--------------------------------------------------------------
  
private:
  //ImageMetric(const ImageMetric& that); - use default
  //ImageMetric& operator=(const ImageMetric& that); - use default
};

#endif   // DO NOT ADD CODE AFTER THIS LINE! END OF DEFINITION FOR CLASS ImageMetric.

