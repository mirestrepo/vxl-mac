#ifndef rgrl_feature_face_pt_h_
#define rgrl_feature_face_pt_h_
//:
// \file
// \brief Class to represent a N-d face edge element.
//    It has 1 normal direction and N-1 tangent directions.
//    There could be a defined image region surrounding these.
// \author Chuck Stewart
// \date 16 Sep 2003

#include <rgrl/rgrl_feature.h>

class rgrl_feature_face_pt
  : public rgrl_feature
{
 public:
  rgrl_feature_face_pt( vnl_vector< double > const& location,
                        vnl_vector< double > const& normal );

  virtual vnl_vector<double> const& location() const;

  virtual vnl_matrix<double> const& error_projector() const;

  virtual unsigned int num_constraints() const;

  // Defines type-related functions
  rgrl_type_macro( rgrl_feature_face_pt, rgrl_feature );

  virtual vnl_vector<double> const& normal() const;

  //: Return a matrix whose columns form the subspace tangent to the face normal
  virtual vnl_matrix<double> const&
  tangent_subspace();

  //: Result is a rgrl_feature_face_pt
  virtual rgrl_feature_sptr transform( rgrl_transformation const& xform ) const;

 protected:
  //:
  // Create an uninitialized face_pt of dimension dim
  //
  rgrl_feature_face_pt( );
  //:  The location, the normal, and the error projector.
  vnl_vector<double> location_;
  vnl_vector<double> normal_;
  vnl_matrix<double> error_proj_;

 private:

  //:  The basis for the subspace of vectors normal to the normal direction.
  //   This is tangent subspace.  It is computed once, when first needed, and cached.
  //   This is because the feature
  //  location and tangent are fixed.
  bool subspace_cached_;
  vnl_matrix< double > tangent_subspace_;
};


#endif
