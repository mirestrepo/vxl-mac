// BeginLatex
//
// With the same set of images as in section~\ref{sec:init:ransac}, we
// now align the images using both the landmarks and vessel center
// points. Different from the previous example, the landmarks are only
// for initialization using invariant indexing, which is the focus of
// this section. The initial transformation from the initializer is
// refined by the registration engine using the vessel center points
// as features.
//
// A brief description of the invariant signature computation,
// matching, and initial estimation is as follows. Matches between two
// landmarks, one in each image, are generated by computing and
// comparing invariants \cite{binford_quasi:arpa93, mundy:book92}.
// Invariants are blood vessel width ratios and blood vessel
// orientations (Figure~\ref{fig:landmark}), giving a five-component
// invariant signature vector. The first 2 elements are cartesian and
// the last 3 are angular invariants. The closest match is found for
// each signature vector. Additional matches are determined when the
// Mahalanobis distance between signature vectors is within a 95\%
// confidence chi-squared uncertainty bound. The set is ordered by the
// chi-squared confidence levels. For each match, a similarity
// transformation is estimated from the landmark locations and the
// orientations and widths of the vessels that meet to form the
// landmarks.
// 
// \begin{figure}[tb]
// \begin{center}
// \includegraphics[width=2.5in]{single_invariant} 
// \end{center}
// \caption{A landmark is characterized by a center location $\vect{c}$,
// the orientations of the three blood vessels that meet to form it and
// the widths $w_j$ of the blood vessel.  Differences in orientations and
// the ratios of the blood vessel widths are invariant to rotation,
// translation and scaling of the image.  The orientations themselves are
// invariant to translation and scaling.}
// \label{fig:landmark}
// \end{figure}
//
// The base class of a feature with invariant properties (also refer
// to as invariant feature) is \code{rgrl\_invariant}. The invariant
// indexing initializer interacts with such features via calls for
// cartesian and angular invariants. An invariant feature can also
// estimate the transformation from another feature of the same
// type. The derived class for a landmark invariant feature is
// \code{rgrl\_invariant\_single\_landmark}.
//
// EndLatex

// A input landmark file contains a landmark entry in the format:
//
// \begin{verbatim}
// x y       
// dx1 dy1 w1 
// dx2 dy2 w2
// dx3 dy3 w3
// \end{verbatim}
//
// The first row is the location. The 2nd to the 4th each contains a
// vessel direction with width. 
//


#include <vcl_fstream.h>
#include <vcl_iostream.h>
#include <vnl/vnl_vector_fixed.h>
#include <vnl/vnl_matrix.h>

#include <vil/vil_load.h>
#include <vil/vil_image_view.h>

#include <rrel/rrel_muset_obj.h>
#include <rrel/rrel_tukey_obj.h>

#include <rgrl/rgrl_feature_based_registration.h>
#include <rgrl/rgrl_feature_set_location_masked.h>
#include <rgrl/rgrl_matcher_k_nearest.h>
#include <rgrl/rgrl_trans_quadratic.h>
#include <rgrl/rgrl_est_quadratic.h>
#include <rgrl/rgrl_trans_similarity.h>
#include <rgrl/rgrl_est_similarity2d.h>
#include <rgrl/rgrl_trans_reduced_quad.h>
#include <rgrl/rgrl_est_reduced_quad2d.h>
#include <rgrl/rgrl_initializer_inv_indexing.h>
#include <rgrl/rgrl_invariant_single_landmark.h>

#include <rgrl/rgrl_feature_trace_pt.h>
#include <rgrl/rgrl_scale_est_all_weights.h>
#include <rgrl/rgrl_scale_est_closest.h>
#include <rgrl/rgrl_weighter_m_est.h>
#include <rgrl/rgrl_convergence_on_weighted_error.h>

#include <rgrl/rgrl_cast.h>
#include <rgrl/rgrl_mask.h>
#include <rgrl/rgrl_converge_status.h>
#include <rgrl/rgrl_data_manager.h>

#include "test_util.h"

typedef vcl_vector< rgrl_feature_sptr >  feature_vector;
typedef vnl_vector_fixed<double,2>       vector_2d;
typedef vcl_vector< rgrl_invariant_sptr> landmark_invaraint_vector;

// using command/observer pattern
class command_iteration_update: public rgrl_command 
{
public:
  void execute(rgrl_object* caller, const rgrl_event & event )
  {
    execute( (const rgrl_object*) caller, event );
  }
  
  void execute(const rgrl_object* caller, const rgrl_event & event )
  {
    const rgrl_feature_based_registration* reg_engine =
      dynamic_cast<const rgrl_feature_based_registration*>(caller);
    rgrl_transformation_sptr trans = reg_engine->current_transformation();

    if ( trans->is_type( rgrl_trans_similarity::type_id() ) ) {
      rgrl_trans_similarity* sim_xform = rgrl_cast<rgrl_trans_similarity*>(trans);
      vcl_cout<<"xform: A = \n"<<sim_xform->A()<<"t = "<<sim_xform->t()<<vcl_endl;}
    else if ( trans->is_type( rgrl_trans_reduced_quad::type_id() ) ) {
      rgrl_trans_reduced_quad* rq_xform = rgrl_cast<rgrl_trans_reduced_quad*>(trans);
      vcl_cout<<"xform: Q = \n"<<rq_xform->Q()<<"A = "<<rq_xform->A()<<
        "t = "<<rq_xform->t()<<vcl_endl;
    }
    else if ( trans->is_type( rgrl_trans_quadratic::type_id() ) ) {
      rgrl_trans_quadratic* q_xform = rgrl_cast<rgrl_trans_quadratic*>(trans);
      vcl_cout<<"xform: Q = \n"<<q_xform->Q()<<"A = "<<q_xform->A()<<
        "t = "<<q_xform->t()<<vcl_endl;
    }
    else vcl_cout<<"Unknown type"<<vcl_endl; 
  }
};

void 
read_feature_file( const char* filename,
                   feature_vector& trace_points )
{
  vcl_ifstream istr( filename );

  if( !istr ) {
    vcl_cerr<<"ERROR: Cannot open "<<filename<<vcl_endl;
    return;
  }

  vector_2d location;
  vector_2d direction;

  bool done = false;
  while ( !done && istr ) {
    if ( !(istr >> location[0] >> location[1] >> direction[0] >> direction[1]) )
      done = true;
    else trace_points.push_back( new rgrl_feature_trace_pt(location, direction) );
  }

  istr.close();
  vcl_cout<<"There are "<<trace_points.size()<<" features"<<vcl_endl;

}

void
read_landmark_file( const char* filename,
                    landmark_invaraint_vector&  landmark_inv )
{
  vcl_ifstream istr( filename );

  if( !istr ) {
    vcl_cerr<<"ERROR: Cannot open "<<filename<<vcl_endl;
    return;
  }

  double angular_std = 5.5*vnl_math::pi/180;
  double width_ratio_std = 4.5*vnl_math::pi/180;

  vector_2d location;
  vector_2d direction1;
  vector_2d direction2;
  vector_2d direction3;
  double width1, width2, width3;
  
  bool done = false;
  while ( !done && istr ) {
    if ( !(istr >> location[0] >> location[1] ) )
      done = true;
    else {
      // read in the directions and widths
      istr >> direction1[0] >> direction1[1]>>width1;
      istr >> direction2[0] >> direction2[1]>>width2;
      istr >> direction3[0] >> direction3[1]>>width3;
      // Make sure all widths are at least 1
      width1 = vnl_math_max( 1.0, width1);
      width2 = vnl_math_max( 1.0, width2);
      width3 = vnl_math_max( 1.0, width3);

      // BeginCodeSnippet
      rgrl_invariant_single_landmark* single =  
        new rgrl_invariant_single_landmark( location, direction1, 
                                            direction2, direction3,
                                            width1, width2, width3,
                                            angular_std, width_ratio_std );
      // EndCodeSnippet
      landmark_inv.push_back( single );

      // If the single constellation was ambiguous create a copy with
      // the indices shifted to releive the ambiguity
      if ( single->is_ambiguous() ) {
        rgrl_invariant_single_landmark* copy = 
          new rgrl_invariant_single_landmark( *single, angular_std, width_ratio_std );
        landmark_inv.push_back( copy );
      } 
    }
  }
}            
      
int 
main( int argc, char* argv[] )
{
  if( argc < 5 ) {
    vcl_cerr << "Missing Parameters " << vcl_endl;
    vcl_cerr << "Usage: " << argv[0];
    vcl_cerr << " FixedImageTraceFile FixedImageLandmarkFile MovingImageTraceFile MovingImageLandmarkFile MaskImage";
    return 1;
  }

  prepare_testing();

  // Prepare feature sets
  //
  feature_vector moving_set;
  feature_vector fixed_set;
  landmark_invaraint_vector moving_landmark_set;
  landmark_invaraint_vector fixed_landmark_set;

  const char* fixed_trace_file_name = argv[1];
  const char* fixed_landmark_file_name = argv[2];
  const char* moving_trace_file_name = argv[3];
  const char* moving_landmark_file_name = argv[4];

  read_feature_file( moving_trace_file_name, moving_set );
  read_feature_file( fixed_trace_file_name, fixed_set );
  read_landmark_file( moving_landmark_file_name, moving_landmark_set );
  read_landmark_file( fixed_landmark_file_name, fixed_landmark_set );


  const unsigned int dimension = 2;
  rgrl_feature_set_sptr moving_feature_set;
  rgrl_feature_set_sptr fixed_feature_set;

  const char* make_file_name = argv[5];
  vil_image_view<vxl_byte> mask_image;
  mask_image = vil_load(make_file_name);
  rgrl_mask_sptr mask = new rgrl_mask_2d_image( mask_image );

  moving_feature_set = 
    new rgrl_feature_set_location_masked<dimension>(moving_set,
                                                    mask);
  fixed_feature_set = 
    new rgrl_feature_set_location_masked<dimension>(fixed_set,
                                                    mask);
  rgrl_mask_box moving_image_region = moving_feature_set->bounding_box();
  rgrl_mask_box fixed_image_region  = fixed_feature_set->bounding_box(); 

  // Create the initializer
  //
  // BeginLatex
  //
  // Having all landmark invariant features stored in two lists, one
  // for each image, we are ready to construct the initializer, which
  // performs the matching and ordering of the
  // matches. \code{add\_data(.)} can be called multiple times if
  // there are multiple invariant lists for an image. All matches are
  // ordered by chi-squared confidence levels. Unlike
  // \code{rgrl\_initializer\_ran\_sam},
  // \code{rgrl\_initializer\_inv\_indexing} can return more than one
  // initial estimate. Each call to the initializer for the next
  // initial estimate takes the next unexplored match from the list.
  //
  // EndLatex

  // BeginCodeSnippet
  rgrl_initializer_inv_indexing* inv_initializer = 
    new rgrl_initializer_inv_indexing( moving_image_region,
                                       fixed_image_region );
  double angular_std = 5.5*vnl_math::pi/180;
  double nn_radius = angular_std * vcl_sqrt(11.0704);//95% chi-sqr uncertainty bound 
  inv_initializer->add_data( fixed_landmark_set,
                             moving_landmark_set,
                             nn_radius );
  rgrl_initializer_sptr initializer = inv_initializer;
  // EndCodeSnippet

  // Add the data and model hierarchy needed by the view generator.
  //
  rgrl_data_manager_sptr data = new rgrl_data_manager();
  
  data->add_data( moving_feature_set,  // from data
                  fixed_feature_set);  // to data

  data->add_estimator(new rgrl_est_quadratic(2) );

  // Set up the reg algorithm
  //
  //rgrl_view_based_registration reg( data, view_gen, conv_test );
  rgrl_feature_based_registration reg( data );

  reg.set_expected_max_geometric_scale(30);
  reg.set_expected_min_geometric_scale(0.5);
  reg.penalize_scaling( true );

  // For debugging
  reg.add_observer( new rgrl_event_iteration(), new command_iteration_update());

  reg.run( initializer );

  // Output Results
  //
  if ( reg.has_final_transformation() ) {
    vcl_cout<<"Final xform: "<<vcl_endl;
    rgrl_transformation_sptr trans = reg.final_transformation();
    rgrl_trans_quadratic* q_xform = rgrl_cast<rgrl_trans_quadratic*>(trans);
    vcl_cout<<"Q =\n"<<q_xform->Q()<<"A = "<<q_xform->A()<<"t ="<<q_xform->t()<<vcl_endl;
    vcl_cout<<"Final alignment error = "<<reg.final_status()->error()<<vcl_endl;
  }

  // BeginLatex
  //
  // Let's execute this example with the following feature files.
  // \verb+rgrl/examples/IMG0002.pgm.txt+ contains traces and
  // \verb+rgrl/examples/IMG0002.pgm.landmarks.invariant.txt+ contains
  // landmarks of IMG0002.pgm. Similarly,
  // \verb+rgrl/examples/IMG0004.pgm.txt+ and
  // \verb+rgr/examples/IMG0004.pgm.landmarks.invariant.txt+ for
  // IMG0004.pgm. It does not matter which image is the fixed and
  // which is the moving, since the initialization process is
  // automatic. This executable also takes a binary image, called mask
  // image. Such an image is used to define the valid region of the
  // retina image. The mask image is \verb+examples/mask.png+. When
  // mapping IMG0002 to IMG0004 (the same pair as in
  // section~\ref{sec:init:ransac}), the registration process should
  // succeed on the second initial estimate with the final result:
  //
  // \begin{verbatim}
  // Final xform: 
  // Q =
  // 0.000101439 0.000101008 8.86866e-06 
  // 1.85556e-05 2.149e-05 -4.64715e-07 
  // A = 0.859269 -0.0949512 
  // -0.0495303 0.971695 
  // t =-209.843 -7.68227
  // Final alignment error = 0.576973
  // \end{verbatim}
  //
  // The improvement in accuracy over the previous example is due to
  // the use of vesel center points for refinement of the
  // transformation.
  // 
  // EndLatex

  // Perform testing
  //
  test_macro( "Registration using invariant indexing on retinal images" , 
              reg.final_status()->error(), 3 );

  return 0;
}
