#ifndef vsrl_epipolar_dp_setup_h
#define vsrl_epipolar_dp_setup_h

// this class will take a vsrl_image_correlation class 
// and a given y value to define which raster is to be used.
// it will then create the dynamic program which will determine
// the optimal assignments between the two images


#include <vcl_vector.h>
#include <vsrl/vsrl_intensity_token.h>
#include <vsrl/vsrl_image_correlation.h>

class vsrl_raster_dp_setup
{
 public:
  // constructor
  vsrl_raster_dp_setup(int raster_line, vsrl_image_correlation *image_correlation);
  // destructor 
  ~vsrl_raster_dp_setup();
  
  // accessors 
  int get_raster_line(); // which raster was processed
  
  int get_assignment(int x); // get the assignment of pixel x

  int get_image1_width(); // the width of image1

  int get_image2_width(); // the width of image2 

 
 // set the search range for the dynamic program
  
  void set_search_range(int range);
  
  // set the prior raster
  void set_prior_raster(vsrl_raster_dp_setup *prior_raster);
  
  
  // set the cost of deviating from the bias

  void set_bias_cost(double bias_cost);

  // perform the dynamic program

  double execute();

 

 private:
  
  // the image correlation mechanism 
  
  vsrl_image_correlation *_image_correlation;
  
  // the raster line to be processed
  
  int _raster_line;
  
  // the token lists 

  vcl_vector<vsrl_intensity_token*> tok_list1;
  vcl_vector<vsrl_intensity_token*> tok_list2;

  void create_token_list(int width,
			 vsrl_image_correlation *image_correlation,
			 vcl_vector<vsrl_intensity_token*> &tok_list,
			 double step=1);
  
  // clear token list;
  void clear_token_list(vcl_vector<vsrl_intensity_token*> &tok_list);
  
  // the search range used
  int _search_range;
  
  // the bias cost
  double _bias_cost;

  // the prior raster
  vsrl_raster_dp_setup *_prior_raster;
  
  // a method that uses the prior_raster to bias the next raster
  void set_token_biases();
  
  

};


#endif

