#ifndef vsrl_results_dense_matcher_h
#define vsrl_results_dense_matcher_h

//:
//  \file

#include <vil/vil_memory_image_of.h>
#include <vil/vil_image.h>
#include <vsrl/vsrl_dense_matcher.h>
#include <vsrl/vsrl_image_correlation.h>

//:
// This class will take a disparity image which is the output
// of the dynamic programming and give results as is required

class vsrl_results_dense_matcher : public vsrl_dense_matcher
{
 protected:

  vil_memory_image_of<int> disparity_; //!< memory images of the disparities

 public:

  // constructor
  vsrl_results_dense_matcher(const vil_image &image1, const vil_image &disparity);

  // destructor
  virtual ~vsrl_results_dense_matcher();

  // set the correlation range
  virtual int get_assignment(int x, int y);

  // get the disparity of pixel x,y
  virtual int get_disparity(int x, int y);
};

#endif
