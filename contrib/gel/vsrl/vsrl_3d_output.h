#ifndef vsrl_3d_output_h
#define vsrl_3d_output_h

// this program will take the disparity between two images and
// compute a 3d output for the data. If (u,v) -> (u + d, v) then
// one posible set of cameral could be P1 =(1000,0100,0010) and P2 
// would equal P2=(1010,0100,0010). In this way the point in space
// would be X=(u,v,d,1) it turns out that (u,v,1) = P1 X and
// (u,v+d,1) = P2 X. This means that X is within a projective
// transformation of the true X.

#include <vsrl/vsrl_dense_matcher.h>
#include <vsrl/vsrl_image_correlation.h>
#include <vnl/vnl_matrix.h>
#include <vil/vil_image.h>
#include <vil/vil_memory_image_of.h>
#include <vsrl/vsrl_diffusion.h>

class vsrl_3d_output
{


 public:	

  // the constructor
  
  vsrl_3d_output(const vil_image &im1, const vil_image &im2);
  
  // destructor
  ~vsrl_3d_output();

  // set the matcher of the data 
  void set_matcher(vsrl_dense_matcher *matcher);

  // set the projective transform

  void set_projective_transform(vnl_matrix<double> &H);
  
  // read in the transform

  void read_projective_transform(char *filename);

  // write the data 

  void write_output(char *filename);
  

 private:
  
  // the matcher used to compute the data
  
  vsrl_dense_matcher *_matcher;
  
  // the image buffers

  vil_byte_buffer _buffer1;
  vil_byte_buffer _buffer2;

  // the projective transform used to convert the initial values 
  // of X into the true values of X

  vnl_matrix<double> _H;

  bool non_valid_point(int x, int y); // identifies non valid points
  
  vsrl_image_correlation _image_correlation; // the image correlation object

  vil_image _image1; // the first image
  vil_image _image2; // the second image


 void  write_disparaty_image(char *filename,vsrl_diffusion *diff);


  
};

#endif

  

  
