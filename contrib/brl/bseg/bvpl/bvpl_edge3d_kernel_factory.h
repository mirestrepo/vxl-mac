// This is brl/bseg/bvpl/bvpl_edge3d_kernel_factory.h
#ifndef bvpl_edge3d_kernel_h_
#define bvpl_edge3d_kernel_h_
//:
// \file
#include "bvpl_kernel_factory.h"
#include <vcl_iostream.h>


class bvpl_edge3d_kernel_factory : public bvpl_kernel_factory
{
 public:
  //:Default Constructor
  bvpl_edge3d_kernel_factory();

  //: Constructs an edge of dimensions height*width. The canonical form of this edge is along the y-axis
  // + + 0 - -
  // + + 0 - -
  // + + 0 - -
  // + + 0 - -
  bvpl_edge3d_kernel_factory(unsigned length, unsigned width, unsigned height);

  virtual ~bvpl_edge3d_kernel_factory() {}

  //Height of edge
  unsigned height() const { return height_; }

  //Width of edge
  unsigned width() const { return width_; }

  //Length of edge
  unsigned length() const { return length_; }


  /******************Batch Methods ***********************/
  ////: Creates a vector of kernels with azimuthal and elevation resolutio equal to pi/4. And angle of rotation= angular_resolution_
  //virtual bvpl_kernel_vector_sptr create_kernel_vector();

  ////: Creates a vector of kernels according to given  azimuthal and elevation resolutio, and angle of rotation= angular_resolution_
  //virtual bvpl_kernel_vector_sptr create_kernel_vector(float pi, float phi);

  ////: Creates a vector of kernels  according to given azimuthal, levation resolutio and angle_res
  //virtual bvpl_kernel_vector_sptr create_kernel_vector(float pi, float phi, float angular_res);

 private:

  //:Creates a 2d edge kernel
  virtual void create_canonical();

  unsigned height_;
  unsigned width_;
  unsigned length_;

  static const unsigned max_size_ = 71;
};

#endif
