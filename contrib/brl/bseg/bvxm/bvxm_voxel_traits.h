// This is brl/bseg/bvxm/bvxm_voxel_traits.h
#ifndef bvxm_voxel_traits_h_
#define bvxm_voxel_traits_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \brief Templated component for voxel datatype details
//
// In most cases it is probably better to use vil_pixel_format.
//
// \author Isabel Restrepo
// \date   February 24, 2008
// \verbatim
//  Modifications
//   8/5/2008 Isabel Restrepo -Added template class bvxm_voxel_traits_mc<unsigned dim, unsigned modes>
//                             This class is incharged of proper instantiation of multi-channel appereance model processor
//                            -Added bvxm_voxel_traits<APM_MOG_MC_3_3> : public bvxm_voxel_traits_mc<3,3>,
//                             Everytime an user needs an mog appereance model with unexisting dimension of number
//                             of gaussian modes, it needs to add an entry in the enum and and new class that looks as follows
//                             class bvxm_voxel_traits<APM_MOG_MC_DIM_MODES> : public bvxm_voxel_traits_mc<DIM,MODES> {};
// \endverbatim
//-----------------------------------------------------------------------------
#include "bvxm_mog_grey_processor.h"
#include "bvxm_mog_rgb_processor.h"
#include "bvxm_mog_mc_processor.h"
#include "bvxm_lidar_processor.h"
#include "bvxm_float_processor.h"
#include "grid/bvxm_opinion.h"

enum bvxm_voxel_type
{
  OCCUPANCY = 0,
  OCCUPANCY_OPINION,
  APM_MOG_GREY,
  APM_MOG_RGB,
  APM_MOG_MC_2_3,
  APM_MOG_MC_3_3,
  APM_MOG_MC_4_3,
  EDGES,
  LIDAR,
  FLOAT,
  UNKNOWN
};

//: Pixel properties for templates.
template <bvxm_voxel_type>
class bvxm_voxel_traits;

//: Specialization of Pixel properties for bvxm_mog_mc_processor templates.
template <unsigned int dim, unsigned int modes>
class bvxm_voxel_traits_mc
{
 public:
  //:Datatype of the occupancy probabilities
  typedef bvxm_mog_mc_processor<dim,modes> appearance_processor;
  typedef typename bvxm_mog_mc_processor<dim,modes>::apm_datatype voxel_datatype;
  typedef typename bvxm_mog_mc_processor<dim,modes>::obs_datatype obs_datatype;
  typedef typename bvxm_mog_mc_processor<dim,modes>::obs_mathtype obs_mathtype;

  static inline vcl_string filename_prefix(){ return "apm_mog_mc"; }
  static inline bool is_multibin() { return false; }
  static inline voxel_datatype initial_val()
  {
    voxel_datatype init_val;
    return init_val;
  }
};


//: Voxel traits for an occupancy grid
template<>
class bvxm_voxel_traits<OCCUPANCY>
{
 public:
  //:Datatype of the occupancy probabilities
  typedef float voxel_datatype;

  static inline vcl_string filename_prefix(){ return "ocp"; }
  static inline bool is_multibin() { return false; }
  static inline voxel_datatype initial_val() { return 0.005f; }
};

//: Voxel traits for an occupancy grid
template<>
class bvxm_voxel_traits<OCCUPANCY_OPINION>
{
 public:
  //:Datatype of the occupancy probabilities
  typedef bvxm_opinion voxel_datatype;

  static inline vcl_string filename_prefix(){ return "ocp_opinion"; }
  static inline bool is_multibin() { return false; }
  static inline voxel_datatype initial_val() { return bvxm_opinion(0.005f);}
};
//: Voxel traits for a mixture of gaussian appereance model of grey-scale images
template<>
class bvxm_voxel_traits<APM_MOG_GREY>
{
 public:
  //:Datatype of the occupancy probabilities
  typedef bvxm_mog_grey_processor appearance_processor;
  typedef bvxm_mog_grey_processor::apm_datatype voxel_datatype;
  typedef bvxm_mog_grey_processor::obs_datatype obs_datatype;
  typedef bvxm_mog_grey_processor::obs_mathtype obs_mathtype;

  static inline vcl_string filename_prefix() { return "apm_mog_grey"; }
  static inline bool is_multibin() { return true; }
  static inline voxel_datatype initial_val()
  {
    voxel_datatype init_val;
    return init_val;
  }
};

//: Voxel traits for a mixture of gaussian appereance model of rgb images
template<>
class bvxm_voxel_traits<APM_MOG_RGB>
{
 public:
  //:Datatype of the occupancy probabilities
  typedef bvxm_mog_rgb_processor appearance_processor;
  typedef bvxm_mog_rgb_processor::apm_datatype voxel_datatype;
  typedef bvxm_mog_rgb_processor::obs_datatype obs_datatype;
  typedef bvxm_mog_rgb_processor::obs_mathtype obs_mathtype;

  static inline vcl_string filename_prefix(){ return "apm_mog_rgb"; }
  static inline bool is_multibin() { return true; }
  static inline voxel_datatype initial_val()
  {
    voxel_datatype init_val;
    return init_val;
  }
};

//: Initialize voxel traits for a mixture of gaussian appereance model of 2-d images, with 3 gaussian modes
template<>
class bvxm_voxel_traits<APM_MOG_MC_2_3> : public bvxm_voxel_traits_mc<2,3>{};

//: Initialize voxel traits for a mixture of gaussian appereance model of 3-d images, with 3 gaussian modes
template<>
class bvxm_voxel_traits<APM_MOG_MC_3_3> : public bvxm_voxel_traits_mc<3,3>{};

//: Initialize voxel traits for a mixture of gaussian appereance model of 4-d images, with 3 gaussian modes
template<>
class bvxm_voxel_traits<APM_MOG_MC_4_3> : public bvxm_voxel_traits_mc<4,3>{};

//: Voxel traits for an EDGES grid
template<>
class bvxm_voxel_traits<EDGES>
{
 public:
  //:Datatype of the occupancy probabilities
  typedef float voxel_datatype;

  static inline vcl_string filename_prefix() { return "edges"; }
  static inline bool is_multibin() { return false; }
  static inline voxel_datatype initial_val() { return 0.01f; }
};

//: Voxel traits for an LIDAR grid
template<>
class bvxm_voxel_traits<LIDAR>
{
 public:

  typedef bvxm_lidar_processor lidar_processor;

  //:Datatype of the occupancy probabilities
  typedef float voxel_datatype;

  static inline vcl_string filename_prefix() { return "lidar"; }
  static inline bool is_multibin() { return false; }
  static inline voxel_datatype initial_val() { return 0.0f; }
};

//: Voxel traits for an occupancy grid
template<>
class bvxm_voxel_traits<FLOAT>
{

 public:
  //:Datatype of the occupancy probabilities
  typedef bvxm_float_processor appearance_processor;
  typedef bvxm_float_processor::apm_datatype voxel_datatype;
  typedef bvxm_float_processor::obs_datatype obs_datatype;
  typedef bvxm_float_processor::obs_mathtype obs_mathtype;

  static inline vcl_string filename_prefix(){ return "apm_float"; }
  static inline bool is_multibin() { return true; }
  static inline voxel_datatype initial_val()
  {
    voxel_datatype init_val = 0.0f;
    return init_val;
  }



};

#endif // bvxm_voxel_traits_h_
