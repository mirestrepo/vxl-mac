#ifndef boxm2_data_traits_h_
#define boxm2_data_traits_h_
//:
// \file
// \brief traits for data types
//
// \author Vishal Jain
// \date nov 17, 2010

#include <vcl_string.h>
#include <vcl_cstddef.h> // for std::size_t
#include <vnl/vnl_vector_fixed.h>
#include <vcl_iostream.h>

#include "boxm2_normal_albedo_array.h"

class boxm2_mog3_grey_processor;
class boxm2_gauss_grey_processor;
class boxm2_gauss_rgb_processor;

enum boxm2_data_type
{
  BOXM2_ALPHA=0,
  BOXM2_GAMMA,
  BOXM2_MOG3_GREY,
  BOXM2_MOG3_GREY_16,
  BOXM2_BATCH_HISTOGRAM,
  BOXM2_GAUSS_RGB,
  BOXM2_MOG2_RGB,
  BOXM2_NUM_OBS,
  BOXM2_NUM_OBS_SINGLE,
  BOXM2_NUM_OBS_SINGLE_INT,
  BOXM2_AUX,
  BOXM2_INTENSITY,
  BOXM2_AUX0,
  BOXM2_AUX1,
  BOXM2_AUX2,
  BOXM2_AUX3,
  BOXM2_AUX4,
  BOXM2_FLOAT,
  BOXM2_FLOAT8,
  BOXM2_FLOAT16,
  BOXM2_VIS_SPHERE,
  BOXM2_NORMAL,
  BOXM2_POINT,
  BOXM2_VIS_SCORE,
  BOXM2_GAUSS_GREY,
  BOXM2_NORMAL_ALBEDO_ARRAY,
  BOXM2_UNKNOWN
};

//: Pixel properties for templates.
template <boxm2_data_type type>
class boxm2_data_traits;

//: traits for a mixture of gaussian appearance model of gray-scale images
template<>
class boxm2_data_traits<BOXM2_ALPHA>
{
 public:
  typedef float datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "alpha"; else return "alpha_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_GAMMA>
{
 public:
  typedef float datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "gamma"; else return "gamma_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_MOG3_GREY>
{
 public:
  typedef boxm2_mog3_grey_processor processor;
  typedef vnl_vector_fixed<unsigned char, 8> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_mog3_grey"; else return "boxm2_mog3_grey_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_MOG3_GREY_16>
{
 public:
  typedef boxm2_mog3_grey_processor processor;
  typedef vnl_vector_fixed<unsigned short, 8> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_mog3_grey_16"; else return "boxm2_mog3_grey_16_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_GAUSS_RGB>
{
 public:
  typedef boxm2_gauss_rgb_processor processor;
  typedef vnl_vector_fixed<unsigned char, 8> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_gauss_rgb"; else return "boxm2_gauss_rgb_"+identifier; }
};

//: simple gaussian with a sigma and std dev
template<>
class boxm2_data_traits<BOXM2_GAUSS_GREY>
{
 public:
  typedef boxm2_gauss_grey_processor processor;
  typedef vnl_vector_fixed<unsigned char, 2> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_gauss_grey"; else return "boxm2_gauss_grey_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_MOG2_RGB>
{
 public:
  typedef vnl_vector_fixed<unsigned char, 16> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_mog2_rgb"; else return "boxm2_mog2_rgb_"+identifier; }
};


template<>
class boxm2_data_traits<BOXM2_NORMAL>
{
 public:
  typedef vnl_vector_fixed<float, 4> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_normal"; else return "boxm2_normal_"+identifier; }
};


template<>
class boxm2_data_traits<BOXM2_POINT>
{
 public:
  typedef vnl_vector_fixed<float, 4> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_point"; else return "boxm2_point_"+identifier; }
};


template<>
class boxm2_data_traits<BOXM2_NUM_OBS>
{
 public:
  typedef vnl_vector_fixed<unsigned short, 4> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_num_obs"; else return "boxm2_num_obs_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_NUM_OBS_SINGLE>
{
 public:
  typedef unsigned short datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_num_obs_single"; else return "boxm2_num_obs_single_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_NUM_OBS_SINGLE_INT>
{
 public:
  typedef unsigned datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_num_obs_single_int"; else return "boxm2_num_obs_single_int_"+identifier; }
};


//: Aux data contains four values to assist update calculations:
// * seg_len_array,   (total cell segment length)
// * mean_obs_array,  (total cell mean obs)
// * vis_array,       (total cell visibility)
// * beta_array,      (total cell bayes update factor)
template<>
class boxm2_data_traits<BOXM2_AUX>
{
 public:
  typedef vnl_vector_fixed<float, 4> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "aux"; else return "aux_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_FLOAT8>
{
 public:
  typedef vnl_vector_fixed<float, 8> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "float8"; else return "float8_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_FLOAT>
{
 public:
  typedef float datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "float"; else return "float_"+identifier; }
};


template<>
class boxm2_data_traits<BOXM2_FLOAT16>
{
 public:
  typedef vnl_vector_fixed<float, 16> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "float16"; else return "float16_"+identifier; }
};


template<>
class boxm2_data_traits<BOXM2_VIS_SPHERE>
{
 public:
  typedef vnl_vector_fixed<float, 16> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_vis_sphere"; else return "boxm2_vis_sphere_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_VIS_SCORE>
{
 public:
  typedef float datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_vis_score"; else return "boxm2_vis_score_"+identifier; }
};


template<>
class boxm2_data_traits<BOXM2_AUX0>
{
 public:
  typedef float datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "aux0"; else return "aux0_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_AUX1>
{
 public:
  typedef float datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "aux1"; else return "aux1_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_AUX2>
{
 public:
  typedef float datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "aux2"; else return "aux2_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_AUX3>
{
 public:
  typedef float datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "aux3"; else return "aux3_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_AUX4>
{
 public:
  typedef float datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "aux4"; else return "aux4_"+identifier; }
};


template<>
class boxm2_data_traits<BOXM2_BATCH_HISTOGRAM>
{
 public:
  typedef vnl_vector_fixed<float, 8> datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix() { return "boxm2_batch_histogram"; }
};

template<>
class boxm2_data_traits<BOXM2_INTENSITY>
{
 public:
  typedef float datatype;
  static vcl_size_t datasize() { return sizeof(datatype); }
  static vcl_string prefix(const vcl_string& identifier = "")
  { if (!identifier.size()) return "boxm2_intensity"; else return "boxm2_intensity_"+identifier; }
};

template<>
class boxm2_data_traits<BOXM2_NORMAL_ALBEDO_ARRAY>
{
 public:
  typedef boxm2_normal_albedo_array datatype;
  static vcl_size_t datasize() { return sizeof(boxm2_normal_albedo_array); }
  static vcl_string prefix() { return "boxm2_normal_albedo_array"; }
};

//: HACKY WAY TO GENERICALLY GET DATASIZES -
class boxm2_data_info
{
 public:
  static vcl_size_t datasize(vcl_string prefix)
  {
    // some of them changed to using find method to account for identifiers

    if (prefix.find(boxm2_data_traits<BOXM2_ALPHA>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_ALPHA>::datasize();
    if (prefix.find(boxm2_data_traits<BOXM2_GAMMA>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_GAMMA>::datasize();

     if (prefix.find(boxm2_data_traits<BOXM2_MOG3_GREY_16>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_MOG3_GREY_16>::datasize();

    if (prefix.find(boxm2_data_traits<BOXM2_MOG3_GREY>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_MOG3_GREY>::datasize();

    if (prefix == boxm2_data_traits<BOXM2_BATCH_HISTOGRAM>::prefix())
      return boxm2_data_traits<BOXM2_BATCH_HISTOGRAM>::datasize();

    if (prefix.find(boxm2_data_traits<BOXM2_MOG2_RGB>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_MOG2_RGB>::datasize();

    if (prefix.find(boxm2_data_traits<BOXM2_NUM_OBS_SINGLE_INT>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_NUM_OBS_SINGLE_INT>::datasize();

    if (prefix.find(boxm2_data_traits<BOXM2_NUM_OBS_SINGLE>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_NUM_OBS_SINGLE>::datasize();

    if (prefix.find(boxm2_data_traits<BOXM2_NUM_OBS>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_NUM_OBS>::datasize();

    if (prefix.find(boxm2_data_traits<BOXM2_AUX0>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_AUX0>::datasize();
    if (prefix.find(boxm2_data_traits<BOXM2_AUX1>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_AUX1>::datasize();
    if (prefix.find(boxm2_data_traits<BOXM2_AUX2>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_AUX2>::datasize();
    if (prefix.find(boxm2_data_traits<BOXM2_AUX3>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_AUX3>::datasize();
    if (prefix.find(boxm2_data_traits<BOXM2_AUX4>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_AUX4>::datasize();
    if (prefix.find(boxm2_data_traits<BOXM2_AUX>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_AUX>::datasize();
    if (prefix.find(boxm2_data_traits<BOXM2_FLOAT8>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_FLOAT8>::datasize();
    if (prefix.find(boxm2_data_traits<BOXM2_FLOAT>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_FLOAT>::datasize();
    if (prefix.find(boxm2_data_traits<BOXM2_FLOAT16>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_FLOAT16>::datasize();
    if (prefix == boxm2_data_traits<BOXM2_VIS_SPHERE>::prefix())
      return boxm2_data_traits<BOXM2_VIS_SPHERE>::datasize();
    if (prefix == boxm2_data_traits<BOXM2_VIS_SCORE>::prefix())
      return boxm2_data_traits<BOXM2_VIS_SCORE>::datasize();

    if (prefix.find(boxm2_data_traits<BOXM2_INTENSITY>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_INTENSITY>::datasize();

    if (prefix.find(boxm2_data_traits<BOXM2_GAUSS_RGB>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_GAUSS_RGB>::datasize();

    if (prefix.find(boxm2_data_traits<BOXM2_GAUSS_GREY>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_GAUSS_GREY>::datasize();

    if (prefix.find(boxm2_data_traits<BOXM2_NORMAL_ALBEDO_ARRAY>::prefix()) != vcl_string::npos) 
      return boxm2_data_traits<BOXM2_NORMAL_ALBEDO_ARRAY>::datasize();

    if (prefix.find(boxm2_data_traits<BOXM2_NORMAL>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_NORMAL>::datasize();
    
    if (prefix.find(boxm2_data_traits<BOXM2_POINT>::prefix()) != vcl_string::npos)
      return boxm2_data_traits<BOXM2_POINT>::datasize();

    return 0;
  }

  static boxm2_data_type data_type(vcl_string prefix)
  {
    // some of them changed to using find method to account for identifiers
    if (prefix.find(boxm2_data_traits<BOXM2_ALPHA>::prefix()) != vcl_string::npos)
      return BOXM2_ALPHA;
    if (prefix.find(boxm2_data_traits<BOXM2_GAMMA>::prefix()) != vcl_string::npos)
      return BOXM2_GAMMA;
     if (prefix.find(boxm2_data_traits<BOXM2_MOG3_GREY_16>::prefix()) != vcl_string::npos)
      return  BOXM2_MOG3_GREY_16 ;

    if (prefix.find(boxm2_data_traits<BOXM2_MOG3_GREY>::prefix()) != vcl_string::npos)
      return  BOXM2_MOG3_GREY ;

    if (prefix == boxm2_data_traits<BOXM2_BATCH_HISTOGRAM>::prefix())
      return  BOXM2_BATCH_HISTOGRAM ;

    if (prefix.find(boxm2_data_traits<BOXM2_MOG2_RGB>::prefix()) != vcl_string::npos)
      return  BOXM2_MOG2_RGB ;

    if (prefix.find(boxm2_data_traits<BOXM2_NUM_OBS_SINGLE_INT>::prefix()) != vcl_string::npos)
      return  BOXM2_NUM_OBS_SINGLE_INT ;

    if (prefix.find(boxm2_data_traits<BOXM2_NUM_OBS_SINGLE>::prefix()) != vcl_string::npos)
      return  BOXM2_NUM_OBS_SINGLE ;

    if (prefix.find(boxm2_data_traits<BOXM2_NUM_OBS>::prefix()) != vcl_string::npos)
      return  BOXM2_NUM_OBS ;

    if (prefix.find(boxm2_data_traits<BOXM2_AUX0>::prefix()) != vcl_string::npos)
      return  BOXM2_AUX0 ;
    if (prefix.find(boxm2_data_traits<BOXM2_AUX1>::prefix()) != vcl_string::npos)
      return  BOXM2_AUX1 ;
    if (prefix.find(boxm2_data_traits<BOXM2_AUX2>::prefix()) != vcl_string::npos)
      return  BOXM2_AUX2 ;
    if (prefix.find(boxm2_data_traits<BOXM2_AUX3>::prefix()) != vcl_string::npos)
      return  BOXM2_AUX3 ;
    if (prefix.find(boxm2_data_traits<BOXM2_AUX4>::prefix()) != vcl_string::npos)
      return  BOXM2_AUX4 ;
    if (prefix.find(boxm2_data_traits<BOXM2_AUX>::prefix()) != vcl_string::npos)
      return  BOXM2_AUX ;
    if (prefix == boxm2_data_traits<BOXM2_FLOAT8>::prefix())
      return  BOXM2_FLOAT8 ;

    if (prefix == boxm2_data_traits<BOXM2_FLOAT16>::prefix())
      return  BOXM2_FLOAT16 ;
    if (prefix == boxm2_data_traits<BOXM2_FLOAT>::prefix())
      return  BOXM2_FLOAT;
    if (prefix.find(boxm2_data_traits<BOXM2_INTENSITY>::prefix()) != vcl_string::npos)
      return  BOXM2_INTENSITY ;
    if (prefix.find(boxm2_data_traits<BOXM2_GAUSS_RGB>::prefix()) != vcl_string::npos)
      return  BOXM2_GAUSS_RGB ;
    if (prefix.find(boxm2_data_traits<BOXM2_GAUSS_GREY>::prefix()) != vcl_string::npos)
      return  BOXM2_GAUSS_GREY ;
    if (prefix.find(boxm2_data_traits<BOXM2_NORMAL_ALBEDO_ARRAY>::prefix()) != vcl_string::npos) 
      return  BOXM2_NORMAL_ALBEDO_ARRAY ;
    return BOXM2_UNKNOWN;
  }


  static void print_data(vcl_string prefix, char *cell)
  {
    if (prefix.find(boxm2_data_traits<BOXM2_ALPHA>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_ALPHA>::datatype*>(cell)[0];
      return;
    }
#if 0
    if (prefix.find(boxm2_data_traits<BOXM2_MOG3_GREY_16>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_MOG3_GREY_16>::datatype*>(cell)[0];
      return;
    }

    if (prefix.find(boxm2_data_traits<BOXM2_MOG3_GREY>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_MOG3_GREY>::datatype*>(cell)[0];
      return;
    }

    if (prefix == boxm2_data_traits<BOXM2_BATCH_HISTOGRAM>::prefix()) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_BATCH_HISTOGRAM>::datatype*>(cell)[0];
      return;
    }

    if (prefix.find(boxm2_data_traits<BOXM2_MOG2_RGB>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_MOG2_RGB>::datatype*>(cell)[0];
      return;
    }

    if (prefix.find(boxm2_data_traits<BOXM2_NUM_OBS_SINGLE>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_NUM_OBS_SINGLE>::datatype*>(cell)[0];
      return;
    }

    if (prefix.find(boxm2_data_traits<BOXM2_NUM_OBS>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_NUM_OBS>::datatype*>(cell)[0];
      return;
    }
#endif // 0
    if (prefix.find(boxm2_data_traits<BOXM2_AUX0>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_AUX0>::datatype*>(cell)[0];
      return;
    }
#if 0
    if (prefix.find(boxm2_data_traits<BOXM2_AUX1>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_AUX1>::datatype*>(cell)[0];
      return;
    }
    if (prefix.find(boxm2_data_traits<BOXM2_AUX2>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_AUX2>::datatype*>(cell)[0];
      return;
    }
    if (prefix.find(boxm2_data_traits<BOXM2_AUX3>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_AUX3>::datatype*>(cell)[0];
      return;
    }
#endif // 0
    if (prefix.find(boxm2_data_traits<BOXM2_AUX>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_AUX>::datatype*>(cell)[0];
      return;
    }

    if (prefix.find(boxm2_data_traits<BOXM2_INTENSITY>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_INTENSITY>::datatype*>(cell)[0];
      return;
    }
#if 0
    if (prefix.find(boxm2_data_traits<BOXM2_GAUSS_RGB>::prefix()) != vcl_string::npos) {
      vcl_cout <<  reinterpret_cast<boxm2_data_traits<BOXM2_GAUSS_RGB>::datatype*>(cell)[0];
      return;
    }


#endif // 0

    vcl_cerr << "In boxm2_data_info::print_data() -- type: " << prefix << " could not be identified!\n";
    return;
  }
};


#endif
