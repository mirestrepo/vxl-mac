#ifndef bcam_CAMERA_H__
#define bcam_CAMERA_H__
//#=====================================================================================
//#
//#       Filename:  camera.h
//#
//#    Description:
//#
//#        Version:  1.0
//#        Created:  03/16/2003
//#       Revision:  none
//#       Compiler:  MSVC
//#
//#         Author:  Kongbin Kang  (kk)
//#        Company:  Brown University
//#          Email:  kk@lems.brown.edu
//#
//#=====================================================================================

#include <vnl/vnl_double_3x3.h>
#include <vcl_vector.h>

// a structure used to store the lens distortion parameters
// It provide facility for set it on and off.
// the difination of each bits is given at Brown's paper.

class lens_model
{
  double kc_[7];
  bool flags_[7]; // to show which distortion is on
 public:
  lens_model() {
    for (int i=0; i<7; i++) {
      kc_[i] = 0;
      flags_[i] = false;
    }
  }

  double& operator[](int i) {return kc_[i];}
  bool is_on(int i) { return flags_[i];}
  inline void turn_on(int i) { flags_[i] = true;}
  inline void turn_off(int i) { flags_[i] = false;}
};


// an abstract camera definition.
// it store the lens model and intrisic parameter of camera
class camera
{
  int id_;
  vnl_double_3x3 k_;
  lens_model lm_;
 public:
   int getID() { return id_;}
  vnl_double_3x3 get_intrisic_matrix(){ return k_;}
  void set_lens_model(vcl_vector<bool> flags);

  void set_intrisic_matrix(vnl_double_3x3 k) {k_ = k;}

  camera(int id);
  ~camera(){}
};

#endif // bcam_CAMERA_H__
