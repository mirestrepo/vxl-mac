//this-sets-emacs-to-*-c++-*-mode
#ifndef cmu_1394_camera_params_h_
#define cmu_1394_camera_params_h_
//--------------------------------------------------------------------------------
//:
// \file
// \brief cmu_1394_camera_params
//   A parameter block for CMU's 1394 camera class. This block supports
//   both menu operations and file-based configuration. The acquisition
//   mode is constrained to match the rgb/monochrome flag.
// \author
//   J.L. Mundy
//
// \verbatim
//  Modifications:
//   J.L. Mundy Aug 29, 2002    Initial version.
// \endverbatim
//--------------------------------------------------------------------------------
#include <vcl_iostream.h>

class cmu_1394_camera_params{
public:
  cmu_1394_camera_params(int video_format=0, int video_mode=4,
                         int frame_rate=3, int brightness=350,
                         int sharpness=128, int exposure=350, int gain = 350,
                         bool capture=true, bool rgb=true);
  cmu_1394_camera_params(const cmu_1394_camera_params& cp);
  ~cmu_1394_camera_params();
  void set_params(const cmu_1394_camera_params& cp);
  void constrain();//make sure the parameters are consistent

  friend 
  vcl_ostream& operator << (vcl_ostream& os, const cmu_1394_camera_params& cpp);

  //The parameters
  int _video_format;
  int _video_mode;
  int _frame_rate;
  int _brightness;
  int _sharpness;
  int _exposure;
  int _gain;
  bool _capture;//vs acquisition
  bool _rgb;//color image vs monochrome
};
 

#endif // cmu_1394_camera_params_h_
