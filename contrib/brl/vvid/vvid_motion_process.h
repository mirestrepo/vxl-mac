//this-sets-emacs-to-*-c++-*-mode
#ifndef vvid_motion_process_h_
#define vvid_motion_process_h_
//--------------------------------------------------------------------------------
//:
// \file
// \brief detects linear motion
// \author
//   J.L. Mundy
//
// \verbatim
//  Modifications:
//   J.L. Mundy February 16, 2003    Initial version.
// \endverbatim
//-----------------------------------------------------------------------------
#include <vcl_vector.h>
#include <vvid/vvid_video_process.h>

class vvid_motion_process : public vvid_video_process
{
 public:
  enum state_symbol {NO_IMAGE=0, FIRST_IMAGE, IN_PROCESS};
  vvid_motion_process();
  ~vvid_motion_process();
  virtual process_data_type get_output_type() const { return IMAGE; }
  //: compute motion
  virtual bool execute();
  virtual bool finish();
 private:
  //local methods
  void compute_motion(vil_image ix, vil_image iy);
  void update_queue(vil_image ix, vil_image iy);
  //members
  state_symbol state_;
  bool first_frame_;
  vcl_vector<vil_image > queuex_;
  vcl_vector<vil_image > queuey_;
};

#endif // vvid_motion_process_h_
