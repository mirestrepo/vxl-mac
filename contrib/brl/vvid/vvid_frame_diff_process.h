//this-sets-emacs-to-*-c++-*-mode
#ifndef vvid_frame_diff_process_h_
#define vvid_frame_diff_process_h_
//--------------------------------------------------------------------------------
//:
// \file
// \brief Computes frame difference (first output is all zeros)
// \author
//   J.L. Mundy
//
// \verbatim
//  Modifications:
//   J.L. Mundy February 16, 2003    Initial version.
// \endverbatim
//-----------------------------------------------------------------------------
#include <vil1/vil1_memory_image_of.h>
#include <vvid/vvid_video_process.h>

class vvid_frame_diff_process : public vvid_video_process
{
 public:
  vvid_frame_diff_process();
  ~vvid_frame_diff_process();
  virtual process_data_type get_output_type() const { return IMAGE; }
  //: difference sequential frames
  virtual bool execute();
  virtual bool finish();
 private:
  //members
  bool first_frame_;
  vil1_memory_image_of<float> img0_;
};

#endif // vvid_frame_diff_process_h_
