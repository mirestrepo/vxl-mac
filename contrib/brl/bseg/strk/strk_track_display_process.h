// This is brl/bseg/strk/strk_track_display_process.h
#ifndef strk_track_display_process_h_
#define strk_track_display_process_h_
//----------------------------------------------------------------------------
//:
// \file
// \brief Displays a tracked polygon on a video
// \author
//   J.L. Mundy
//
// \verbatim
//  Modifications:
//   J.L. Mundy October 14, 2003    Initial version.
// \endverbatim
//---------------------------------------------------------------------------
// not used? #include <vcl_fstream.h>
#include <vcl_string.h>
#include <vgl/vgl_point_2d.h>
#include <vtol/vtol_face_2d_sptr.h>
#include <vpro/vpro_video_process.h>

class strk_track_display_process : public vpro_video_process
{
 public:
  strk_track_display_process();
  ~strk_track_display_process();
  virtual process_data_type get_output_type() const { return TOPOLOGY; }

  //: track to next frame
  virtual bool execute();
  virtual bool finish() { return true; }
  bool set_input_file(vcl_string const& file_name);
 private:
  //members
  bool failure_;
  bool first_frame_;
  vcl_string track_file_;
  vcl_vector<vgl_point_2d<double> > tracked_cogs_;
  vcl_vector<vtol_face_2d_sptr> tracked_faces_;
};


#endif // strk_track_display_process_h_
