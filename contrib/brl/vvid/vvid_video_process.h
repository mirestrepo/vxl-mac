// This is brl/vvid/vvid_video_process.h
#ifndef vvid_video_process_h_
#define vvid_video_process_h_
//--------------------------------------------------------------------------------
//:
// \file
// \brief live vvid_video_process
//
//  A generic video processor that is called from the live_video_manager
//  to carry out algorithms on the live video frames.
// \author
//   J.L. Mundy
//
// \verbatim
//  Modifications:
//   J.L. Mundy October 9, 2002    Initial version.
// \endverbatim
//--------------------------------------------------------------------------------
#include <vcl_vector.h>
#include <vil1/vil1_image.h>
#include <vbl/vbl_ref_count.h>
#include <vsol/vsol_spatial_object_2d_sptr.h>
#include <vtol/vtol_topology_object_sptr.h>

class vvid_video_process : public vbl_ref_count
{
 public:
  enum process_data_type {NOTYPE=0, IMAGE, SPATIAL_OBJECT, TOPOLOGY};

  vvid_video_process();
  ~vvid_video_process();
  void clear_input();
  void clear_output();

  void add_input_image(vil1_image const& im){input_images_.push_back(im);}

  void add_input_spatial_objects(vcl_vector<vsol_spatial_object_2d_sptr> const& spat_objs);

  void add_input_topology(vcl_vector<vtol_topology_object_sptr> const& topo_objes);

  int get_N_input_images() const { return input_images_.size(); }
  vil1_image get_input_image(unsigned int i) const;
  vil1_image get_output_image() const { return output_image_; }


  int get_N_input_spat_objs() const { return input_spat_objs_.size(); }
  vcl_vector<vsol_spatial_object_2d_sptr> const&
  get_input_spatial_objects() const { return input_spat_objs_; }

  int get_N_input_topo_objs() const { return input_topo_objs_.size(); }
  vcl_vector<vtol_topology_object_sptr> const &
  get_input_topology() const { return input_topo_objs_; }

  //:output handling may depend on the specific process
  virtual vcl_vector<vsol_spatial_object_2d_sptr> const& get_output_spatial_objects();
  virtual vcl_vector<vtol_topology_object_sptr> const & get_output_topology();

  virtual process_data_type get_input_type() const { return NOTYPE; }
  virtual process_data_type get_output_type() const { return NOTYPE; }
  virtual bool execute()=0;
  virtual bool finish()=0;
 protected:
  //members
  vcl_vector<vil1_image> input_images_;
  vcl_vector<vsol_spatial_object_2d_sptr> input_spat_objs_;
  vcl_vector<vtol_topology_object_sptr> input_topo_objs_;
  vil1_image output_image_;
  vcl_vector<vtol_topology_object_sptr> output_topo_objs_;
  vcl_vector<vsol_spatial_object_2d_sptr> output_spat_objs_;
};

#endif // vvid_video_process_h_
