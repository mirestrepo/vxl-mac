// This is brl/vvid/vvid_edge_process.cxx
#include <vcl_iostream.h>
#include <vul/vul_timer.h>
#include <vcl_vector.h>
#include <vil1/vil1_memory_image_of.h>
#include <vtol/vtol_edge_2d_sptr.h>
#include <vtol/vtol_edge_2d.h>
#include <brip/brip_float_ops.h>
#include <sdet/sdet_detector.h>
#include <vvid/vvid_edge_process.h>

vvid_edge_process::vvid_edge_process(sdet_detector_params & dp)
  : sdet_detector_params(dp)
{
}

vvid_edge_process::~vvid_edge_process()
{
}

bool vvid_edge_process::execute()
{
  vul_timer t;
  if (this->get_N_input_images()!=1)
    {
      vcl_cout << "In vvid_edge_process::execute() - not exactly one"
               << " input image \n";
      return false;
    }
  output_topo_objs_.clear();

  vil1_image img = vvid_video_process::get_input_image(0);
  vil1_memory_image_of<unsigned char> cimg;
  if (img.components()==3)
    {
      vil1_memory_image_of<float> fimg = brip_float_ops::convert_to_float(img);
      vvid_video_process::clear_input();//remove image from input
      //convert a color image to grey
      cimg = brip_float_ops::convert_to_byte(fimg);
    }
  else
    {
      cimg = vil1_memory_image_of<unsigned char>(img);
      vvid_video_process::clear_input();
    }
  //initialize the detector
  sdet_detector detector(*((sdet_detector_params*)this));
  detector.SetImage(cimg);
  //process edges
  detector.DoContour();
  vcl_vector<vtol_edge_2d_sptr> * edges = detector.GetEdges();

  if (!edges)
    return false;

   for (vcl_vector<vtol_edge_2d_sptr>::iterator eit = edges->begin();
        eit != edges->end(); eit++)
     output_topo_objs_.push_back((*eit)->cast_to_topology_object());

  output_image_ = 0;//no output image is produced
  vcl_cout << "process " << edges->size()
           << " edges in " << t.real() << " msecs.\n";
  return true;
}
