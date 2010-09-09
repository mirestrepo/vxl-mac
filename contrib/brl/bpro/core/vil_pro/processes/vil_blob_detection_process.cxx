// This is brl/bpro/core/vil_pro/processes/vil_blob_detection_process.cxx
#include <bprb/bprb_func_process.h>
//:
// \file

#include <vil/vil_math.h>
#include <vil/vil_image_view.h>
#include <vil/algo/vil_structuring_element.h>
#include <vil/algo/vil_blob_finder.h>
#include <vil/algo/vil_binary_closing.h>
#include <vil/vil_convert.h>
#include <vcl_iostream.h>


//: Constructor
bool vil_blob_detection_process_cons(bprb_func_process& pro)
{
  //this process takes one input: the filename
  bool ok=false;
  vcl_vector<vcl_string> input_types;
  input_types.push_back("vil_image_view_base_sptr"); 
  input_types.push_back("unsigned"); // max size of the blob in pixels
  input_types.push_back("unsigned"); // min size of the blob in pixels

  ok = pro.set_input_types(input_types);
  if (!ok) return ok;

  vcl_vector<vcl_string> output_types;
  output_types.push_back("vil_image_view_base_sptr");  // label image
  ok = pro.set_output_types(output_types);
  if (!ok) return ok;
  
  return true;
}

//: Execute the process
bool vil_blob_detection_process(bprb_func_process& pro)
{
  // Sanity check
  if (pro.n_inputs()< 3) {
    vcl_cout << "vil_blob_detection_process: The number of inputs should be 3" << vcl_endl;
    return false;
  }

  // get the inputs
  unsigned i=0;
  vil_image_view_base_sptr img_ptr_a = pro.get_input<vil_image_view_base_sptr>(i++);
  float min_size  = pro.get_input<unsigned>(i++);
  float max_size  = pro.get_input<unsigned>(i++);

  if(vil_image_view<bool> *view=dynamic_cast<vil_image_view<bool>* > (img_ptr_a.ptr()))
  {
    
      //: Closing the holes or gaps
      vil_structuring_element selem;
      selem.set_to_disk(1.0);
      vil_image_view<bool> view_closed(view->ni(),view->nj());
      vil_binary_closing(*view,view_closed,selem);
      
      //: Find the blobs
      vil_blob_finder blob_finder(*view);//view_closed);
      vil_image_view<bool> view_blobs(view->ni(),view->nj());
      view_blobs.fill(false);

      //: Threshold the blobs.
      vcl_vector<vil_chord> bregions;
      while(blob_finder.next_4con_region(bregions))
      {
          int sizecount=0;
          for(unsigned i=0;i<bregions.size();i++)
              sizecount+=bregions[i].length();
          if(sizecount>min_size && sizecount<max_size)
              for(unsigned i=0;i<bregions.size();i++)
                  for(unsigned j=0;j<bregions[i].length();j++)
                      view_blobs(bregions[i].ilo+j,bregions[i].j)=true;
      }
      vil_image_view<unsigned char>* temp_out = new vil_image_view<unsigned char>(view_blobs.ni(),view_blobs.nj());

      for(unsigned i=0;i<view_blobs.ni();i++)
          for(unsigned j=0;j<view_blobs.nj();j++)
              (*temp_out)(i,j)=view_blobs(i,j)?255:0;

      pro.set_output_val<vil_image_view_base_sptr>(0, new vil_image_view<bool>(view_blobs));
      return true;
  }

  vcl_cout<<"Error! Require a boolean image "<<vcl_endl;
  return false;

}



