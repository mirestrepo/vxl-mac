//:
// \file
// \brief Example of loading an image and convolving with a mask
// \author Tim Cootes

#include <vcl_iostream.h>
#include <vxl_config.h> // for vxl_byte
#include <testlib/testlib_root_dir.h>
#include <vil2/vil2_load.h>
#include <vil2/vil2_save.h>
#include <vil2/vil2_image_view.h>
#include <vil2/algo/vil2_normalised_correlation_2d.h>
#include <vil2/algo/vil2_sobel_3x3.h>
#include <vil2/vil2_convert.h>
#include <vil2/vil2_print.h>

int main(int argc, char** argv)
{
  if (argc!=4)
	{
	  vcl_cout<<"vil2_convolve_tool  src_image kernel_image dest_image"<<vcl_endl;
		vcl_cout<<"Convolves kernel_image with src_image, saves to dest_image"<<vcl_endl;
		return 0;
  }

  vil2_image_view<vxl_byte> src_im = vil2_load(argv[1]);
	if (src_im.size()==0)
	{
	  vcl_cout<<"Unable to load source image from "<<argv[1]<<vcl_endl;
    return 1;
  }

  vil2_image_view<vxl_byte> kernel_im_byte = vil2_load(argv[2]);
	if (kernel_im_byte.size()==0)
	{
	  vcl_cout<<"Unable to load kernel image from "<<argv[1]<<vcl_endl;
    return 1;
  }

	vcl_cout<<"Kernel image is "<<kernel_im_byte.ni()<<" x "<<kernel_im_byte.nj()<<vcl_endl;

	// Create a normalised kernel image
  vil2_image_view<float> kernel_image;
	vil2_convert_cast(kernel_im_byte,kernel_image);
	float mean,var;
  vil2_math_mean_and_variance(mean,var,kernel_image,0);
	double ks = 1.0/vcl_sqrt(var);
	vil2_math_scale_and_offset_values(kernel_image,ks,-ks*mean);

  vil2_image_view<float> dest_image;

	vil2_normalised_correlation_2d(src_im,dest_image,kernel_image,double());

	// Stretch range to [0,255]
	float min_v,max_v;
	vil2_math_value_range(dest_image,min_v,max_v);
	double s = 255/(max_v-min_v);
	vil2_math_scale_and_offset_values(dest_image,s,-s*min_v);
	vcl_cout<<"Range of result: "<<min_v<<","<<max_v<<vcl_endl;

  vil2_image_view<vxl_byte> result_image;
  vil2_convert_cast(dest_image,result_image);

	vcl_cout<<"Resulting image is "<<result_image.ni()<<" x "<<result_image.nj()<<vcl_endl;

  if (!vil2_save(result_image, argv[3]))
	{
	  vcl_cerr<<"Unable to save result image to "<<argv[3]<<vcl_endl;
    return 1;
  }

	vcl_cout<<"Saved result of convolution to "<<argv[3]<<vcl_endl;

  return 0;
}
