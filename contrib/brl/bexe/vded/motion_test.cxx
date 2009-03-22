//:
// \file
#include <vcl_string.h>
#include <vcl_fstream.h>
#include <vcl_sstream.h>
#include <vul/vul_arg.h>
#include <vidl1/vidl1_movie.h>
#include <vidl1/vidl1_clip.h>
#include <vidl1/vidl1_io.h>
#include <vil/vil_image_view.h>
#include <vil/vil_convert.h>
#include <vil/vil_math.h>
#include <vil/vil_new.h>
#include <vil/algo/vil_gauss_filter.h>
#include <vil/vil_image_resource.h>
#include <brip/brip_vil_ops.h>


#include <vil/algo/vil_threshold.txx>
VIL_THRESHOLD_INSTANTIATE(float);

bool print_xml_params( vcl_string output_file,
                       vul_arg_info_list& arg_list,
                       vcl_string param_block_name );

bool print_xml_performance( vcl_string output_file,
                            vcl_string video_file,
                            const vcl_vector< double >& scores );

void write_status(vcl_string output_file, int iframe, int nframes);


int main(int argc, char** argv)
{
  // Arguments
  vul_arg_info_list arg_list;
  vul_arg<double> sigma(arg_list,"-sigma","sigma for Gaussian smoothing",1.0);
  vul_arg<float> thresh1(arg_list,"-t1","Threshold for truncating motion",0.001f);
  vul_arg<float> thresh2(arg_list,"-t2","Threshold for detecting motion",0.0001f);
  vul_arg<bool> no_output(arg_list,"-no_output","Disable output",false);
  vul_arg<vcl_string> parameter_output_file(arg_list,"-X","parameter output file","");
  vul_arg<vcl_string> input_video_file(arg_list,"-V","video input file","");
  vul_arg<vcl_string> status_block_file(arg_list,"-S","status block file","");
  vul_arg<vcl_string> performance_output_file(arg_list,"-P","performance output file","");
  vul_arg<vcl_string> output_directory(arg_list,"-O","output directory","");
  vul_arg<vcl_string> output_video_directory(arg_list,"-R","results video directory","");
  vul_arg<bool> print_params_only(arg_list,"-print_params_only",
                                           "print xml parameter file and exit",false);
  vul_arg_include(arg_list);
  vul_arg_parse(argc, argv);

  // if print_xml_params returns 0, exit with no error
  if (parameter_output_file() != "")
    print_xml_params(parameter_output_file(), arg_list, "motion_test_params");
  if ( print_params_only() )
    return 0;

  // Open the movie
  vidl1_movie_sptr video = vidl1_io::load_movie(input_video_file().c_str());
  if (!video) {
    vcl_cerr << "Failed to open the video\n";
    return -1;
  }

  bool make_output_video = (output_video_directory() != "") && (!no_output());

  // create a gaussian filter with sigma()
  vil_gauss_filter_5tap_params gauss_params(sigma());

  vil_image_view<float> prev_image;
  vcl_vector< double > scores;
  vcl_vector< vil_image_resource_sptr > results;
  for ( vidl1_movie::frame_iterator f_itr = video->first();
        f_itr != video->end();  ++f_itr )
  {
    vil_image_view<vxl_byte> input_image;
    vil_image_view<float> smooth_image;
    input_image = vil_convert_to_grey_using_rgb_weighting(f_itr->get_view());
    vil_gauss_filter_5tap(input_image, smooth_image, gauss_params);

    if ( f_itr == video->first() ){
      prev_image = smooth_image;
      continue;
    }

    vil_image_view<float> sing_img;
    brip_sqrt_grad_singular_values(smooth_image, sing_img, 1);

    vil_image_view<float> diff_img;
    vil_math_image_abs_difference(smooth_image, prev_image, diff_img);

    vil_image_view<float> motion_img;
    vil_math_image_product(diff_img, sing_img, motion_img);

    float min_val, max_val;
    vil_math_value_range(motion_img, min_val, max_val);
    vil_math_scale_values(motion_img, 1.0/max_val);
    vcl_cout << "min: " << min_val << " max: "<< max_val<<vcl_endl;

    vil_image_view<bool> bool_img;
    vil_threshold_above( motion_img, bool_img, thresh2());

    int sum = 0;
    vil_math_sum(sum, bool_img, 0);
    int total = bool_img.ni()*bool_img.nj();
    vcl_cout << "Sum: "<<sum<<" total: "<< total << " ratio: "<< double(sum)/total << vcl_endl;
    scores.push_back(double(sum)/total);

    // Make the output video frame
    if ( make_output_video ) {
      vil_math_truncate_range( motion_img, 0.0f, thresh1());
      vil_image_view<vxl_byte> byte_img;
      vil_convert_stretch_range(motion_img, byte_img);
      vil_image_resource_sptr img_sptr = vil_new_image_resource(byte_img.ni(), byte_img.nj(),
                                                              byte_img.nplanes(), byte_img.pixel_format());
      img_sptr->put_view(byte_img);
      results.push_back(img_sptr);
    }

    if (status_block_file() != "")
      write_status(status_block_file(), scores.size(), video->length()-1);

    prev_image = smooth_image;
  }
  if (performance_output_file() != "")
    print_xml_performance( performance_output_file(), input_video_file(), scores );

  if ( make_output_video ) {
    vidl1_movie_sptr result_movie = new vidl1_movie(new vidl1_clip(results, 0, results.size()));
    vcl_string output_name = output_video_directory()+"/output";
    vidl1_io::save(result_movie.ptr(), output_name.c_str(), "ImageList");
  }

  vcl_cout << "done!\n";
  return 0;
}


//: print the performance to an XML file
bool print_xml_performance( vcl_string output_file,
                            vcl_string video_file,
                            const vcl_vector< double >& scores )
{
  vcl_ofstream outstream(output_file.c_str());
  if (!outstream)
  {
    vcl_cerr << "error: could not create performance output file ["<<output_file<<"]\n";
    return false;
  }
  outstream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            << "<performance>\n"
            << "  <description>Video Frame Scores</description>\n"
            << "  <frames>\n"
            << "    <video name=\""<< video_file <<"\" totalframes=\""<<scores.size()<<"\">\n"
            << "      <category type=\"car\">\n";
  for (unsigned int i=0; i<scores.size(); ++i) {
    outstream << "        <frame index=\""<<i<<"\" score=\""<<scores[i]<<"\"/>\n";
  }

  outstream << "      </category>\n"
            << "    </video>\n"
            << "  </frames>\n"
            << "</performance>\n";

  return true;
}

void write_status(vcl_string output_file, int iframe, int nframes)
{
  float percent_done = float(iframe)/float(nframes);

  vcl_ofstream outstream(output_file.c_str());
  outstream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            << "<status>\n"
            << "  <basic>\n"
            << "    <info percent_completed=\""<<percent_done<<"\"/>\n"
            << "  </basic>\n"
            << "  <optional>\n"
            << "    <info number_of_frames_processed=\""<<iframe<<"\"/>\n"
            << "    <info total_number_of_frames=\""<<nframes<<"\"/>\n"
            << "  </optional>\n"
            << "</status>\n";

  outstream.close();
}


//: print the parameters to an XML file
bool print_xml_params( vcl_string output_file,
                       vul_arg_info_list& arg_list,
                       vcl_string param_block_name )
{
  vcl_ofstream outstream(output_file.c_str());
  if (!outstream)
  {
    vcl_cerr << "error: could not create param output file ["<<output_file<<"]\n";
    return false;
  }
  int n_args = arg_list.args_.size();

  outstream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            << "<vxl>\n"
            << "  <params app=\""<<param_block_name<<"\" nparams=\""<<n_args<<"\">\n";

  for (vcl_vector<vul_arg_base*>::iterator arg_it = arg_list.args_.begin();
       arg_it != arg_list.args_.end(); ++arg_it)
  {
    vcl_string command = (*arg_it)->option();
    vcl_string description = (*arg_it)->help();
    vcl_string type = (*arg_it)->type_;
    if (type == "bool")
      type = "flag";

    outstream << "    <param command=\"" << command << "\" description=\""
              << description << "\" type=\"" << type << '\"';
    if (command == "-V")
      outstream << " System_info=\"INPUT_VIDEO\"";
    if (command == "-O")
      outstream << " System_info=\"OUTPUT\"";
    if (command == "-R")
      outstream << " System_info=\"OUTPUT_VIDEO\"";
    if (command == "-S")
      outstream << " System_info=\"STATUS_BLOCK\"";
    if (command == "-P")
      outstream << " System_info=\"PERFORMANCE_OUTPUT\"";
#if 0
    // see if param is an output file
    for (vcl_vector<vul_arg<vcl_string> >::iterator pit = output_files_.begin();
         pit != output_files_.end(); pit++)
    {
      vcl_string of_cmd = ( (*pit).option() );
      if (command == of_cmd)
      {
        outstream << " System_info=\"OUTPUT\"";
        break;
      }
    }
#endif // 0
    outstream << " value=\"";
    // if arg is a string, we have to get rid of ' ' around value
    vcl_ostringstream value_stream;
    (*arg_it)->print_value(value_stream);
    if (type == "string")
    {
      vcl_string value_string = value_stream.str();
      value_string.replace(value_string.find("'"),1,"");
      value_string.replace(value_string.find("'"),1,"");
      outstream << value_string;
    }
    else if (type == "flag")
    {
      vcl_string value_string = value_stream.str();
      if ((value_string == "not set") || (command == "-print_params_only"))
        value_string = "off";
      else if (value_string == "set")
        value_string = "on";
      else
        value_string = "unknown";
      outstream << value_string;
    }
    else
      outstream << value_stream.str();
    outstream << "\" />\n";
  }
  outstream << "  </params>\n"
            << "</vxl>\n";

  outstream.close();
  return true;
}

