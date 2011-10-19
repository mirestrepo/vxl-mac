// This is brl/bpro/core/bbas_pro/processes/brad_estimate_phongs_model_process.cxx
#include <bprb/bprb_func_process.h>
#include <bpro/core/bbas_pro/bbas_1d_array_float.h>
#include <brad/brad_phongs_model_est.h>
#include <vcl_algorithm.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vnl/vnl_math.h>
//:
// \file
namespace brad_estimate_phongs_model_process_globals
{
  const unsigned n_inputs_  = 6;
  const unsigned n_outputs_ = 6;
}


//: Constructor
bool brad_estimate_phongs_model_process_cons(bprb_func_process& pro)
{
  using namespace brad_estimate_phongs_model_process_globals;

  vcl_vector<vcl_string> input_types_(n_inputs_);
  input_types_[0] = "bbas_1d_array_float_sptr";
  input_types_[1] = "bbas_1d_array_float_sptr";
  input_types_[2] = "bbas_1d_array_float_sptr";
  input_types_[3] = "bbas_1d_array_float_sptr";
  input_types_[4] = "float";
  input_types_[5] = "float";

  vcl_vector<vcl_string>  output_types_(n_outputs_);
  output_types_[0] = "bbas_1d_array_float_sptr";
  output_types_[1] = "float";
  output_types_[2] = "float";
  output_types_[3] = "float";
  output_types_[4] = "float";
  output_types_[5] = "float";

  return pro.set_input_types(input_types_) &&
         pro.set_output_types(output_types_);
}


//: Execute the process
bool brad_estimate_phongs_model_process(bprb_func_process& pro)
{
  // Sanity check
  if (pro.n_inputs()< 6) {
    vcl_cout << "brip_extrema_process: The input number should be 6" << vcl_endl;
    return false;
  }

  // get the inputs
  unsigned i=0;
  bbas_1d_array_float_sptr intensities = pro.get_input<bbas_1d_array_float_sptr>(i++);
  bbas_1d_array_float_sptr visibilities = pro.get_input<bbas_1d_array_float_sptr>(i++);
  bbas_1d_array_float_sptr camera_elev_array = pro.get_input<bbas_1d_array_float_sptr>(i++);
  bbas_1d_array_float_sptr camera_azim_array = pro.get_input<bbas_1d_array_float_sptr>(i++);
  float sun_elev = pro.get_input<float>(i++);
  float sun_azim = pro.get_input<float>(i++);

  unsigned num_samples=intensities->data_array.size();
  vnl_vector<double> samples(num_samples);
  vnl_vector<double> samples_weights(num_samples);
  vnl_vector<double> camera_elev(num_samples);
  vnl_vector<double> camera_azim(num_samples);

  float mean_intensities = 0.0f ;
  float sum_weights = 0.0f ;
  for (unsigned i=0;i<num_samples;i++)
  {
    camera_elev[i]    =camera_elev_array->data_array[i];
    camera_azim[i]    =camera_azim_array->data_array[i];

    samples[i]        =intensities->data_array[i];
    samples_weights[i]=visibilities->data_array[i];
    if (samples[i] <0.0 || samples[i] > 1.0 )
      samples_weights[i] = 0.0;
    mean_intensities += (samples_weights[i]* samples[i]);
    sum_weights      +=  samples_weights[i];
  }
  brad_phongs_model_est f(sun_elev,sun_azim,
                          camera_elev,camera_azim,
                          samples,samples_weights,true);
  vnl_levenberg_marquardt lm(f);
  lm.set_verbose(true);
  //: initialize diffusion component to be mean of the intensities
  vnl_vector<double> x(5);
  x[0] = mean_intensities/sum_weights;
  x[1] = 0.0;
  x[2] = 4.0;
  x[3] = 0.0;
  x[4] = 0.0;
  lm.minimize(x);

  vcl_cout<<"\n Phong's Model : "
          <<vcl_fabs(x[0])<<','
          <<vcl_fabs(x[1])<<','
          <<x[2]<<','<<x[3]<<','<<x[4]<<'\n'
          <<"St Error "<<f.error_var(x)<<'\n';

  brad_phongs_model pm(vcl_fabs(x[0]),vcl_fabs(x[1]),x[2],x[3],x[4]);
  bbas_1d_array_float_sptr new_obs = new bbas_1d_array_float(num_samples);
  for (unsigned i=0;i<num_samples;++i)
    new_obs->data_array[i]=pm.val(camera_elev[i],camera_azim[i],sun_elev,sun_azim);
  for (unsigned i=0;i<num_samples;++i)
      if(samples_weights[i] > 0.0)
        new_obs->data_array[i]=pm.val(camera_elev[i],camera_azim[i],sun_elev,sun_azim);

  i=0;
  pro.set_output_val<bbas_1d_array_float_sptr>(i++, new_obs);
  pro.set_output_val<float>(i++, x[0]);
  pro.set_output_val<float>(i++, x[1]);
  pro.set_output_val<float>(i++, x[2]);
  pro.set_output_val<float>(i++, x[3]);
  pro.set_output_val<float>(i++, x[4]);
  return true;
}

