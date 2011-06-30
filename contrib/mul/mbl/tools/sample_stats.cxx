//////////////////////////////////////////////////////////////////////////
//
// Calculate various statistics on a sample of 1D data.
//
//////////////////////////////////////////////////////////////////////////

#include <vcl_iostream.h>
#include <vcl_iterator.h>
#include <vcl_fstream.h>
#include <vcl_string.h>
#include <vcl_exception.h>
#include <vcl_map.h>
#include <vcl_typeinfo.h>
#include <vul/vul_arg.h>
#include <vul/vul_sprintf.h>
#include <mbl/mbl_log.h>
#include <mbl/mbl_exception.h>
#include <mbl/mbl_sample_stats_1d.h>


//=========================================================================
// Static function to create a static logger when first required
//=========================================================================
static mbl_logger& logger()
{
  static mbl_logger l("mul.mbl.tools.sample_stats");
  return l;
}


//=========================================================================
// Report an error message in several ways, then throw an exception.
//=========================================================================
static void do_error(const vcl_string& msg)
{
  MBL_LOG(ERR, logger(), msg);
  throw mbl_exception_abort(msg);
}


//=========================================================================
// Actual main function (apart from exception-handling wrapper)
//=========================================================================
int main2(int argc, char *argv[])
{
  // Parse the program arguments
  vul_arg<vcl_string> in_file("-i", "input file containing scalar values (whitespace-separated); otherwise uses stdin", "");
  vul_arg<vcl_string> out_file("-o", "output file to append statistics; logger will be used otherwise", "");
  vul_arg<vcl_string> sep("-sep", "String to use as a separator between output values, e.g. \", \" or \"  \" (default=TAB)", "\t");
  vul_arg<bool> nohead("-h","Specify this to SUPPRESS column headers", false);
  vul_arg<bool> n("-n", "Specify this to record the number of samples", false);
  vul_arg<bool> mean("-mean", "Specify this to record the mean", false);
  vul_arg<bool> variance("-var", "Specify this to record the variance", false);
  vul_arg<bool> sd("-sd", "Specify this to record the standard deviation", false);
  vul_arg<bool> se("-se", "Specify this to record the standard error", false);
  vul_arg<bool> med("-med", "Specify this to record the median", false);
  vul_arg<bool> min("-min", "Specify this to record the minimum", false);
  vul_arg<bool> max("-max", "Specify this to record the maximum", false);
  vul_arg<vcl_vector<int> > pc("-pc", "Specify this switch with 1 or more comma-separated integers (e.g. 5,50,95) to record percentile(s)");
  vul_arg<vcl_vector<double> > quant("-q", "Specify this switch with 1 or more comma-separated floats (e.g. 0.05,0.50,0.95) to record quantile(s)");
  vul_arg<bool> sum("-sum", "Specify this to record the sum", false);
  vul_arg<bool> sum_squares("-ssq", "Specify this to record the sum of squares", false);
  vul_arg<bool> rms("-rms", "Specify this to record the rms", false);
  vul_arg<bool> mean_of_absolutes("-moa", "Specify this to record the mean_of_absolutes", false);
  vul_arg<bool> skewness("-skew", "Specify this to record the skewness", false);
  vul_arg<bool> kurtosis("-kurt", "Specify this to record the kurtosis", false);
  vul_arg<bool> absolute("-absolute", "Calculate statistics of absolute sample values", false);
  vul_arg<vcl_string> label("-label","Adds this label to each line outputting a statistic - useful for later grep");
  vul_arg_parse(argc, argv);

  // Try to open the input file if specified or use stdin
  vcl_istream* is=0;
  if (!in_file().empty())
  {
    is = new vcl_ifstream(in_file().c_str());
    if (!is || !is->good())
      do_error(vcl_string("Failed to open input file ") + in_file().c_str());
    MBL_LOG(DEBUG, logger(), "Opened input file: " << in_file().c_str());
  }
  else
  {
    is = &vcl_cin;
  }

  // Load the data from stream until end
  vcl_vector<double> data_vec;
  data_vec.assign(vcl_istream_iterator<double>(*is), vcl_istream_iterator<double>());
  if (data_vec.empty())
    do_error("Could not parse data file.");
  MBL_LOG(DEBUG, logger(), "data file contained " << data_vec.size() << " values.");
  
  if (absolute())
  {
    for (unsigned i=0;i<data_vec.size();++i) data_vec[i]=vcl_abs(data_vec[i]);
  }


  // Close the input filestream
  if (!in_file().empty()) (dynamic_cast<vcl_ifstream*>(is))->close();

  // Calculate the requested statistics
  mbl_sample_stats_1d data(data_vec);
  vcl_map<vcl_string, double> stats;
  if (n.set())                 stats["n"]=data.n_samples();
  if (mean.set())              stats["mean"]=data.mean();
  if (variance.set())          stats["var"]=data.variance();
  if (sd.set())                stats["sd"]=data.sd();
  if (se.set())                stats["se"]=data.stdError();
  if (med.set())               stats["med"]=data.median();
  if (min.set())               stats["min"]=data.min();
  if (max.set())               stats["max"]=data.max();
  if (sum.set())               stats["sum"]=data.sum();
  if (sum_squares.set())       stats["ssq"]=data.sum_squares();
  if (rms.set())               stats["rms"]=data.rms();
  if (mean_of_absolutes.set()) stats["moa"]=data.mean_of_absolutes();
  if (skewness.set())          stats["skew"]=data.skewness();
  if (kurtosis.set())          stats["kurt"]=data.kurtosis();
  if (pc.set())
  {
    for (vcl_vector<int>::const_iterator it=pc().begin(); it!=pc().end(); ++it)
    {
      vcl_string name = vul_sprintf("pc%02u", *it);
      stats[name] = data.nth_percentile(*it);
    }
  }
  if (quant.set())
  {
    for (vcl_vector<double>::const_iterator it=quant().begin(); it!=quant().end(); ++it)
    {
      vcl_string name = vul_sprintf("q%f", *it);
      stats[name] = data.quantile(*it);
    }
  }

  // Open output file if requested
  vcl_ofstream* ofs=0;
  if (!out_file().empty())
  {
    ofs = new vcl_ofstream(out_file().c_str(), vcl_ios::app);
    if (!ofs || !ofs->good())
      do_error(vcl_string("Failed to open outout file ") + out_file().c_str());
    MBL_LOG(DEBUG, logger(), "Opened output file: " << out_file().c_str());
  }

  // Output requested statistics
  MBL_LOG(INFO, logger(), "in_file: " << in_file());

  // Write a line of column headers unless suppressed
  if (!nohead())
  {
    if (ofs && ofs->good()) *ofs << '#' << sep();
    for (vcl_map<vcl_string,double>::const_iterator it=stats.begin(); it!=stats.end(); ++it)
    {
      if (ofs && ofs->good()) *ofs << it->first << sep();
    }
    if (ofs && ofs->good()) *ofs << '\n';
  }
  vcl_string my_label = in_file();
  if (label.set()) my_label = label();
  // Write statistics in a single line
  if (ofs && ofs->good()) *ofs << my_label << sep();
  for (vcl_map<vcl_string,double>::const_iterator it=stats.begin(); it!=stats.end(); ++it)
  {
    vcl_cout <<  my_label << " " << it->first << ": " << it->second << vcl_endl;
    if (ofs && ofs->good()) *ofs << it->second << sep();
  }
  if (ofs && ofs->good()) *ofs << vcl_endl;

  // Close file
  if (ofs) ofs->close();

  return 0;
}


//=========================================================================
// main() function with exception-handling wrapper
//=========================================================================
int main(int argc, char *argv[])
{
  int errcode = 0;

  // Initialize the logger
  mbl_logger::root().load_log_config_file();

  try
  {
    errcode = main2(argc, argv);
  }
  catch (const vcl_exception & e)
  {
    vcl_cerr << "ERROR: " << typeid(e).name() << '\n' <<
      e.what() << vcl_endl;
    errcode = 4;
  }

  return errcode;
}

