// This is ./gel/gevdl/gevd_region_proc_params.cxx

//:
// \file
// See gevd_region_proc_params.h
//
//-----------------------------------------------------------------------------

#include <gevd/gevd_region_proc_params.h>
#include <vcl_iostream.h>
#include <vcl_strstream.h>

//------------------------------------------------------------------------
// Constructors
//

gevd_region_proc_params::
gevd_region_proc_params(const gevd_region_proc_params& rpp)
{
  InitParams(rpp.expand_scale_,
             rpp.burt_adelson_factor_,
             rpp.debug_,
             rpp.verbose_,
             (gevd_detector_params)rpp.dp_
             );
}

gevd_region_proc_params::gevd_region_proc_params(float expand_scale,
                                       float burt_adelson_factor,
                                       bool debug,
                                       bool verbose,
                                       const gevd_detector_params& dp
                                       )
{
  InitParams(expand_scale, burt_adelson_factor, debug, verbose, dp);
}

void gevd_region_proc_params::InitParams(float expand_scale,
                                    float burt_adelson_factor,
                                    bool debug,
                                    bool verbose,
                                    const gevd_detector_params& dp)
{
 expand_scale_ = expand_scale;
 burt_adelson_factor_ = burt_adelson_factor;
 debug_ = debug;
 verbose_ = verbose;
 dp_ = dp;
}
//-----------------------------------------------------------------------------
//
//:   Checks that parameters are within acceptable bounds
//    Note that msg << ends seems to restart the string and erase the
//    previous string. We should only use it as the last call, use
//    vcl_endl otherwise.
bool gevd_region_proc_params::SanityCheck()
{
  vcl_strstream msg;
  bool valid = true;

  valid = valid && expand_scale_ >=0.5 && expand_scale_ <=2;
  if (!valid)
    msg << "currently only handle a scale factors of 0.5, 1.0, or 2.0 " << vcl_endl;

  valid = valid && burt_adelson_factor_ <= .6 && burt_adelson_factor_ >=.3;
  if (!valid)
    msg << "burt_adelson_factor must be in the range .3<=ka<=.6 "
        << vcl_endl;

  valid = valid && dp_.SanityCheck();
  msg << dp_.GetErrorMsg() << vcl_endl;

  msg << vcl_ends;//see comments above.

  SetErrorMsg(msg.str());
  delete [] msg.str();
  return valid;
}

vcl_ostream& operator << (vcl_ostream& os, const gevd_region_proc_params& rpp)
{
  os << "gevd_region_proc_params:" << vcl_endl << "[---" << vcl_endl;
  //os << rpp.dp_ << vcl_endl
  //   << "  ---" << vcl_endl;
  os << "expand scale factor  " << rpp.expand_scale_ << vcl_endl;
  os << "burt_adelson_factor " << rpp.burt_adelson_factor_ << vcl_endl;
  os << "debug " << rpp.debug_ << vcl_endl;
  os << "verbose " << rpp.verbose_ << vcl_endl;
  os << "---]" << vcl_endl;
  return os;
}

