// This is brl/vvid/vvid_line_fit_process.cxx
#include <vcl_iostream.h>
#include <vul/vul_timer.h>
#include <vtol/vtol_edge_2d_sptr.h>
#include <vtol/vtol_edge_2d.h>
#include <sdet/sdet_fit_lines.h>
#include <vvid/vvid_line_fit_process.h>

vvid_line_fit_process::vvid_line_fit_process(sdet_fit_lines_params & flp)
  : sdet_fit_lines_params(flp)
{
}

vvid_line_fit_process::~vvid_line_fit_process()
{
}

bool vvid_line_fit_process::execute()
{
  vul_timer t;
  
  if (!get_N_input_topo_objs())
    {
      vcl_cout << "In vvid_line_fit_process::execute() - no input edges\n";
      return false;
    }
  this->clear_output();
  //initialize the line fitter
  sdet_fit_lines fitter(*((sdet_fit_lines_params*)this));
  vcl_vector<vtol_edge_2d_sptr> edges;
  for(vcl_vector<vtol_topology_object_sptr>::iterator eit = input_topo_objs_.begin();
      eit != input_topo_objs_.end(); eit++)
    {
      vtol_edge_2d_sptr e = (*eit)->cast_to_edge()->cast_to_edge_2d();
      if(e)
        edges.push_back(e);
    }
  fitter.set_edges(edges);
  if(!fitter.fit_lines())
    return false;
  vcl_vector<vsol_line_2d_sptr> & lines = fitter.get_line_segs();
  for (vcl_vector<vsol_line_2d_sptr>::iterator lit = lines.begin();
        lit != lines.end(); lit++)
		{
		vsol_line_2d* l2d = (*lit).ptr();
        output_spat_objs_.push_back((vsol_spatial_object_2d*)l2d);
		}
  
  vcl_cout << "process " << lines.size()
           << " line segments in " << t.real() << " msecs.\n";
  return true;
}
