//:
// \file
#include <vcl_algorithm.h>
#include <btol/btol_vertex_algs.h>
#include <btol/btol_edge_algs.h>
#include <vtol/vtol_vertex.h>
#include <vtol/vtol_edge_sptr.h>


//: Destructor
btol_vertex_algs::~btol_vertex_algs()
{
}

//:
//-----------------------------------------------------------------------------
// Replaces va by vb on all the edges connected to va.
// The result is that vb has the union of the sets of 
// edges incident on va and vb as superiors.
//-----------------------------------------------------------------------------
bool btol_vertex_algs::merge_superiors(vtol_vertex_sptr& va,
                                       vtol_vertex_sptr& vb)
{
  if (!va||!vb)
    return false;
  vcl_vector<vtol_edge_sptr>* edges = va->edges();
  for (vcl_vector<vtol_edge_sptr>::iterator eit = edges->begin();
       eit != edges->end(); eit++)
    btol_edge_algs::subst_vertex_on_edge(va, vb, *eit);
  delete edges;
  return true;
}

static void vertex_erase(vcl_vector<vtol_vertex_sptr>& verts,
                         vtol_vertex_sptr& v)
{
  vcl_vector<vtol_vertex_sptr>::iterator vit =
    vcl_find(verts.begin(), verts.end(), v);
  if (vit == verts.end())
    return;
  verts.erase(vit);
  return;
}
