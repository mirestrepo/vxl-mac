//-----------------------------------------------------------------------------
// .DESCRIPTION:
//   See vmal_convert.h
//-----------------------------------------------------------------------------

#include "vmal_convert.h"
#include <vtol/vtol_vertex_sptr.h>
#include <vtol/vtol_vertex_2d.h>
#include <vtol/vtol_zero_chain.h>
#include <vtol/vtol_edge_2d.h>
#include <vil/vil_memory_image_of.h>
#include <vil/vil_image_as.h>

vtol_vertex_2d_sptr convert_vertex_2d(osl_vertex & in)
{
  return new vtol_vertex_2d(in.GetX(),in.GetY());
}

vtol_edge_2d_sptr convert_edge_2d(osl_edge & in,vsol_curve_2d::vsol_curve_2d_type type)
{
  vtol_edge_2d_sptr out;

  if (type==vsol_curve_2d::CURVE_NO_TYPE)
  {
    float *x=in.GetX();
    float *y=in.GetY();
    vcl_vector<vtol_vertex_sptr> new_vertices;

    for (int i=0;i<in.size();i++)
    {
      vtol_vertex* temp_vertex=new vtol_vertex_2d(x[i],y[i]);
      new_vertices.push_back(temp_vertex);
    }
    vtol_zero_chain temp_zchain(new_vertices);
    out=new vtol_edge_2d(temp_zchain);
  }
  else if (type==vsol_curve_2d::LINE)
  {
    osl_vertex* v1=in.GetV1();
    osl_vertex* v2=in.GetV2();
    vtol_vertex_2d_sptr new_v1=convert_vertex_2d(*v1);
    vtol_vertex_2d_sptr new_v2=convert_vertex_2d(*v2);
    out=new vtol_edge_2d(*new_v1,*new_v2,0);
  }

  return out;
}

vcl_vector<vtol_edge_2d_sptr>* convert_vector_edge_2d(vcl_list<osl_edge*> & in,vsol_curve_2d::vsol_curve_2d_type type)
{
  vcl_list<osl_edge*>::iterator iter;
  vcl_vector<vtol_edge_2d_sptr>* out=new vcl_vector<vtol_edge_2d_sptr>();
  for (iter=in.begin();iter!=in.end();iter++)
  {
    vtol_edge_2d_sptr temp_edge_2d=convert_edge_2d(*(*iter),type);
    out->push_back(temp_edge_2d);
  }
  return out;
}

vcl_vector<vcl_vector<vtol_edge_2d_sptr>*>* convert_array_edge_2d(vcl_list<vcl_list<osl_edge *>*> & in,
                                                                  vsol_curve_2d::vsol_curve_2d_type type)
{
  vcl_list<vcl_list<osl_edge *>*>::iterator iter;
  vcl_vector<vcl_vector<vtol_edge_2d_sptr>*>* out=new vcl_vector<vcl_vector<vtol_edge_2d_sptr>*>;
  for (iter = in.begin();iter!=in.end();iter++)
    {
      vcl_vector<vtol_edge_2d_sptr>* vtol_lines=convert_vector_edge_2d(*(*iter),type);
      out->push_back(vtol_lines);
    }
  return out;
}

void convert_pointarray(vcl_vector<vtol_vertex_2d_sptr>& in,
              vcl_vector<HomgPoint2D> & out)
{
  vcl_vector<vtol_vertex_2d_sptr>::iterator iter;
  for (iter=in.begin();iter!=in.end();iter++)
  {
    HomgPoint2D temp((*iter)->x(),(*iter)->y(),1);
    out.push_back(temp);
  }
}

void convert_lines_double_3(vcl_vector<vtol_edge_2d_sptr> in,
              vnl_double_3 * &outp,
              vnl_double_3 * &outq)
{
  int numlines=in.size();
  outp=new vnl_double_3[numlines];
  outq=new vnl_double_3[numlines];

  for (int i=0;i<numlines;i++)
  {
    outp[i][0]=in[i]->v1()->cast_to_vertex_2d()->x();
    outp[i][1]=in[i]->v1()->cast_to_vertex_2d()->y();
    outp[i][2]=1;

    outq[i][0]=in[i]->v2()->cast_to_vertex_2d()->x();
    outq[i][1]=in[i]->v2()->cast_to_vertex_2d()->y();
    outq[i][2]=1;
  }
}

void convert_points_double_3(vcl_vector<vtol_vertex_2d_sptr> in,
                             vnl_double_3 * &out)
{
  int numpoints=in.size();
  out=new vnl_double_3[numpoints];

  for (int i=0;i<numpoints;i++)
  {
    out[i][0]=in[i]->x();
    out[i][1]=in[i]->y();
    out[i][2]=1;
  }
}

void convert_points_vect_double_3(vcl_vector<vtol_vertex_2d_sptr> & in,
                                  vcl_vector<vnl_double_3> & out)
{
  int numpoints=in.size();
  for (int i=0;i<numpoints;i++)
  {
    vnl_double_3 pt;
    pt[0]=in[i]->x();
    pt[1]=in[i]->y();
    pt[2]=1;
    out.push_back(pt);
  }
}

void convert_line_double_3(vtol_edge_2d_sptr in,
                           vnl_double_3 &outp,
                           vnl_double_3 &outq)
{
  outp[0]=in->v1()->cast_to_vertex_2d()->x();
  outp[1]=in->v1()->cast_to_vertex_2d()->y();
  outp[2]=1;

  outq[0]=in->v2()->cast_to_vertex_2d()->x();
  outq[1]=in->v2()->cast_to_vertex_2d()->y();
  outq[2]=1;
}

void convert_line_double_2(vtol_edge_2d_sptr in,
                           vnl_double_2 &outp,
                           vnl_double_2 &outq)
{
  outp[0]=in->v1()->cast_to_vertex_2d()->x();
  outp[1]=in->v1()->cast_to_vertex_2d()->y();

  outq[0]=in->v2()->cast_to_vertex_2d()->x();
  outq[1]=in->v2()->cast_to_vertex_2d()->y();
}

void convert_point_double_3(vtol_vertex_2d_sptr in,
                            vnl_double_3 &out)
{
  out[0]=in->x();
  out[1]=in->y();
  out[2]=1;
}

void convert_grey_memory_image(const vil_image & image,
                               vil_memory_image_of<vil_byte> &ima_mono)
{
  int w=image.width();
  int h=image.height();

  ima_mono.resize(w,h);

  vil_image_as_byte(image).get_section(ima_mono.get_buffer(), 0, 0, w, h);
}

