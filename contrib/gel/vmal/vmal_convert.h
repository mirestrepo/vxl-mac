#ifndef vmal_convert_h_
#define vmal_convert_h_
//------------------------------------------------------------------------------
// .NAME        vmal_convert
// .INCLUDE     xcv/xcv_display.h
// .FILE        vmal_convert.cxx
// .SECTION Description:
//   Toolbox to make conversion from osl to vtol, vil_image to vil_memory_image,
//
// .SECTION Author
//   L. Guichard
// .SECTION Modifications:
//------------------------------------------------------------------------------

#include <vtol/vtol_vertex_2d.h>
#include <vtol/vtol_edge_2d.h>

#include <osl/osl_vertex.h>
#include <osl/osl_edge.h>

#include <vsol/vsol_curve_2d.h>

#include <mvl/HomgPoint2D.h>

#include <vnl/vnl_double_3.h>

#include <vil/vil_image.h>
#include <vil/vil_byte.h>
#include <vil/vil_memory_image_of.h>

vtol_vertex_2d_sptr convert_vertex_2d(osl_vertex & in);

//Convert a osl_edge to a vtol_edge_2d.
//If the osl_edge is a line then set type=LINE.
//If you don't know anything about the shape of the curve, set type=CURVE_NO_TYPE (default)
vtol_edge_2d_sptr convert_edge_2d(osl_edge & in,vsol_curve_2d::vsol_curve_2d_type type=vsol_curve_2d::CURVE_NO_TYPE);

vcl_vector<vtol_edge_2d_sptr>* convert_vector_edge_2d(vcl_list<osl_edge*> & in,
                                                      vsol_curve_2d::vsol_curve_2d_type type=vsol_curve_2d::CURVE_NO_TYPE);

vcl_vector<vcl_vector<vtol_edge_2d_sptr>*>* convert_array_edge_2d(vcl_list<vcl_list<osl_edge *>*> & in,
                                                                  vsol_curve_2d::vsol_curve_2d_type type);

void convert_pointarray(vcl_vector<vtol_vertex_2d_sptr>& in,vcl_vector<HomgPoint2D> & out);

void convert_lines_double_3(vcl_vector<vtol_edge_2d_sptr> in,
              vnl_double_3 * &outp,
              vnl_double_3 * &outq);

void convert_points_vect_double_3(vcl_vector<vtol_vertex_2d_sptr> & in,
                  vcl_vector<vnl_double_3> & out);

void convert_points_double_3(vcl_vector<vtol_vertex_2d_sptr> in,
               vnl_double_3 * &out);

//Convert a vtol_edge_2d to two vnl_double_3 representing its end-points.
void convert_line_double_3(vtol_edge_2d_sptr in,
               vnl_double_3 &outp,
               vnl_double_3 &outq);

void convert_line_double_2(vtol_edge_2d_sptr in,
               vnl_double_2 &outp,
               vnl_double_2 &outq);

void convert_point_double_3(vtol_vertex_2d_sptr in,
              vnl_double_3 &out);

void convert_grey_memory_image(const vil_image & image,
                 vil_memory_image_of<vil_byte> &ima_mono);

#endif // vmal_convert
