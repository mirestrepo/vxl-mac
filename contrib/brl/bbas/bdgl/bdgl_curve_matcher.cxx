//-----------------------------------------------------------------------------
//:
// \file
// \author P.L. Bazin
//-----------------------------------------------------------------------------

#include "bdgl_curve_matcher.h"
#include <vdgl/vdgl_edgel_chain.h>
#include <vdgl/vdgl_edgel_chain_sptr.h>
#include <vcl_cmath.h>
#include <vnl/vnl_math.h>
#include <bdgl/bdgl_curve_description.h>
#include <vcl_vector.h>
#include <vcl_utility.h>
#include "curveMatch.h"

void bdgl_curve_matcher::match()
{
  // Transfer into a single curve
  vdgl_edgel_chain_sptr  edge1 = primitive_curve_.get_curve();

  // compute description
  bdgl_curve_description desc1(edge1);
  bdgl_curve_description desc2(image_curve_);

  //desc1.info();
  //desc2.info();

  // distance ...
  double dist;
  dist = 1.0*vcl_sqrt( vnl_math_sqr( (desc1.center_.x()-desc2.center_.x())/image_scale() )
                      +vnl_math_sqr( (desc1.center_.y()-desc2.center_.y())/image_scale() ) );
  dist+= 0.5*vnl_math_abs( (desc1.length_-desc2.length_)/image_scale() );
  dist+= 0.5*vnl_math_abs( (desc1.curvature_-desc2.curvature_)/image_scale() );

  dist+= 1.0*vnl_math_abs( (desc1.gradient_mean_val_-desc2.gradient_mean_val_)/grad_scale() );
  dist+= 1.0*vnl_math_abs( (desc1.gradient_mean_dir_-desc2.gradient_mean_dir_)/angle_scale() );
  dist+= 1.0*vnl_math_abs( (desc1.gradient_std_val_-desc2.gradient_std_val_)/grad_scale() );
  dist+= 1.0*vnl_math_abs( (desc1.gradient_std_dir_-desc2.gradient_std_dir_)/angle_scale() );

  matching_score_ = dist;
  matching_flag_ = 1;
 Tx_= desc2.center_.x()-desc1.center_.x();
 Ty_= desc2.center_.y()-desc1.center_.y();
 Rtheta_=0;
  return;
}
void bdgl_curve_matcher::match_DP()
{

  vcl_pair<double,double> coordinate;
  vcl_vector<vcl_pair<double,double> > v1,v2;
  int i;

  vdgl_edgel_chain_sptr  edge_primitive_curve = primitive_curve_.get_curve();
  vdgl_edgel_chain_sptr  edge_image_curve = image_curve_;
  int len_primitive_curve=edge_primitive_curve->size();
  int len_image_curve=edge_image_curve->size();
    //bdgl_curve_description desc1(edge_primitive_curve);
    //bdgl_curve_description desc2(edge_image_curve);
  for (i=0;i<len_primitive_curve;i++)
  {
    coordinate.first=edge_primitive_curve->edgel(i).get_x();
    coordinate.second=edge_primitive_curve->edgel(i).get_y();
    v1.push_back(coordinate);
  }
  for (i=0;i<len_image_curve;i++)
  {
    coordinate.first=edge_image_curve->edgel(i).get_x();
    coordinate.second=edge_image_curve->edgel(i).get_y();
    v2.push_back(coordinate);
  }
  // writing curves to a file
#if 0 // commented out
  vcl_ofstream of1("c:\\curve1.con");
  of1<<"CONTOUR";
  of1<<"\nOPEN";
  of1<<"\n"<<v1.size()<<"\n";
  for (i=0;i<v1.size();i++)
  {
    of1<<v1[i].first<<"\t"<<v1[i].second<<"\t";

  }
  of1.close();
  //reverse(v2.begin(),v2.end());
  vcl_ofstream of2("C:\\curve2.con");
  of2<<"CONTOUR";
  of2<<"\nOPEN";
  of2<<"\n"<<v2.size()<<"\n";
  for (i=0;i<v2.size();i++)
  {
    of2<<v2[i].first<<"\t"<<v2[i].second<<"\t";
  }
  of2.close();
#endif // 0

  // deciding the orientation ofthe curves to be matched

  double e1=(v1[0].first -v2[0].first )*(v1[0].first -v2[0].first )
           +(v1[0].second-v2[0].second)*(v1[0].second-v2[0].second);
  double e2=(v1[0].first -v2[v2.size()-1].first )*(v1[0].first -v2[v2.size()-1].first )
           +(v1[0].second-v2[v2.size()-1].second)*(v1[0].second-v2[v2.size()-1].second);

  if (e1<e2)
    matching_score_ = curveMatch(v1,v2,Tx_,Ty_,Rtheta_);
  else
  {
    reverse(v2.begin(),v2.end());
    matching_score_ = curveMatch(v1,v2,Tx_,Ty_,Rtheta_);
  }
}
