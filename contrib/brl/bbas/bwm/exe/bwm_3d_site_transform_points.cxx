//:
// \file
#include <bwm/bwm_observer_cam.h>
#include <bwm/bwm_observer_mgr.h>
#include <bwm/bwm_3d_corr.h>
#include <bwm/bwm_3d_corr_sptr.h>
#include <vcl_vector.h>
#include <vcl_set.h>
#include <vcl_cassert.h>
#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_string.h>
#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include <vul/vul_file_iterator.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/algo/vgl_rotation_3d.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vpgl/algo/vpgl_ortho_procrustes.h>

#include <rply.h>   //.ply parser


//helper class to read in bb from file
class ply_points_reader
{
 public:
  vcl_vector<vnl_vector_fixed<double,3> > all_points;
  vnl_vector_fixed<double,3> p;
  vcl_vector<int > vertex_indices;
};

//: Call-back function for a "vertex" element
int plyio_vertex_cb(p_ply_argument argument)
{
  long index;
  void* temp;
  ply_get_argument_user_data(argument, &temp, &index);

  ply_points_reader* parsed_ply =  (ply_points_reader*) temp;

  switch (index)
  {
    case 0: // "x" coordinate
      parsed_ply->p[0] = ply_get_argument_value(argument);
      break;
    case 1: // "y" coordinate
      parsed_ply->p[1] = ply_get_argument_value(argument);
      break;
    case 2: // "z" coordinate
		{
		 parsed_ply->p[2] = ply_get_argument_value(argument);
		 // Instert into vector
		 parsed_ply->all_points.push_back(parsed_ply->p);
		 break;
		}
    default:
      assert(!"This should not happen: index out of range");
  }
  return 1;
}


void readPointsFromPLY(const vcl_string& filename, vcl_vector<vnl_vector_fixed<double,3> > &all_points)
{
  ply_points_reader parsed_ply;
  parsed_ply.all_points = all_points;

  p_ply ply = ply_open(filename.c_str(), NULL, 0, NULL);
  if (!ply) {
    vcl_cout << "File " << filename << " doesn't exist.";
  }
  if (!ply_read_header(ply))
    vcl_cout << "File " << filename << " doesn't have header.";

  // vertex
  int  nvertices =
  ply_set_read_cb(ply, "vertex", "x", plyio_vertex_cb, (void*) (&parsed_ply), 0);
  ply_set_read_cb(ply, "vertex", "y", plyio_vertex_cb, (void*) (&parsed_ply), 1);
  ply_set_read_cb(ply, "vertex", "z", plyio_vertex_cb, (void*) (&parsed_ply), 2);

  vcl_cerr << nvertices << " points \n";

  // Read DATA
  ply_read(ply);

  // CLOSE file
  ply_close(ply);

  all_points=parsed_ply.all_points;
}


//: Write points to a PLY file
void writePointsToPLY(const vcl_string& ply_file_out, vcl_vector<vnl_vector_fixed<double,3> > &all_points)
{
	  // OPEN output file
  p_ply oply = ply_create(ply_file_out.c_str(), PLY_ASCII, NULL, 0, NULL);
  
  vcl_cerr << "  saving " << ply_file_out << " :\n";
  
  // HEADER SECTION
  // vertex
  ply_add_element(oply, "vertex", all_points.size());
  ply_add_scalar_property(oply, "x", PLY_DOUBLE); //PLY_FLOAT
  ply_add_scalar_property(oply, "y", PLY_DOUBLE); //PLY_FLOAT
  ply_add_scalar_property(oply, "z", PLY_DOUBLE); //PLY_FLOAT
  // comment
  ply_add_comment(oply, "created by bwm_transform_site_3d");
  // object info
  ply_add_obj_info(oply, "a vector of vnl_vector_fixed<double,3> object");
  // end header
  ply_write_header(oply);
  
  // DATA SECTION
  // save min and max boint of the box to ply file
  for (unsigned pi=0; pi<all_points.size(); ++pi) {
    vnl_vector_fixed<double,3> p = all_points[pi];
    ply_write(oply, p[0]);
    ply_write(oply, p[1]);
    ply_write(oply, p[2]);
  }
 // CLOSE PLY FILE
  ply_close(oply);
}

// the resulting similarity maps from the coordinate frame of pts1
// to the coordinate frame of pts0
static bool compute_similarity(vnl_matrix<double> const& pts0,
                               vnl_matrix<double> const& pts1,
                               vgl_rotation_3d<double>& R,
                               vnl_vector_fixed<double, 3>& t,
                               double& scale)
{
  vpgl_ortho_procrustes op(pts0, pts1);
  R = op.R();
  t = op.t();
  scale = op.s();
  if (! op.compute_ok()) return false;
  vcl_cout << "Ortho procrustes error "
           << vcl_sqrt(op.residual_mean_sq_error()) << '\n';
  return true;
}

// this executable finds a similarity transform, given a set of corresponding
// 3-d points. The similiarity transform is then applied to a directory of
// PLY files containing lists of 3d points that are transformed to the new coordinate system
int main(int argc, char** argv)
{
  //Get Inputs
  vul_arg<vcl_string> corrs_path   ("-corrs", "corr input file",  "");
  vul_arg<vcl_string> input_path ("-input_path","directory to get .ply files containing the points","");
  vul_arg<vcl_string> output_path ("-output_path","directory to store transformed points", "");
  vul_arg<vcl_string> tform_path ("-transform_path","file to save the transformation file (don't add extension)", "");
  vul_arg<vcl_string> pts0_path ("-pts0_path","file to save the first points as .ply", "");
  vul_arg<vcl_string> pts1_path ("-pts1_path","file to save the sceond points as .ply", "");


  if (argc < 1) {
    vcl_cout << "usage: bwm_3d_site_transform -corrs <corr file> -in_point_dir <dir> -out_point_dir <dir> -transform_path <tfrom path>\n";
    return -1;
  }

  vul_arg_parse(argc, argv);

  vcl_vector<bwm_3d_corr_sptr> corrs;
  vcl_cout<<"Loading Corresponances from file:  "<<corrs_path() <<vcl_endl;
  bwm_observer_mgr::load_3d_corrs(corrs_path(), corrs);
  vcl_cout<<"Done loading  "<<corrs.size() << " correspondances" <<vcl_endl;

  // assume correspondences between two sites only
  unsigned n = corrs.size();
  vnl_matrix<double> pts0(3,n), pts1(3,n);
  vcl_vector<vnl_vector_fixed<double,3> > pts0_v;
  vcl_vector<vnl_vector_fixed<double,3> > pts1_v;

  for (unsigned i = 0; i<n; ++i) {
    vcl_cout << *(corrs[i]);
    vcl_vector<vgl_point_3d<double> > match_pts = corrs[i]->matching_pts();
    pts0[0][i] = match_pts[0].x();  pts1[0][i] = match_pts[1].x();
    pts0[1][i] = match_pts[0].y();  pts1[1][i] = match_pts[1].y();
    pts0[2][i] = match_pts[0].z();  pts1[2][i] = match_pts[1].z();
    pts0_v.push_back(vnl_vector_fixed<double,3>(match_pts[0].x(), match_pts[0].y(), match_pts[0].z()));
    pts1_v.push_back(vnl_vector_fixed<double,3>(match_pts[1].x(), match_pts[1].y(), match_pts[1].z()));

  }
  vgl_rotation_3d<double> R;
  vnl_vector_fixed<double, 3> t;
  double scale;
  if (!compute_similarity(pts0, pts1, R, t, scale)) {
    vcl_cout << "similarity computation failed\n";
    return -1;
  }
  
  //save the transformation
  vcl_cout << "scale = " << scale << "\nR = " << R << "\nt = " << t << vcl_endl;
  
  vcl_string srt_tfile = tform_path() + ".txt";
  vcl_cout << "Saving file: " << srt_tfile <<vcl_endl;

  vcl_ofstream srt_ofs(srt_tfile.c_str());
  srt_ofs << scale << "\n" << R << "\n" << t << "\n";
  srt_ofs.close();
  
  vcl_string mat_tfile = tform_path() + "_matrix.txt";
  vcl_ofstream mat_ofs(mat_tfile.c_str());
  vnl_matrix_fixed<double, 4, 4 > S;
  S.set_identity();
  S.update(scale * vnl_matrix_fixed<double, 3, 3 >(S.extract(3,3)));
  vcl_cout << "Scale:\n" << S << "\n";
  vnl_matrix_fixed<double, 4, 4 > RT;
  RT.set_identity();
  RT.update(R.as_matrix());
  RT.set_column(3, t);
  vcl_cout << "RT:\n" << RT << "\n";
  vnl_matrix_fixed<double, 4, 4 > Tmat = S*RT;
  mat_ofs << Tmat;
  mat_ofs.close();

   
  srt_ofs.close();
  
  //transform the points
  if (vul_file::is_directory(input_path().c_str())){
    vcl_cout << "Looking for .ply files in directory: " << input_path() <<vcl_endl;
    vcl_string in_dir = input_path() + "/*.ply";
    for (vul_file_iterator fn = in_dir.c_str(); fn; ++fn) {
      vcl_string f = fn();
      vcl_vector<vnl_vector_fixed<double,3> > points2transform;
      vcl_vector<vnl_vector_fixed<double,3> > transformed_points;
      readPointsFromPLY(f, points2transform);
      for(unsigned pi = 0; pi < points2transform.size(); ++pi){
          vnl_vector_fixed<double, 3> new_p = scale*(R * (points2transform[pi])+ t);
        transformed_points.push_back(new_p);
      }
      vcl_cout << "Transformed Poins: " << points2transform.size() << "\n";
      vcl_string fname = vul_file::strip_directory(f.c_str());
      vcl_cout << fname << '\n';
      vcl_string out_dir = output_path() + "/";
      vcl_string out_file = out_dir + fname;
      writePointsToPLY(out_file, transformed_points);
    }
  }else if ((vul_file::exists(input_path().c_str())) && !(vul_file::is_directory(input_path().c_str()))){
    vcl_cout << "Reading file: " << input_path() <<vcl_endl;
    vcl_vector<vnl_vector_fixed<double,3> > points2transform;
    vcl_vector<vnl_vector_fixed<double,3> > transformed_points;
     
    readPointsFromPLY(input_path(), points2transform);
    for(unsigned pi = 0; pi < points2transform.size(); ++pi){
      vnl_vector_fixed<double, 3> new_p = scale*(R * (points2transform[pi])+ t);
      transformed_points.push_back(new_p);
    }
    vcl_cout << "Transformed Poins: " << points2transform.size() << "\n";
    writePointsToPLY(output_path(), transformed_points);
    
    if (pts0_path() != "") {
      writePointsToPLY(pts0_path(), pts0_v);
    }
    
    if (pts1_path() != "") {
      writePointsToPLY(pts1_path(), pts1_v);
    }
    
  }
  else
  {
    vcl_cout<<"Warning: Input path does not exist"<<vcl_endl;
    return 0;
  }
  
  return 0;
}
