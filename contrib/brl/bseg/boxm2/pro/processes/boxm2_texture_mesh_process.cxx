// This is brl/bseg/boxm2/pro/processes/boxm2_texture_mesh_process.cxx
#include <bprb/bprb_func_process.h>
//:
// \file
// \brief  A process for exporting a texture mapped mesh of a scene
//
// \author Vishal Jain
// \date Mar 15, 2011

#include <vcl_fstream.h>
#include <vul/vul_file.h>
#include <vul/vul_timer.h>
#include <vnl/vnl_random.h>
#include <boxm2/boxm2_scene.h>
#include <boxm2/boxm2_util.h>

//vil includes
#include <vil/vil_image_view.h>
#include <vil/vil_save.h>
#include <vil/vil_new.h>
#include <vil/vil_math.h>

//vgl
#include <vgl/vgl_distance.h>
#include <vgl/vgl_polygon.h>
#include <vgl/vgl_polygon_scan_iterator.h>
#include <vgl/vgl_triangle_3d.h>
#include <bvgl/bvgl_triangle_interpolation_iterator.h>

//vpgl camera stuff
#include <vpgl/vpgl_perspective_camera.h>
#include <vpgl/algo/vpgl_project.h>

//det and imesh includes
#include <sdet/sdet_image_mesh.h>
#include <sdet/sdet_image_mesh_params.h>
#include <imesh/imesh_fileio.h>
#include <imesh/algo/imesh_render.h>

namespace boxm2_texture_mesh_process_globals
{
  const unsigned n_inputs_ = 4;
  const unsigned n_outputs_ = 0;

  //struct for passing 3d triangles (couldn't find a 3d triangle in VGL...)
  struct triangle_3d {
    vgl_point_3d<double> points[3];
    unsigned face_id;
  };

  //helper texture map methods
  void boxm2_texture_mesh_from_imgs(vcl_string im_dir,
                                    vcl_string cam_dir,
                                    imesh_mesh& in_mesh,
                                    vcl_map<vcl_string, imesh_mesh>& meshes);

  //populates a vector of visibility images (by face id int)
  void boxm2_visible_faces( vcl_vector<vpgl_perspective_camera<double>* >& cameras,
                            vcl_vector<vil_image_view<int>* >& vis_images,
                            imesh_mesh& in_mesh);

  //checks if a triangle in UV space is visible.  us and vs are double buffers of length 3
  bool face_is_visible( vpgl_perspective_camera<double>* cam,
                        vil_image_view<int>* vis_img,
                        triangle_3d& world_tri);

  //matches textures
  void boxm2_match_textures(vcl_vector<vil_image_view<int>* >& vis_images,
                            vcl_vector<vpgl_perspective_camera<double>* >& cameras,
                            vcl_vector<vcl_string>& imfiles,
                            imesh_mesh& in_mesh,
                            vcl_map<vcl_string, vcl_vector<unsigned> >& app_faces,
                            vcl_map<vcl_string, vpgl_perspective_camera<double>* >& texture_cams);

  //returns a list of visible triangles given a camera,
  //visibility image, and world coordinate 3d triangle
  vcl_vector<triangle_3d> get_visible_triangles(vpgl_perspective_camera<double>* cam,
                                                vil_image_view<int>* vis_img,
                                                triangle_3d& world_tri);
}

bool boxm2_texture_mesh_process_cons(bprb_func_process& pro)
{
  using namespace boxm2_texture_mesh_process_globals;

  //process takes 2 inputs
  int i=0;
  vcl_vector<vcl_string> input_types_(n_inputs_);
  input_types_[i++] = "imesh_mesh_sptr";  //depth image
  input_types_[i++] = "vcl_string";                //directory of scene images
  input_types_[i++] = "vcl_string";                //directory of corresponding cams
  input_types_[i++] = "vcl_string";                //output dir of saved mesh

  // process has 1 output
  vcl_vector<vcl_string>  output_types_(n_outputs_);
  //output_types_[0] = "boxm2_scene_sptr";

  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}

bool boxm2_texture_mesh_process(bprb_func_process& pro)
{
  using namespace boxm2_texture_mesh_process_globals;
  if ( pro.n_inputs() < n_inputs_ ){
    vcl_cout << pro.name() << ": The input number should be " << n_inputs_<< vcl_endl;
    return false;
  }
  unsigned argIdx = 0;
  imesh_mesh_sptr mesh = pro.get_input<imesh_mesh_sptr>(argIdx++);
  vcl_string img_dir   = pro.get_input<vcl_string>(argIdx++);
  vcl_string cam_dir   = pro.get_input<vcl_string>(argIdx++);
  vcl_string out_dir   = pro.get_input<vcl_string>(argIdx++);

  //create the mesh directory
  if (out_dir != "") {
    if (!vul_file::make_directory_path(out_dir.c_str())) {
      vcl_cout<<"Couldn't make directory path "<<out_dir<<vcl_endl;
      return false;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //Texture map the mesh
  //////////////////////////////////////////////////////////////////////////////
  vul_timer t;
  vcl_map<vcl_string, imesh_mesh> meshes;
  boxm2_texture_mesh_from_imgs(img_dir, cam_dir, *mesh, meshes);

  ////////////////////////////////////////////////////////////////////////////////
  //// Write out in VRML format
  ////////////////////////////////////////////////////////////////////////////////
  //output file and stream
  vcl_string vrfile = out_dir + "/vrmesh.wrl";
  vcl_ofstream os(vrfile.c_str());

  //write each submesh into one file
  vcl_map<vcl_string, imesh_mesh>::iterator subMesh;
  for (subMesh = meshes.begin(); subMesh != meshes.end(); ++subMesh)
  {
    imesh_mesh& sMesh = subMesh->second;
    vcl_cout<<"Writing sub mesh: "<<sMesh.tex_source()<<" has "<<sMesh.num_faces()<<" faces"<<vcl_endl;
    imesh_write_vrml(os, sMesh);
  }
  os.close();
  vcl_cout<<"Texture Mapping Mesh Time: "<<t.all()<<"ms"<<vcl_endl;

  return true;
}


//Given a directory of images, dir cams, an input mesh, this function creates
//map of textured meshes (imesh doesn't ostensibly handle meshes from multiple textures)
void boxm2_texture_mesh_process_globals::boxm2_texture_mesh_from_imgs(vcl_string im_dir,
                                                                      vcl_string cam_dir,
                                                                      imesh_mesh& in_mesh,
                                                                      vcl_map<vcl_string, imesh_mesh>& meshes)
{
  ////////////////////////////////////////////////////////////////////////////////
  // BEGIN TEXTURE MAPPING
  // Gather cameras and iamges that will contribute to the texture
  ////////////////////////////////////////////////////////////////////////////////
  vcl_vector<vcl_string> allims  = boxm2_util::images_from_directory(im_dir);
  vcl_vector<vpgl_perspective_camera<double>* > allcams = boxm2_util::cameras_from_directory(cam_dir);
  if (allims.size() != allcams.size()) {
    vcl_cout<<"Texture images are not 1 to 1 with cameras:: dirs "<<im_dir<<" and "<<cam_dir<<vcl_endl;
    return;
  }

  //choose a few random images
  vcl_vector<vcl_string> imfiles;
  vcl_vector<vpgl_perspective_camera<double>* > cameras;

  int handpicked[] = { 0, 1, 40, 82, 96, 105, 133, 153};
  //int handpicked[] = { 0,133 };
  for (unsigned int i=0; i<sizeof(handpicked)/sizeof(int); ++i) {
    imfiles.push_back(allims[handpicked[i]]);
    cameras.push_back(allcams[handpicked[i]]);
    vcl_cout<<"added image: "<<imfiles[i]<<vcl_endl;
  }
#if 0
  vnl_random rand(9667566);
  for (int i=0; i<5; ++i) {
    unsigned filenum = rand.lrand32(1, allims.size()-1);
    imfiles.push_back(allims[filenum]);
    cameras.push_back(allcams[filenum]);
  }
#endif

  ////////////////////////////////////////////////////////////////////////////////
  // make sure mesh has computed vertex normals
  ////////////////////////////////////////////////////////////////////////////////
  in_mesh.compute_vertex_normals_from_faces();

  ////////////////////////////////////////////////////////////////////////////////
  // Render Visibility Images
  ////////////////////////////////////////////////////////////////////////////////
  vcl_cout<<"calculating visibility images (for each textured image)"<<vcl_endl;
  vcl_vector<vil_image_view<int>* > vis_images;
  boxm2_visible_faces(cameras, vis_images, in_mesh);

  ////////////////////////////////////////////////////////////////////////////////
  // For each Face:
  //   determine which image is closest in normal and visible, store face index in a map[image_str, vector<int>face]
  // match each face to best image
  ////////////////////////////////////////////////////////////////////////////////
  vcl_cout<<"Populating faces for each texture"<<vcl_endl;
  vcl_map<vcl_string, vcl_vector<unsigned> > app_faces; //image_name to face_list
  vcl_map<vcl_string, vpgl_perspective_camera<double>* > texture_cams;
  boxm2_match_textures(vis_images, cameras, imfiles, in_mesh, app_faces, texture_cams);

  ////////////////////////////////////////////////////////////////////////////////
  // For each image/appearance:
  //   - create a vert list
  //   - create a face list
  //   - create a sub mesh that is textured
  ////////////////////////////////////////////////////////////////////////////////
  imesh_regular_face_array<3>& in_faces = (imesh_regular_face_array<3>&) in_mesh.faces();
  imesh_vertex_array<3>& in_verts = in_mesh.vertices<3>();
  //for each appearance (texture image), create an imesh_mesh (subMesh);
  vcl_cout<<"Creating Sub Meshes for each texture"<<vcl_endl;
  vcl_map<vcl_string, vcl_vector<unsigned> >::iterator apps;
  for (apps = app_faces.begin(); apps != app_faces.end(); ++apps)
  {
    //for each appearance, we're creating a whole new mesh
    // first create the face list
    imesh_regular_face_array<3>* flist = new imesh_regular_face_array<3>();

    //now create the vertex list
    imesh_vertex_array<3>* verts3 = new imesh_vertex_array<3>();

    //get faces list corresponding to this texture
    vcl_vector<unsigned>& face_list = apps->second;
    for (unsigned int i=0; i<face_list.size(); ++i) {
      //get old face
      unsigned old_fIdx = face_list[i];

      //old face vertices
      unsigned v1 = in_faces[old_fIdx][0];
      unsigned v2 = in_faces[old_fIdx][1];
      unsigned v3 = in_faces[old_fIdx][2];

      //push these vertices onto the new vert list
      verts3->push_back( imesh_vertex<3>(in_verts[v1][0], in_verts[v1][1], in_verts[v1][2]) );
      verts3->push_back( imesh_vertex<3>(in_verts[v2][0], in_verts[v2][1], in_verts[v2][2]) );
      verts3->push_back( imesh_vertex<3>(in_verts[v3][0], in_verts[v3][1], in_verts[v3][2]) );

      imesh_tri tri(3*i, 3*i+1, 3*i+2);
      flist->push_back(tri);
    }

    //create the submesh using the auto ptrs
    vcl_auto_ptr<imesh_vertex_array_base> v3(verts3);
    vcl_auto_ptr<imesh_face_array_base> f3(flist);
    imesh_mesh subMesh(v3, f3);

    vcl_cout<<"Setting tex source: "<<apps->first<<vcl_endl;
    meshes[apps->first] = subMesh;
    meshes[apps->first].set_tex_source(apps->first);
  }

  //////////////////////////////////////////////////////////////////////////////
  //For each mesh, map each vertex
  //////////////////////////////////////////////////////////////////////////////
  vcl_cout<<"Mapping sub meshes for each texture"<<vcl_endl;
  vcl_map<vcl_string, imesh_mesh>::iterator subMesh;
  vcl_map<vcl_string, vpgl_perspective_camera<double>* >::iterator txCam = texture_cams.begin();
  for (subMesh = meshes.begin(); subMesh != meshes.end(); ++subMesh, ++txCam)
  {
    imesh_mesh& mesh = subMesh->second;
    imesh_vertex_array<3>& verts = mesh.vertices<3>();
    unsigned nverts = mesh.num_verts();

    //texture map the non empty appearances
    if (txCam->first != "empty")
    {
      vcl_vector<vgl_point_2d<double> > tex_coords(nverts, vgl_point_2d<double>(0.0,0.0));
      for (unsigned iv = 0; iv<nverts; ++iv)
      {
        //find camera corresponding to this texture
        vpgl_perspective_camera<double>* closest = txCam->second;
        vgl_point_2d<double> principal_point = closest->get_calibration().principal_point();
        double ni = principal_point.x()*2.0;
        double nj = principal_point.y()*2.0;

        //project the vertex onto the camera, store the texture coordinate
        double x = verts[iv][0];
        double y = verts[iv][1];
        double z = verts[iv][2];
        double u,v;
        closest->project(x, y, z, u, v);

        //flip v about the y axis
        v=nj-v;

        //store the tex_coordinate
        vgl_point_2d<double> uv(u/ni,v/nj);
        tex_coords[iv] = uv;
      }
      mesh.set_tex_coords(tex_coords);
    }
  }
}


//image_name to face_list
void boxm2_texture_mesh_process_globals::boxm2_match_textures(vcl_vector<vil_image_view<int>* >& vis_images,
                                                              vcl_vector<vpgl_perspective_camera<double>* >& cameras,
                                                              vcl_vector<vcl_string>& imfiles,
                                                              imesh_mesh& in_mesh,
                                                              vcl_map<vcl_string, vcl_vector<unsigned> >& app_faces,
                                                              vcl_map<vcl_string, vpgl_perspective_camera<double>* >& texture_cams)
{
  //grab faces and vertices from the mesh
  imesh_regular_face_array<3>& in_faces = (imesh_regular_face_array<3>&) in_mesh.faces();
  unsigned nfaces = in_mesh.num_faces();
  imesh_vertex_array<3>& in_verts = in_mesh.vertices<3>();

  //for each face, determine which view is best
  for (unsigned iface = 0; iface<nfaces; ++iface)
  {
    //make a triangle_3d out of this face
    triangle_3d world_tri;
    world_tri.face_id = iface;
    for (int i=0; i<3; ++i) {
      unsigned vertexId = in_faces[iface][i];
      double x = in_verts[vertexId][0];
      double y = in_verts[vertexId][1];
      double z = in_verts[vertexId][2];
      world_tri.points[i] = vgl_point_3d<double>(x,y,z);
    }

    //create list of cameras from which you can see this face
    vcl_vector<vpgl_perspective_camera<double>* > visible_views;
    for (unsigned int i=0; i<vis_images.size(); ++i) {
      if ( face_is_visible( cameras[i], vis_images[i], world_tri) )
        visible_views.push_back(cameras[i]);
    }

    //now compare the normal to each of the visible view cameras
    vgl_vector_3d<double>& normal = in_faces.normal(iface);

    //find camera with the closest look vector to this normal
    int closeIdx = boxm2_util::find_nearest_cam(normal, visible_views);
    vpgl_perspective_camera<double>* closest = NULL;
    vcl_string im_name = "empty";
    if (closeIdx >= 0) {
      closest = cameras[closeIdx];
      im_name = imfiles[closeIdx];
    }

    //grab appropriate face list (create it if it's not there)
    vcl_map<vcl_string, vcl_vector<unsigned> >::iterator iter = app_faces.find(im_name);
    if ( iter == app_faces.end() ) {
      vcl_cout<<"boxm2_match_textures:: Adding image "<<im_name<<" to texture list"<<vcl_endl;
      vcl_vector<unsigned> faceList;
      faceList.push_back(iface);
      app_faces[im_name] = faceList;

      //keep track of the camera
      texture_cams[im_name] = closest;
    }
    else {
      app_faces[im_name].push_back(iface);
    }
  }

#if 0
  ////////////////////////////////////////////////////////////////////////////////
  // Now there is a set of faces without textures,
  // for each face
  //    for each image
  //      - find visible portions of these faces, record area and angle from dist
  //    find angle < 60 degrees AND largest area patch
  //    cut the face such that the patch is exposed (patch may be some arbitrary polygon)
  //    triangulate the n-gon into n triangles
  //    add the *new* points to the mesh, as well as the new faces
  //    remember to not leave the old face in the mesh...
  ////////////////////////////////////////////////////////////////////////////////
  for (unsigned iface = 0; iface<nfaces; ++iface)
  {
    //whole first chunk figures out which set of triangles this face should be broken into
    double max_area = 0.0;
    double max_img = -1;
    vcl_vector<triangle_3d> final_triangles;
    for (int imIdx=0; imIdx<vis_images.size(); ++imIdx) {
      //if this angle is too large, just pass over it
      double midAngle = angle(in_faces.normal(iface), -1*cameras[imIdx]->principal_axis()); // return acos(cos_angle(a,b));
      if (midAngle > vnl_math::pi/3) continue;

      //find visible portion of iface on this image, record area (make a 3d tri out of it first)
      triangle_3d world_tri;
      world_tri.face_id = iface;
      for (int i=0; i<3; ++i) {
        unsigned vertexId = in_faces[iface][i];
        double x = in_verts[vertexId][0];
        double y = in_verts[vertexId][1];
        double z = in_verts[vertexId][2];
        world_tri.points[i] = vgl_point_3d<double>(x,y,z);
      }
      vcl_vector<triangle_3d> vis_tris = get_visible_triangles(cameras[imIdx], vis_images[imIdx], world_tri);

      //get the total area of the triangles
      double totalArea = 0;
      for (int tri_i = 0; tri_i < vis_tris.size(); ++tri_i) {
        totalArea+= vgl_triangle_3d_area(vis_tris[tri_i].points[0],
                                         vis_tris[tri_i].points[1],
                                         vis_tris[tri_i].points[2]);
      }

      //store it if it's the biggest patch so far
      if (totalArea > max_area) {
        max_area = totalArea;
        max_img = imIdx;
        final_triangles = vis_tris;
      }
    }

    //now that you have the visible triangles that will be mapped in your mesh...
    // 1. add the new vertices
    // 2. add the new faces

    // 3. add the new faces to the correct texture list
    vcl_string im_name =
    app_faces[im_name].push_back(iface);
  }

  //Be wary of this interpolation iterator - didn't match up with the imesh one
  bvgl_triangle_interpolation_iterator(double *verts_x, double *verts_y, T *values, unsigned int v0 = 0, unsigned int v1 = 1, unsigned int v2 = 2);

  // Create a new face list with the faces split by visibility
  imesh_regular_face_array<3>* newFaces = new imesh_regular_face_array<3>();
#endif
}


//returns a list of visible triangles given a camera,
//visibility image, and world coordinate 3d triangle
vcl_vector<boxm2_texture_mesh_process_globals::triangle_3d>
boxm2_texture_mesh_process_globals::get_visible_triangles(vpgl_perspective_camera<double>* cam,
                                                          vil_image_view<int>* vis_img,
                                                          triangle_3d& world_tri)
{
  vcl_vector<triangle_3d> triangles;
  vcl_cerr << "TODO: boxm2_texture_mesh_process_globals::get_visible_triangles is not yet implemented\n";
  return triangles;
}


//given a camera, and an image with ID faces, this method returns true if the entire
//triangle (id face_id) is unoccluded from this point of view
bool boxm2_texture_mesh_process_globals::face_is_visible( vpgl_perspective_camera<double>* cam,
                                                          vil_image_view<int>* vis_img,
                                                          triangle_3d& world_tri)
{
  //project triangle
  double us[3], vs[3];
  for (int vIdx=0; vIdx<3; ++vIdx) {
    //project these verts into UV
    double x = world_tri.points[vIdx].x();
    double y = world_tri.points[vIdx].y();
    double z = world_tri.points[vIdx].z();
    double u,v;
    cam->project(x, y, z, u, v);
    us[vIdx] = u;
    vs[vIdx] = v;
  }

  //now create a polygon, and find the integer image coordinates (U,V) that this polygon covers
  int ni = vis_img->ni();
  int nj = vis_img->nj();
  unsigned int numPixels = 0;
  unsigned int numMatches = 0;

  vgl_triangle_scan_iterator<double> tsi;
  tsi.a.x = us[0];  tsi.a.y = vs[0];
  tsi.b.x = us[1];  tsi.b.y = vs[1];
  tsi.c.x = us[2];  tsi.c.y = vs[2];
  for (tsi.reset(); tsi.next(); ) {
    int y = tsi.scany();
    if (y<0 || y>=nj) continue;
    int min_x = tsi.startx();
    int max_x = tsi.endx();
    if (min_x >= ni || max_x < 0)
      continue;
    if (min_x < 0) min_x = 0;
    if (max_x >= ni) max_x = ni-1;
    for (int x = min_x; x <= max_x; ++x) {
      ++numPixels;
      if ( (*vis_img)(x,y) == world_tri.face_id )
        ++numMatches;
    }
  }

  // if the majority (90%) match, it's visible:
  return  numMatches * 10 > numPixels * 9;
}

//Constructs vector of visibility images - images that identify which triangle
//is visible at which pixel
// function is more or less complete - not much more to it.
void boxm2_texture_mesh_process_globals::boxm2_visible_faces( vcl_vector<vpgl_perspective_camera<double>* >& cameras,
                                                              vcl_vector<vil_image_view<int >* >& vis_images,
                                                              imesh_mesh& in_mesh)
{
  imesh_regular_face_array<3>& in_faces = (imesh_regular_face_array<3>&) in_mesh.faces();
  unsigned nfaces = in_mesh.num_faces();
  imesh_vertex_array<3>& in_verts = in_mesh.vertices<3>();

  //iterate over each camera, creating a visibility image for each
  for (unsigned int i=0; i<cameras.size(); ++i)
  {
    //get the principal point of the cam for image size
    vpgl_perspective_camera<double>* pcam = cameras[i];
    vgl_point_2d<double> principal_point = pcam->get_calibration().principal_point();
    unsigned ni = (unsigned) (principal_point.x()*2.0);
    unsigned nj = (unsigned) (principal_point.y()*2.0);

    // render the face_id/distance image
    vil_image_view<double> depth_im(ni, nj);
    vil_image_view<int>*   face_im     = new vil_image_view<int>(ni, nj);
    depth_im.fill(10e100);  //Initial depth is huge,
    face_im->fill(-1); //initial face id is -1
    for (unsigned iface = 0; iface<nfaces; ++iface)
    {
      //get the vertices from the face, project into UVs
      double us[3], vs[3], dists[3];
      for (int vIdx=0; vIdx<3; ++vIdx) {
        unsigned vertIdx = in_faces[iface][vIdx];

        //project these verts into UV
        double x = in_verts[vertIdx][0], y = in_verts[vertIdx][1], z = in_verts[vertIdx][2];
        double u,v;
        pcam->project(x, y, z, u, v);
        us[vIdx] = u;
        vs[vIdx] = v;

        //keep track of distance to each vertex
        dists[vIdx] = vgl_distance(vgl_point_3d<double>(x,y,z), pcam->get_camera_center());
      }

      //render the triangle label onto the label image, using the depth image
      vgl_point_3d<double> v1(us[0], vs[0], dists[0]);
      vgl_point_3d<double> v2(us[1], vs[1], dists[1]);
      vgl_point_3d<double> v3(us[2], vs[2], dists[2]);
      imesh_render_triangle_label<int>(v1, v2, v3, (int) iface, (*face_im), depth_im);
    } //end for iface

    //keep the vis_image just calculated
    vis_images.push_back(face_im);
    vil_save(depth_im, "/media/VXL/mesh/downtown/dist_im.tif");
  }
}

