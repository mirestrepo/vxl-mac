#include "boxm2_multi_cache.h"
//:
// \file
#include <vcl_sstream.h>
#include <vgl/vgl_distance.h>
#include <vgl/vgl_box_3d.h>
#include <vgl/vgl_intersection.h>
#include <vcl_algorithm.h>
#include <vsph/vsph_camera_bounds.h>

//: init opencl cache for each device
boxm2_multi_cache::boxm2_multi_cache(boxm2_scene_sptr              scene,
                                     vcl_vector<bocl_device*> &    devices)
{
  scene_ = scene;
  boxm2_lru_cache::create(scene);

#if  1
  int blocksAdded = 0; 
  //create a sub scene for each device
  for (int dev_id=0; dev_id<devices.size(); ++dev_id){
    //create scene
    boxm2_scene_sptr sub_scene = new boxm2_scene();
    sub_scene->set_local_origin(scene->local_origin());
    sub_scene->set_xml_path(scene->xml_path());
    sub_scene->set_data_path(scene->data_path());
    sub_scenes_.push_back(sub_scene);

    //create ocl_cache for this scene...
    boxm2_opencl_cache* ocl_cache = new boxm2_opencl_cache(sub_scene, devices[dev_id]);
    ocl_caches_.push_back(ocl_cache);
  }

  //get scene block bounds
  vgl_vector_3d<unsigned int> scene_dim = scene->scene_dimensions();
  vgl_point_3d<int> min_ids, max_ids;
  vgl_point_3d<double> min_origin, max_origin;
  scene->min_block_index(min_ids, min_origin);
  scene->max_block_index(max_ids, max_origin);
  vcl_cout<<"Min ids, max ids: "<<min_ids<<","<<max_ids<<vcl_endl;

  //check if scene dimensions will work
  if(scene_dim.y() % sub_scenes_.size() != 0) {
    vcl_cerr<<"  Boxm2_multi_cache::scene y dimension is not divisible by num devices\n"
            <<"  "<<scene_dim.y()<<" blocks by "<<sub_scenes_.size()<<" devices"<<vcl_endl;
    throw -1;
  }

  //divy up the blocks
  const int nDev = sub_scenes_.size(); 
  int dev_id=0; 
  for(int i=min_ids.x(); i<max_ids.x()+1; ++i) {
    //iterate over groups in Y
    for(int group=0; group<max_ids.y()+1; group+=nDev) {

      //create a block group
      boxm2_multi_cache_group grp;
      //add the vertical row of blocks to scene with dev_id
      dev_id = 0;
      for(int j=group; j<group+nDev; ++j){
        for(int k=min_ids.z(); k<max_ids.z()+1; ++k) {
          boxm2_block_id id(i,j,k);
          vcl_cout<<"Attempting to add block id: "<<id<<vcl_endl;
          if(scene->block_exists(id)) {
            boxm2_block_metadata md = scene->get_block_metadata(id);
            grp.add_block(md); 
            sub_scenes_[dev_id]->add_block_metadata(md); 
            blocksAdded++;
          }
        }
        dev_id++; 
      }
      groups_.push_back(grp);
      //dev_id = (dev_id+1) % sub_scenes_.size();
    }
  }

  for(int i=0; i<groups_.size(); ++i)
    vcl_cout<<"Group "<<i<<": "<<groups_[i]<<vcl_endl;;
#endif
#if 0
  //partition the scene into smaller (continuous) scenes
  vgl_point_3d<int> min_ids, max_ids;
  vgl_point_3d<double> min_origin, max_origin;
  scene->min_block_index(min_ids, min_origin);
  scene->max_block_index(max_ids, max_origin);

  //divide by half in each direction (take ceiling)
  vgl_point_3d<int> incr_ids( (int) vcl_ceil( (float)(max_ids.x()+1)/devices.size() ),
                              (int) vcl_ceil( (float)(max_ids.y()+1)/devices.size() ),
                              (int) vcl_ceil( (float)(max_ids.z()+1)/devices.size() ) );

  //for each device create a new scene
  int blocksAdded = 0;
  for (int dev_id=0; dev_id<devices.size(); ++dev_id)
  {
    vcl_map<boxm2_block_id, boxm2_block_metadata> sub_scene_blocks;

    //figure out which blocks to add to it...
    vcl_map<boxm2_block_id, boxm2_block_metadata>& blocks = scene->blocks();
    vcl_map<boxm2_block_id, boxm2_block_metadata>::iterator iter;
    for (iter=blocks.begin(); iter!=blocks.end(); ++iter)
    {
      boxm2_block_id id = iter->first;
      int lower = dev_id * incr_ids.x();
      int upper = (dev_id+1) * incr_ids.x();
      if (id.i() >= lower && id.i() < upper) {
        boxm2_block_metadata md = iter->second;
        sub_scene_blocks[id] = md;
        vcl_cout<<" added: "<<id<<" to dev "<<dev_id<<vcl_endl;
        blocksAdded++;
      }
    }

    //create scene
    boxm2_scene_sptr sub_scene = new boxm2_scene();
    sub_scene->set_local_origin(scene->local_origin());
    sub_scene->set_xml_path(scene->xml_path());
    sub_scene->set_data_path(scene->data_path());
    sub_scene->set_blocks(sub_scene_blocks);
    sub_scenes_.push_back(sub_scene);

    //create ocl_cache for this scene...
    boxm2_opencl_cache* ocl_cache = new boxm2_opencl_cache(sub_scene, devices[dev_id]);
    ocl_caches_.push_back(ocl_cache);
  }
#endif
  if ( blocksAdded != scene->blocks().size() ) {
    vcl_cout<<"boxm2_multi_cache blocks added not equal to number of blocks in original scene:\n"
            <<"  Num gpus: "<<devices.size()<< '\n'
            <<"  blocks added: "<<blocksAdded<<" != "<<scene->blocks().size()<<vcl_endl;
    throw -1;
  }
}

boxm2_multi_cache::~boxm2_multi_cache()
{
#if 0
  delete ocl caches
  for (int i=0; i<ocl_caches_.size(); ++i)
    if (ocl_caches_[i]) delete ocl_caches_[i];
#endif
}

vcl_vector<boxm2_opencl_cache*> boxm2_multi_cache::get_vis_sub_scenes(vpgl_perspective_camera<double>* cam)
{
  vcl_vector<boxm2_opencl_cache*> vis_order;
  if (!cam) {
    vcl_cout << "null camera in boxm2_scene::get_vis_blocks(.)\n";
    return vis_order;
  }
  //get camera center and order blocks distance from the cam center
  //for non-projective cameras there may not be a single center of projection
  //so instead get the ray origin farthest from the scene origin.
  vgl_point_3d<double> cam_center = cam->camera_center();

  //Map of distance, id
  return this->get_vis_order_from_pt(cam_center);
}

vcl_vector<boxm2_opencl_cache*> 
boxm2_multi_cache::get_vis_sub_scenes(vpgl_generic_camera<double>* cam){
  vgl_point_3d<double> cam_center = cam->max_ray_origin();
  return this->get_vis_order_from_pt(cam_center);
}

vcl_vector<boxm2_opencl_cache*> 
boxm2_multi_cache::get_vis_sub_scenes(vpgl_camera_double_sptr cam)
{
  if ( cam->type_name() == "vpgl_generic_camera" )
    return this->get_vis_sub_scenes( (vpgl_generic_camera<double>*) cam.ptr() );
  else if ( cam->type_name() == "vpgl_perspective_camera" )
    return this->get_vis_sub_scenes( (vpgl_perspective_camera<double>*) cam.ptr() );
  else
    vcl_cout<<"boxm2_scene::get_vis_blocks doesn't support camera type "<<cam->type_name()<<vcl_endl;
  
  //else return empty
  vcl_vector<boxm2_opencl_cache*> empty;
  return empty;
}

vcl_vector<boxm2_opencl_cache*> boxm2_multi_cache::get_vis_order_from_pt(vgl_point_3d<double> const& pt)
{
  //Map of distance, id
  vcl_vector<boxm2_opencl_cache*>   vis_order;
  vcl_vector<boxm2_dist_cache_pair> distances;

  //iterate through each block
  for (int idx=0; idx<ocl_caches_.size(); ++idx) {
    boxm2_opencl_cache*     cache   = ocl_caches_[idx];
    boxm2_scene_sptr        sscene  = cache->get_scene();
    vgl_box_3d<double>      bbox    = sscene->bounding_box();
    vgl_point_3d<double>    center  = bbox.centroid();
    double                  dist    = vgl_distance( center, pt );
    distances.push_back( boxm2_dist_cache_pair(dist, cache) );
  }

  //sort distances
  vcl_sort(distances.begin(), distances.end());

  //put blocks in "vis_order"
  vcl_vector<boxm2_dist_cache_pair>::iterator di;
  for (di = distances.begin(); di != distances.end(); ++di) {
    vis_order.push_back(di->cache_);
  }
  return vis_order;
}

//----------------------------------------------
// Cache block group visibility order functions
//----------------------------------------------
vcl_vector<boxm2_multi_cache_group> 
boxm2_multi_cache::get_vis_groups(vpgl_camera_double_sptr cam)
{
  vgl_point_3d<double> center;
  vgl_box_2d<double> camBox; 
  if ( cam->type_name() == "vpgl_generic_camera" ){
    vpgl_generic_camera<double>* gcam = (vpgl_generic_camera<double>*) cam.ptr();
    center = gcam->max_ray_origin(); 
  }
  else if ( cam->type_name() == "vpgl_perspective_camera" ){
    vpgl_perspective_camera<double>* pcam = (vpgl_perspective_camera<double>*) cam.ptr();
    center = pcam->camera_center(); 
    //find intersection box
    vgl_box_3d<double> sceneBB = scene_->bounding_box();
    vgl_box_2d<double> lowBox, highBox; 
    vsph_camera_bounds::planar_bounding_box(*pcam, lowBox, sceneBB.min_z()); 
    vsph_camera_bounds::planar_bounding_box(*pcam, highBox, sceneBB.max_z());
    camBox.add(lowBox); 
    camBox.add(highBox);
  }
  else {
    vcl_cout<<"boxm2_scene::get_vis_blocks doesn't support camera type "<<cam->type_name()<<vcl_endl;
    return vcl_vector<boxm2_multi_cache_group>();
  }
  return this->group_order_from_pt(center, camBox);
}

vcl_vector<boxm2_multi_cache_group> 
boxm2_multi_cache::group_order_from_pt(vgl_point_3d<double> const& pt,
                                       vgl_box_2d<double> const& camBox)
{
  //Map of distance, id
  vcl_vector<boxm2_dist_group_pair> distances;

  //iterate through each block
  for (int i=0; i<groups_.size(); ++i) {
    boxm2_multi_cache_group& grp = groups_[i];

    //check if cam bbox intersects
    vgl_box_2d<double> grp2d(grp.bbox_.min_x(), grp.bbox_.max_x(),
                             grp.bbox_.min_y(), grp.bbox_.max_y());
    vgl_box_2d<double> intersect = vgl_intersection(grp2d, camBox);
    if(!intersect.is_empty() || camBox.is_empty()) {
      vgl_point_3d<double> center  = grp.bbox_.centroid();
      double dist = vgl_distance( center, pt );
      distances.push_back( boxm2_dist_group_pair(dist, grp) );
    }
  }

  //sort distances
  vcl_sort(distances.begin(), distances.end());

  //put blocks in "vis_order"
  vcl_vector<boxm2_multi_cache_group>   vis_order;
  vcl_vector<boxm2_dist_group_pair>::iterator di;
  for (di = distances.begin(); di != distances.end(); ++di)
    vis_order.push_back(di->grp_);
  return vis_order;
}

vcl_string boxm2_multi_cache::to_string()
{
  vcl_stringstream s;
  s <<"******* boxm2_multi_cache ************************************\n"
    <<" num sub scenes: "<<sub_scenes_.size()<<'\n';
  for (int i=0; i<sub_scenes_.size(); ++i) {
    s << "Sub Scene "<<i<<":\n"
      <<(*sub_scenes_[i])<<vcl_endl;
  }
  return s.str();
}

void boxm2_multi_cache::clear()
{
  for(int i=0; i<ocl_caches_.size(); ++i)
    ocl_caches_[i]->clear_cache();
}


//----------------------- stream io----------------------------------------//
//: shows elements in cache
vcl_ostream& operator<<(vcl_ostream &s, boxm2_multi_cache& cache)
{
  s << cache.to_string();
  return s;
}


vcl_ostream& operator<<(vcl_ostream &s, boxm2_multi_cache_group& grp)
{
  s << "boxm2_multi_cache_group [" ;
  for(int i=0; i<grp.ids_.size(); ++i) 
    s << grp.ids_[i] << " "; 
  s << "]";
  return s; 
}



