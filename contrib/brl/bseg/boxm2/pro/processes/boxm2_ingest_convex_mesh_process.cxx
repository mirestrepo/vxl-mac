// This is brl/bseg/boxm2/pro/processes/boxm2_ingest_convex_mesh_process.cxx
#include <bprb/bprb_func_process.h>
//:
// \file
// \brief  A process for exporting a texture mapped mesh of a scene
//
// \author Vishal Jain
// \date Aug 22, 2012

#include <vcl_fstream.h>
#include <vul/vul_file.h>
#include <boxm2/boxm2_scene.h>
#include <boxm2/boxm2_util.h>
#include <boct/boct_bit_tree.h>

//vil includes
#include <vil/vil_image_view.h>
#include <vil/vil_new.h>
#include <vil/vil_math.h>

//vgl
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_vector_3d.h>
#include <bvgl/bvgl_triangle_interpolation_iterator.h>

//vpgl camera stuff
#include <vpgl/vpgl_perspective_camera.h>
#include <imesh/imesh_fileio.h>
#include <imesh/imesh_operations.h>
#include <imesh/algo/imesh_intersect.h>
#include <imesh/imesh_face.h>

#include <bmsh3d/bmsh3d_mesh.h>
#include <bmsh3d/algo/bmsh3d_fileio.h>
#include <bmsh3d/bmsh3d_mesh_mc.h>
#include <vgl/vgl_intersection.h>
#include <expatpplib.h>
#include <boxm2/boxm2_bounding_box_parser.h>

namespace boxm2_ingest_convex_mesh_process_globals
{
  const unsigned n_inputs_ = 5;
  const unsigned n_outputs_ = 0;

  typedef vnl_vector_fixed<unsigned char, 16> uchar16;
}

bool boxm2_ingest_convex_mesh_process_cons(bprb_func_process& pro)
{
  using namespace boxm2_ingest_convex_mesh_process_globals;

  //process takes 5 inputs
  int i=0;
  vcl_vector<vcl_string> input_types_(n_inputs_);
  input_types_[i++] = "boxm2_scene_sptr";  // scene
  input_types_[i++] = "boxm2_cache_sptr";  // scene
  input_types_[i++] = "vcl_string";        // input ply file
  input_types_[i++] = "int";               // label id
  input_types_[i++] = "vcl_string";        // Name of the category

  // process has no outputs
  vcl_vector<vcl_string>  output_types_(n_outputs_);

  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}

bool boxm2_ingest_convex_mesh_process(bprb_func_process& pro)
{
  using namespace boxm2_ingest_convex_mesh_process_globals;
  if ( pro.n_inputs() < n_inputs_ ) {
    vcl_cout << pro.name() << ": The number of inputs should be " << n_inputs_<< vcl_endl;
    return false;
  }

  //get the inputs
  boxm2_scene_sptr scene = pro.get_input<boxm2_scene_sptr>(0);
  boxm2_cache_sptr cache = pro.get_input<boxm2_cache_sptr>(1);
  vcl_string ply_file = pro.get_input<vcl_string>(2);
  int label_id  = pro.get_input<int>(3);
  vcl_string identifier = pro.get_input<vcl_string>(4);

  vcl_vector<bmsh3d_mesh_mc *> meshes;
  vcl_vector<int> label_ids;
  if (vul_file::extension(ply_file.c_str())==".xml")
  {
    boxm2_bounding_box_parser parser;
    vcl_FILE* xmlFile = vcl_fopen(ply_file.c_str(), "r");
    parser.parseFile(xmlFile);

    vcl_map<int, vcl_map<int, vcl_vector< vgl_point_3d<double> > > >::iterator entity_iter= parser.verts_.begin();
    for (; entity_iter!= parser.verts_.end(); entity_iter++)
    {
      vcl_map<int, vcl_vector< vgl_point_3d<double> > >::iterator volume_iter = entity_iter->second.begin();
      for (; volume_iter != entity_iter->second.end(); volume_iter++)
      {
        bmsh3d_mesh_mc *  bmesh = new bmsh3d_mesh_mc();
        for ( unsigned j = 0 ; j <volume_iter->second.size(); j++)
        {
          bmsh3d_vertex* v = bmesh->_new_vertex();

          v->set_pt(vgl_point_3d<double>(volume_iter->second[j].x(),
                                         volume_iter->second[j].y(),
                                         -100.0));
          bmesh->_add_vertex(v);
        }
        for (unsigned j = 0 ; j <volume_iter->second.size(); ++j)
        {
          bmsh3d_vertex* v = bmesh->_new_vertex();
          //vgl_point_3d<double> newpt(volume_iter->second[j].x(),
          //              volume_iter->second[j].y(),
          //              volume_iter->second[j].z()+parser.heights_[entity_iter->first][volume_iter->first]);
          //v->set_pt(newpt);
          v->set_pt(vgl_point_3d<double>(volume_iter->second[j].x(),
                                         volume_iter->second[j].y(),
                                         600.0));
          bmesh->_add_vertex(v);
        }
        // 01,2,3
        bmsh3d_face* F0 = bmesh->_new_face();
        bmsh3d_edge* e01 = bmesh->add_new_edge(bmesh->vertexmap(0), bmesh->vertexmap(1));
        bmsh3d_edge* e12 = bmesh->add_new_edge(bmesh->vertexmap(1), bmesh->vertexmap(2));
        bmsh3d_edge* e23 = bmesh->add_new_edge(bmesh->vertexmap(2), bmesh->vertexmap(3));
        bmsh3d_edge* e30 = bmesh->add_new_edge(bmesh->vertexmap(3), bmesh->vertexmap(0));

        _connect_F_E_end(F0,e01);
        _connect_F_E_end(F0,e12);
        _connect_F_E_end(F0,e23);
        _connect_F_E_end(F0,e30);

        bmesh->_add_face(F0);
        // 4,5,6,7
        bmsh3d_face* F1 = bmesh->_new_face();
        bmsh3d_edge* e45 = bmesh->add_new_edge(bmesh->vertexmap(4), bmesh->vertexmap(5));
        bmsh3d_edge* e56 = bmesh->add_new_edge(bmesh->vertexmap(5), bmesh->vertexmap(6));
        bmsh3d_edge* e67 = bmesh->add_new_edge(bmesh->vertexmap(6), bmesh->vertexmap(7));
        bmsh3d_edge* e74 = bmesh->add_new_edge(bmesh->vertexmap(7), bmesh->vertexmap(4));

        _connect_F_E_end(F1,e45);
        _connect_F_E_end(F1,e56);
        _connect_F_E_end(F1,e67);
        _connect_F_E_end(F1,e74);
        bmesh->_add_face(F1);
        // 0,4,7,3
        bmsh3d_face* F2 = bmesh->_new_face();
        bmsh3d_edge *e04 = bmesh->add_new_edge(bmesh->vertexmap(0), bmesh->vertexmap(4));
        bmsh3d_edge *e73 = bmesh->add_new_edge(bmesh->vertexmap(7), bmesh->vertexmap(3));
        _connect_F_E_end(F2,e04);
        _connect_F_E_end(F2,e74);
        _connect_F_E_end(F2,e73);
        _connect_F_E_end(F2,e30);
        bmesh->_add_face(F2);

        // 2,6,7,3
        bmsh3d_face* F3 = bmesh->_new_face();
        bmsh3d_edge *e26 = bmesh->add_new_edge(bmesh->vertexmap(2), bmesh->vertexmap(6));
        _connect_F_E_end(F3,e26);
        _connect_F_E_end(F3,e67);
        _connect_F_E_end(F3,e73);
        _connect_F_E_end(F3,e23);
        bmesh->_add_face(F3);

        // 1,5,6,2
        bmsh3d_face* F4 = bmesh->_new_face();
        bmsh3d_edge *e15 = bmesh->add_new_edge(bmesh->vertexmap(1), bmesh->vertexmap(5));
        _connect_F_E_end(F4,e15);
        _connect_F_E_end(F4,e56);
        _connect_F_E_end(F4,e26);
        _connect_F_E_end(F4,e12);
        bmesh->_add_face(F4);

        //0,1,5,4
        bmsh3d_face* F5 = bmesh->_new_face();
        _connect_F_E_end(F5,e01);
        _connect_F_E_end(F5,e15);
        _connect_F_E_end(F5,e45);
        _connect_F_E_end(F5,e04);
        bmesh->_add_face(F5);

        bmesh->build_IFS_mesh();
        meshes.push_back(bmesh);
        label_ids.push_back(entity_iter->first);
        // add vertices
      }
    }
  }
  else if (vul_file::extension(ply_file.c_str())==".ply")
  {
    bmsh3d_mesh_mc *  bmesh = new bmsh3d_mesh_mc();
    bmsh3d_load_ply(bmesh,ply_file.c_str());
    bmesh->IFS_to_MHE();
  }
  else
  {
    vcl_cout<<"Other extensions not supported yet";
    return false;
  }
  vcl_cout<<"Orienting face normals"<<vcl_endl;

  for (unsigned nm = 0 ; nm < meshes.size() ; nm++)
  {
    meshes[nm]->orient_face_normals();
  }
  vcl_map<boxm2_block_id, boxm2_block_metadata> blocks=scene->blocks();
  vcl_map<boxm2_block_id, boxm2_block_metadata>::iterator iter;
  for (iter = blocks.begin(); iter!= blocks.end(); iter ++)
  {
    boxm2_block_metadata mdata = iter->second;
    boxm2_block *     blk  = cache->get_block(iter->first);
    boxm2_data_base *  alpha  = cache->get_data_base(iter->first,boxm2_data_traits<BOXM2_ALPHA>::prefix(),0,true);
    int len_buffer  = alpha->buffer_length()/4*boxm2_data_info::datasize("boxm2_label_short") ;
    boxm2_data<BOXM2_ALPHA> alpha_data(alpha->data_buffer(),alpha->buffer_length(),iter->first);
    boxm2_data_base *  label_data_base = 0;         // dummy initialisations (see "flag" variable
    boxm2_data<BOXM2_LABEL_SHORT> * label_data = 0; // for guarantees on correct initialisation)
    boxm2_array_3d<uchar16>  trees = blk->trees();
    bool flag = false;
    for (unsigned nm = 0 ; nm < meshes.size() ; nm++)
    {
      label_id = label_ids[nm];
      vcl_cout<<"Blk id "<<iter->first<<vcl_endl;
      //meshes[nm]->print_topo_summary();
      vcl_map<int, bmsh3d_face* > fmap = meshes[nm]->facemap();
      vcl_map<int, bmsh3d_face* >::iterator face_it ;
      vgl_box_3d<double> bbox;
      for ( unsigned k = 0 ; k < meshes[nm]->num_vertices() ; k++)
      {
        bmsh3d_vertex * v  = meshes[nm]->vertexmap(k);
        bbox.add(v->get_pt());
      }
      vgl_box_3d<double> rbox = vgl_intersection<double>(mdata.bbox(),bbox);
      if (!rbox.is_empty())
      {
        if (!flag)
        {
          label_data_base = new boxm2_data_base(new char[len_buffer],len_buffer,iter->first, false) ;
          label_data = new boxm2_data<BOXM2_LABEL_SHORT>(label_data_base->data_buffer(),label_data_base->buffer_length(),iter->first);
          flag = true;
        }
        vgl_vector_3d<double> min_dir = rbox.min_point()-mdata.bbox().min_point();
        vgl_vector_3d<double> max_dir = rbox.max_point()-mdata.bbox().min_point();
        unsigned int min_i = (unsigned int) vcl_floor(min_dir.x()/mdata.sub_block_dim_.x());
        unsigned int min_j = (unsigned int) vcl_floor(min_dir.y()/mdata.sub_block_dim_.y());
        unsigned int min_k = (unsigned int) vcl_floor(min_dir.z()/mdata.sub_block_dim_.z());
        unsigned int max_i = (unsigned int) vcl_floor(max_dir.x()/mdata.sub_block_dim_.x());
        unsigned int max_j = (unsigned int ) vcl_floor(max_dir.y()/mdata.sub_block_dim_.y());
        unsigned int max_k = (unsigned int ) vcl_floor(max_dir.z()/mdata.sub_block_dim_.z());
        vcl_cout<<"Paint Cells in block "<<iter->first<<vcl_endl
                <<"mesh # "<<nm <<" Min : ("<<min_i<<','<<min_j<<','<<min_k<<")  Max: ("<<max_i<<','<<max_j<<','<<max_k<<')'<<vcl_endl;
        // read the trees info

        for (unsigned int i = min_i ; i < max_i; i++)
          for (unsigned int j = min_j ; j < max_j; j++)
            for (unsigned int k = min_k ; k < max_k; k++)
            {
              boct_bit_tree tree( trees(i,j,k).data_block() );
              vcl_vector<int> leaf_bits = tree.get_leaf_bits();
              for (unsigned t = 0 ; t < leaf_bits.size(); t++)
              {
                vgl_point_3d<double> cc = tree.cell_center(leaf_bits[t]);
                vgl_point_3d<double> global_cc(((double)i + cc.x())*mdata.sub_block_dim_.x() + mdata.bbox().min_x(),
                                               ((double)j + cc.y())*mdata.sub_block_dim_.y() + mdata.bbox().min_y(),
                                               ((double)k + cc.z())*mdata.sub_block_dim_.z() + mdata.bbox().min_z());
                bool inside = true;
                face_it = fmap.begin();
                while (face_it != fmap.end()) {
                  bmsh3d_face_mc* face = (bmsh3d_face_mc*) face_it->second;
                  vgl_point_3d<double> pt =  face->compute_center_pt();
                  vgl_vector_3d<double> normal = face->compute_normal();
                  double dotprod=dot_product<double>(pt-global_cc,normal);
                  if (dotprod<0)
                    inside = false;
                  face_it++;
                }
                if (inside)
                {
                  int dataindex = tree.get_data_index(t);
                  label_data->data()[dataindex] = label_id;
                  // float p =  1-vcl_exp(-alpha_data.data()[dataindex]*tree.cell_len(leaf_bits[t])); // TODO -- unused!!
                }
              }
            }
      }
      vcl_cout<<"Painted Cells in b lock "<<iter->first<<vcl_endl;
    }
    if (flag)
    {
      boxm2_sio_mgr::save_block_data_base(scene->data_path(), iter->first, label_data_base,boxm2_data_traits<BOXM2_LABEL_SHORT>::prefix(identifier));
      vcl_cout<<"Saved"<<vcl_endl;
    }
  }
  vcl_cout<<"DONE."<<vcl_endl;
  return true;
}
