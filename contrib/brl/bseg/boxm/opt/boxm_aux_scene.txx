#ifndef boxm_aux_scene_txx_
#define boxm_aux_scene_txx_

#include <vbl/vbl_bounding_box.h>
#include <vul/vul_file_iterator.h>
#include <vul/vul_file.h>
#include <vul/vul_expand_path.h>
#include <vgl/vgl_vector_3d.h>
#include <vgl/vgl_point_3d.h>

#include "boxm_aux_traits.h"
#include "boxm_aux_scene.h"

template <class T_loc, class T, class T_AUX>
boxm_aux_scene<T_loc,T,T_AUX>::boxm_aux_scene(boxm_scene<tree_type>* scene, 
                                              vcl_string storage_suffix = "")
{
  vcl_string aux_storage_dir;
  aux_storage_dir_ = scene->path(); 
  aux_scene_ = new boxm_scene<aux_tree_type >(scene->lvcs(), scene->origin(), scene->block_dim(), scene->world_dim());
  aux_scene_->set_path(aux_storage_dir_,  storage_suffix);
  
  // loop through valid blocks and init same blocks in aux scene
  boxm_block_iterator<tree_type > iter(scene);
  iter.begin();
  while (!iter.end()) {
    if (aux_scene_->load_block(iter.index().x(),iter.index().y(),iter.index().z())) {
      // auxilary block file exist 
      boxm_block<boct_tree<T_loc,T_AUX> > * block=aux_scene_->get_active_block();
      assert(block->get_tree() != 0);
    } else {  //clone from the primary block
      scene->load_block(iter.index().x(),iter.index().y(),iter.index().z());
      boxm_block<boct_tree<T_loc,T> > * block=scene->get_active_block();
      boct_tree<T_loc,T>* tree=block->get_tree();
      aux_tree_type * aux_tree = tree->template clone_to_type<T_AUX>();
      boxm_block<aux_tree_type >* aux_block = new boxm_block<aux_tree_type >(block->bounding_box(), aux_tree);
      aux_scene_->set_block(iter.index(), aux_block);
      aux_scene_->write_active_block();
    }
    iter++;
  }
}

template <class T_loc, class T, class T_AUX>
boct_tree_cell_reader<T_loc, T_AUX>* boxm_aux_scene<T_loc,T,T_AUX>::get_block_incremental(vgl_point_3d<int> block_idx)
{
  vcl_string path=aux_scene_->gen_block_path(block_idx.x(), block_idx.y(), block_idx.z());
  //vsl_b_ifstream is(path);
  boct_tree_cell_reader<T_loc, T_AUX>* reader = new boct_tree_cell_reader<T_loc, T_AUX>(path);//&is);
  reader->begin();
  return reader;
}
/*template <class T_loc, class T, class T_AUX>
boct_tree_cell_reader<T_loc, T_AUX>* boxm_aux_scene<T_loc,T,T_AUX>::save_scene()
{
  boxm_block_iterator<tree_type > iter(aux_scene);
  iter.begin();
  while (!iter.end()) {
  }
}*/
#define BOXM_AUX_SCENE_INSTANTIATE(T1,T2,T3) \
template class boxm_aux_scene<T1,T2,T3 >

#endif
