#ifndef boxm_aux_scene_h_
#define boxm_aux_scene_h_
//:
// \file
// \brief  an auxiliary scene keeps a set of octrees for a given main scene
//
// \author Gamze Tunali
// \date   July 31, 2009
// \verbatim
//  Modifications
//   <none yet>
// \endverbatim

#include <vgl/vgl_box_3d.h>

#include <boct/boct_tree.h>
#include <boct/boct_tree_cell_reader.h>
#include <boxm/boxm_scene.h>

#include <boxm/boxm_block.h>
#include <boxm/opt/boxm_rt_sample.h>

template <class T_loc, class T, class T_AUX>
class boxm_aux_scene
{
 public:
   typedef  boct_tree<T_loc, T> tree_type;
   typedef  boct_tree<T_loc, T_AUX> aux_tree_type;
   boxm_aux_scene(){}
   boxm_aux_scene(boxm_scene<tree_type>* scene, vcl_string suffix);

   ~boxm_aux_scene(){/*delete aux_scene_;*/}

   void init_scene(){}


   boxm_block<boct_tree<T_loc,T_AUX> > * get_block(vgl_point_3d<int> index);

   //: get an incremental reader for a scene block
   boct_tree_cell_reader<T_loc, T_AUX >* get_block_incremental(vgl_point_3d<int> block_idx);

   void write_active_block() { aux_scene_->write_active_block(); }
   void clean_scene();

 private:
  boxm_scene<aux_tree_type>* aux_scene_;
  vcl_string aux_storage_dir_;
};

#endif
