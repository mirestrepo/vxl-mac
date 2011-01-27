#include "test_utils.h"
//:
// \file
// \author Isabel Restrepo
// \date 15-Aug-2010

void init_tree(boct_tree<short,float> *tree, unsigned i)
{
  tree-> split(); //now we have 8 cells
  vcl_vector<boct_tree_cell<short,float>*> leaves = tree->leaf_cells();
  leaves[i]->set_data(0.8f);
  leaves[i]->split();
}

boxm_scene<boct_tree<short, float> >* create_scene(unsigned world_dimx,unsigned world_dimy,unsigned world_dimz)
{
  //crete the input scene
  unsigned int max_tree_level = 3;
  unsigned int init_level = 1;
  bgeo_lvcs lvcs(33.33,44.44,10.0, bgeo_lvcs::wgs84, bgeo_lvcs::DEG, bgeo_lvcs::METERS);
  vgl_point_3d<double> origin(0,0,0);

  vgl_vector_3d<double> block_dim(10, 10, 10); //world coordinate dimensions of a block

  vgl_vector_3d<unsigned> world_dim(world_dimx,world_dimy,world_dimz); //number of blocks in a scene

  boxm_scene<boct_tree<short, float> > *scene = new boxm_scene<boct_tree<short, float> >(lvcs, origin, block_dim, world_dim, max_tree_level, init_level );
  vcl_string scene_path(".");
  scene->set_paths(scene_path, "test_scene");
  scene->set_appearance_model(BOXM_FLOAT);
  scene->write_scene("test_scene.xml");

  unsigned cell_index = 7;
  boxm_block_iterator<boct_tree<short, float> > iter=scene->iterator();
  iter.begin();
  while (!iter.end())
  {
    scene->load_block(iter.index());
    boxm_block<boct_tree<short, float> > *block = scene->get_active_block();
    // Construct an empty tree with 3 maximum levels 1 levele initialized to 0.0
    boct_tree<short,float> *tree = new boct_tree<short,float>(0.5f, 3, 1);
    //tree->print();
    init_tree(tree, cell_index);
    //tree->print();
    block->init_tree(tree);
    scene->write_active_block();
    if (cell_index == 0) cell_index = 7;
    cell_index--;
    ++iter;
  }

  return scene;
}


void clean_up()
{
  //clean temporary files
  vul_file_iterator file_it("./*.bin");
  for (; file_it; ++file_it)
  {
    vpl_unlink(file_it());
    vul_file::delete_file_glob(file_it());
  }
}

void clean_up(vcl_string dir, vcl_string ext)
{
  //clean temporary files
  vul_file_iterator file_it(dir+"/"+ext);
  for (; file_it; ++file_it)
  {
    vpl_unlink(file_it());
    vul_file::delete_file_glob(file_it());
  }
}
