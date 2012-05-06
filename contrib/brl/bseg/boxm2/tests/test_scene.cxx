//:
// \file
// \author Andy Miller
// \date 26-Oct-2010
#include <boxm2/boxm2_scene.h>
#include <testlib/testlib_test.h>
#include <testlib/testlib_root_dir.h>
#include <vgl/vgl_point_3d.h>
#include <vpl/vpl.h>

void test_scene()
{
  //test xml file
  vcl_string test_dir  = testlib_root_dir()+ "/contrib/brl/bseg/boxm2/tests/";
  vcl_string test_file = test_dir + "test.xml";

  //create block metadata
  vcl_map<boxm2_block_id, boxm2_block_metadata> blocks;
  for (int i=0; i<2; i++) {
    for (int j=0; j<2; j++) {
      double big_block_side = 2.0;
      boxm2_block_id id(i,j,0);
      boxm2_block_metadata data;
      data.id_ = id;
      data.local_origin_ = vgl_point_3d<double>(big_block_side*i, big_block_side*j, 0.0);
      data.sub_block_dim_ = vgl_vector_3d<double>(.2, .2, .2);
      data.sub_block_num_ = vgl_vector_3d<unsigned>(10, 10, 10);
      data.init_level_ = 1;
      data.max_level_ = 4;
      data.max_mb_ = 400;
      data.p_init_ = .001;
      data.version_ = 1;

      //push it into the map
      blocks[id] = data;
    }
  }

  //create scene
  boxm2_scene scene;
  scene.set_local_origin(vgl_point_3d<double>(0.0, 0.0, 0.0));
  scene.set_rpc_origin(vgl_point_3d<double>(0.0, 0.0, 0.0));
  vpgl_lvcs lvcs;
  scene.set_lvcs(lvcs);
  scene.set_xml_path(test_file);
  scene.set_data_path(test_dir);
  scene.set_blocks(blocks);
  scene.save_scene();

  //create test scene
  boxm2_scene test_scene_data(test_file);
  vcl_cout<<test_scene_data<<vcl_endl;

  //delete file created
  vpl_unlink(test_file.c_str());
}


TESTMAIN(test_scene);
