#ifndef boct_tree_h_
#define boct_tree_h_

#include "boct_tree_cell.h"
#include "boct_loc_code.h"

#include <vgl/vgl_point_3d.h>

class boct_tree {
public:
  //boct_tree(short max_level);
  boct_tree(short max_level, short init_levels=1);
  boct_tree_cell_sptr locate_point(const vgl_point_3d<double>& p);
  boct_tree_cell_sptr locate_point_at_level(const vgl_point_3d<double>& p, short level);
  boct_tree_cell_sptr locate_region(const vgl_box_3d<double>& r);
  boct_tree_cell_sptr get_cell(const boct_loc_code& code);
  bool split();
  bool split_all();
  vcl_vector<boct_tree_cell_sptr> leaf_cells();
  //: return the max level
  short num_levels() { return max_level_; }
  void print();

private:
  short max_level_;
  boct_tree_cell_sptr root_;
};

#endif