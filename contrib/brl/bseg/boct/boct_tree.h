#ifndef boct_tree_h_
#define boct_tree_h_
//:
// \file
#include "boct_tree_cell.h"
#include "boct_loc_code.h"

#include <vgl/vgl_point_3d.h>
#include <vsl/vsl_binary_io.h>
#include <vgl/vgl_box_3d.h>

template <class T_loc, class T_data>
class boct_tree
{
 public:
  boct_tree(): root_(0),global_bbox_(){}
  boct_tree(short max_level, short init_levels=1);
  ~boct_tree();
  boct_tree_cell<T_loc,T_data>* locate_point(const vgl_point_3d<double>& p);
  boct_tree_cell<T_loc,T_data>* locate_point_at_level(const vgl_point_3d<double>& p, short level);
  boct_tree_cell<T_loc,T_data>* locate_region(const vgl_box_3d<double>& r);
  boct_tree_cell<T_loc,T_data>* get_cell( boct_loc_code<T_loc>& code) { return root_->traverse(code); }
  boct_tree_cell<T_loc,T_data>* root(){return root_;}
  bool split();
  vcl_vector<boct_tree_cell<T_loc,T_data>*> leaf_cells();
  //: return the max level
  short num_levels() { return max_level_; }
  void set_bbox(vgl_box_3d<double> & bbox){global_bbox_=bbox;}

  vgl_box_3d<double> cell_bounding_box(boct_tree_cell<T_loc,T_data>* const cell);

  //: returns the location of the cell in the block, with respect to blocks min point
  vgl_box_3d<double> cell_bounding_box_local(boct_tree_cell<T_loc,T_data>* const cell);

  vgl_box_3d<double> bounding_box(){return global_bbox_;}
  
  void print();
  
  void b_write(vsl_b_ostream & os);
  void b_read(vsl_b_istream & is);
  short version_no() { return 1; }
 private:
  short max_level_;
  boct_tree_cell<T_loc,T_data>* root_;

  //: the local coordinates are from 0 to 1
  // that's why we need the global coordinates
  vgl_box_3d<double> global_bbox_;
};

template <class T_loc,class T_data>
void vsl_b_write(vsl_b_ostream & os, boct_tree<T_loc,T_data>& tree) { tree.b_write(os); }

template <class T_loc,class T_data>
void vsl_b_read(vsl_b_istream & is, boct_tree<T_loc,T_data>& tree) { tree.b_read(is); }

#endif
