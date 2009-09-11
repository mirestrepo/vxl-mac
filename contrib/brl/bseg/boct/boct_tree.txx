#ifndef boct_tree_txx_
#define boct_tree_txx_

#include "boct_tree.h"
#include <vcl_cmath.h>
#include <vcl_iostream.h>
#include <vgl/vgl_box_3d.h>
#include <vgl/io/vgl_io_box_3d.h>

//; constructor initializes an empty tree
template <class T_loc,class T_data>
boct_tree<T_loc,T_data>::boct_tree(short max_level, short init_levels): max_level_(max_level)
{
  // root is allocated at (max_level_-1) with code [0,0,0]
  boct_loc_code<T_loc> code;
  if (init_levels>0)
  {
    code.set_code(0,0,0);
    code.set_level(max_level_-1);
    root_=new boct_tree_cell<T_loc,T_data>( code);
  }
  init_levels--;
  while (init_levels > 0) {
    vcl_vector<boct_tree_cell<T_loc,T_data>*> cells;
    cells = leaf_cells();
    for (unsigned i=0; i<cells.size(); i++) {
      boct_tree_cell<T_loc,T_data>* c = static_cast<boct_tree_cell<T_loc,T_data>*>(cells[i]);
      c->split();
      }
    init_levels--;
  }
}

template <class T_loc,class T_data>
boct_tree<T_loc,T_data>::boct_tree(vgl_box_3d<double>  bbox,short max_level, short init_levels)
: max_level_(max_level),global_bbox_(bbox)
{
  // root is allocated at (max_level_-1) with code [0,0,0]
  boct_loc_code<T_loc> code;
  if (max_level_>0)
  {
    code.set_code(0,0,0);
    code.set_level(max_level_-1);
    root_=new boct_tree_cell<T_loc,T_data>( code);
  }
  init_levels--;
  while (init_levels > 0) {
    vcl_vector<boct_tree_cell<T_loc,T_data>*> cells;
    cells = leaf_cells();
    for (unsigned i=0; i<cells.size(); i++) {
      boct_tree_cell<T_loc,T_data>* c = static_cast<boct_tree_cell<T_loc,T_data>*>(cells[i]);
      c->split();
      }
    init_levels--;
  }
}

template <class T_loc,class T_data>
boct_tree<T_loc,T_data>::~boct_tree()
{
  if (root_)
  {
    if (root_->is_leaf()) {
      delete root_;
    } else {
      root_->delete_children();
      delete root_;
    }
  }
}

template <class T_loc,class T_data>
boct_tree<T_loc,T_data>* boct_tree<T_loc,T_data>::clone()
{
  boct_tree_cell<T_loc, T_data>* root = root_->clone(0);
  boct_tree<T_loc,T_data>* tree = new boct_tree<T_loc,T_data>(root,max_level_);
  return tree;
}

template <class T_loc,class T_data>
void boct_tree<T_loc,T_data>::init_cells(T_data val)
{
  vcl_vector<boct_tree_cell<T_loc,T_data>*> cells = leaf_cells();
  for (unsigned i=0; i<cells.size(); i++) {
    cells[i]->set_data(val);
  }
}

template <class T_loc,class T_data>
boct_tree_cell<T_loc,T_data>* boct_tree<T_loc,T_data>::locate_point(const vgl_point_3d<double>& p)
{
  short curr_level=max_level_-1;
  // convert point to location code.
  boct_loc_code<T_loc>* loccode_=new boct_loc_code<T_loc>(p, max_level_);
#if 0
  // check to see if point is contained in the octree
  if (!root_->code_.isequal(loccode_,curr_level))
    return NULL;
#endif
  // temporary pointer to traverse
  boct_tree_cell<T_loc,T_data>* curr_cell=root_;

  while (curr_cell->children()&& curr_level>0)
  {
    short index_child=loccode_->child_index(curr_level);
    curr_cell=curr_cell->children()+index_child;
    --curr_level;
  }
  // delete the location code constructed
  delete loccode_;
  return curr_cell;
}

template <class T_loc,class T_data>
boct_tree_cell<T_loc,T_data>* boct_tree<T_loc,T_data>::locate_point_global(const vgl_point_3d<double>& p)
{
  short curr_level=max_level_-1;
  vgl_point_3d<double> norm_p((p.x()-global_bbox_.min_x())/global_bbox_.width(),
                              (p.y()-global_bbox_.min_y())/global_bbox_.height(),
                              (p.z()-global_bbox_.min_z())/global_bbox_.depth());

 
//  if(norm_p.x()>=1.0 ||norm_p.y()>=1.0||norm_p.z()>=1.0
//     ||norm_p.x()<0.0 ||norm_p.y()<0.0||norm_p.z()<0.0)
//     return 0;
  // convert point to location code.
  boct_loc_code<T_loc>* loccode_=new boct_loc_code<T_loc>(norm_p, max_level_);
#if 0
  // check to see if point is contained in the octree
  if (!root_->code_.isequal(loccode_,curr_level))
    return NULL;
#endif
  // temporary pointer to traverse
  boct_tree_cell<T_loc,T_data>* curr_cell=root_;

  while (curr_cell->children()&& curr_level>0)
  {
    short index_child=loccode_->child_index(curr_level);
    curr_cell=curr_cell->children()+index_child;
    --curr_level;
  }
  // delete the location code constructed
  delete loccode_;
  return curr_cell;
}

template <class T_loc,class T_data>
boct_tree_cell<T_loc,T_data>* boct_tree<T_loc,T_data>::locate_point_at_level(const vgl_point_3d<double>& p, short level)
{
  short curr_level=max_level_-1;
  // convert point to location code.
  boct_loc_code<T_loc>* loccode_=new boct_loc_code<T_loc>(p, max_level_);
#if 0
  // check to see if point is contained in the octree
  //if (!root_->code_.isequal(loccode_,curr_level))
  //  return NULL;
#endif
  // temporary pointer to traverse
  boct_tree_cell<T_loc,T_data>* curr_cell=root_;

  while (curr_cell->children()&& curr_level>level)
  {
    short child_index=loccode_->child_index(curr_level);
    curr_cell=curr_cell->children()+child_index;
    --curr_level;
  }
  // delete the location code constructed
  delete loccode_;
  return curr_cell;
}

template <class T_loc,class T_data>
boct_tree_cell<T_loc,T_data>* boct_tree<T_loc,T_data>::locate_region(const vgl_box_3d<double>& r)
{
  boct_loc_code<T_loc>* mincode=new boct_loc_code<T_loc>(r.min_point(), max_level_);
  boct_loc_code<T_loc>* maxcode=new boct_loc_code<T_loc>(r.max_point(), max_level_);

  boct_loc_code<T_loc>* xorcode=mincode->XOR(maxcode);

  short level_x=max_level_-1;
  short level_y=max_level_-1;
  short level_z=max_level_-1;
  while (!(xorcode->x_loc_&(1<<level_x))&& level_x) level_x--;
  while (!(xorcode->y_loc_&(1<<level_y))&& level_y>level_x) level_y--;
  while (!(xorcode->z_loc_&(1<<level_z))&& level_z>level_y) level_z--;

  level_z++;
  return locate_point_at_level(r.min_point(),level_z);
}

template <class T_loc,class T_data>
bool boct_tree<T_loc,T_data>::split()
{
  return root_->split();
}

template <class T_loc,class T_data>
vcl_vector<boct_tree_cell<T_loc,T_data>*> boct_tree<T_loc,T_data>::leaf_cells()
{
  vcl_vector<boct_tree_cell<T_loc,T_data>*> v;
  if (root_)
  {
    if (root_->is_leaf()) {
      v.push_back(root_);
    }
    else {
      root_->leaf_children(v);
    }
  }
  return v;
}

template <class T_loc,class T_data>
short boct_tree<T_loc,T_data>::finest_level()
{
  short min_level = max_level_;
  vcl_vector<boct_tree_cell<T_loc,T_data>*> cells = leaf_cells();
  for (unsigned i=0; i<cells.size(); i++) {
    if (cells[i]->code_.level < min_level)
      min_level = cells[i]->code_.level;
  }
  return min_level;
}
 
template <class T_loc,class T_data>
vgl_box_3d<double> boct_tree<T_loc,T_data>::cell_bounding_box(boct_tree_cell<T_loc,T_data>* const cell)
{
  double treesize=(double)(1<<(max_level_-1));
  double cellsize=(double)(1<<cell->level())/treesize;
  vgl_point_3d<double> local_origin(cell->code_.x_loc_,cell->code_.y_loc_,cell->code_.z_loc_);
  vgl_point_3d<double> global_origin(global_bbox_.min_x()+local_origin.x()/treesize*global_bbox_.width(),
                                     global_bbox_.min_y()+local_origin.y()/treesize*global_bbox_.height(),
                                     global_bbox_.min_z()+local_origin.z()/treesize*global_bbox_.depth());

  return vgl_box_3d<double>(global_origin,
                            cellsize*global_bbox_.width(),
                            cellsize*global_bbox_.height(),
                            cellsize*global_bbox_.depth(),
                            vgl_box_3d<double>::min_pos);
}

template <class T_loc,class T_data>
vgl_box_3d<double> boct_tree<T_loc,T_data>::cell_bounding_box_local(boct_tree_cell<T_loc,T_data>* const cell)
{
  double treesize=(double)(1<<(max_level_-1));
  double cellsize=(double)(1<<cell->level())/treesize;
  vgl_point_3d<double> local_origin(cell->code_.x_loc_,cell->code_.y_loc_,cell->code_.z_loc_);

  vgl_point_3d<double> global_origin(local_origin.x()/treesize*global_bbox_.width(),
                                     local_origin.y()/treesize*global_bbox_.height(),
                                     local_origin.z()/treesize*global_bbox_.depth());

  return vgl_box_3d<double>(global_origin,
                            cellsize*global_bbox_.width(),
                            cellsize*global_bbox_.height(),
                            cellsize*global_bbox_.depth(),
                            vgl_box_3d<double>::min_pos);
}

template <class T_loc,class T_data>
void boct_tree<T_loc,T_data>::print()
{
  vcl_cout << "Octree Max Level=" << max_level_ << vcl_endl;
  root_->print();
}

template <class T_loc,class T_data>
void boct_tree<T_loc,T_data>::b_write(vsl_b_ostream & os)
{
  vsl_b_write(os, version_no());
  vsl_b_write(os, max_level_);
  vsl_b_write(os, global_bbox_);
  if (root_)
    vsl_b_write(os, *root_);
}

template <class T_loc,class T_data>
void boct_tree<T_loc,T_data>::b_read(vsl_b_istream & is)
{
  // read header info
  if (!is) return;

  short v;
  vsl_b_read(is, v);
  switch (v)
  {
   case (1):
     //short max_level;
     vsl_b_read(is, max_level_);
     vsl_b_read(is, global_bbox_);

     root_ = new boct_tree_cell<T_loc,T_data>();
     vsl_b_read(is, *root_, (boct_tree_cell<T_loc,T_data>*)0);
     break;
   default:
     vcl_cerr << "I/O ERROR: vsl_b_read(vsl_b_istream&, boct_tree<T_loc,T_data>&)\n"
              << "           Unknown version number "<< v << '\n';
     is.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
     return;
  }
}

#define BOCT_TREE_INSTANTIATE(T_loc,T_data) \
template class boct_tree<T_loc,T_data >; \
template void vsl_b_write(vsl_b_ostream & os, boct_tree<T_loc,T_data >&); \
template void vsl_b_read(vsl_b_istream & is, boct_tree<T_loc,T_data >&)

#endif
