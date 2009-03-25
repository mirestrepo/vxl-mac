#ifndef boct_tree_cell_h_
#define boct_tree_cell_h_

#include "boct_loc_code.h"

class boct_tree_cell
{
public:
  typedef enum {NONE, X_LOW, X_HIGH, Y_LOW, Y_HIGH, Z_LOW, Z_HIGH, ALL} FACE_IDX;

  //constructors
  boct_tree_cell();
  boct_tree_cell(const boct_loc_code& code, boct_tree_cell* p, short l) {code_=code; children_=0; parent_=p; level_=l; }
  //constructor given code and level
  boct_tree_cell(const boct_loc_code& code, short level);
  boct_tree_cell(const boct_tree_cell& rhs);
  bool is_leaf();

  //: adds a pointer for each leaf children to v
  void leaf_children(vcl_vector<boct_tree_cell*>& v);

  const boct_loc_code& get_code();

  boct_tree_cell* traverse(boct_loc_code code);
  boct_tree_cell* traverse_to_level(boct_loc_code *code, short level);
  bool split();
  void print();
  short level(){return level_;}
  boct_tree_cell* children(){return children_;}
  void  find_neighbors(FACE_IDX face,vcl_vector<boct_tree_cell*> & neighbors,short max_level);
  boct_tree_cell *get_common_ancestor(short binarydiff);

  boct_loc_code code_;
private:
  short level_;
  boct_tree_cell* parent_;
  boct_tree_cell* children_;
  
};

#endif