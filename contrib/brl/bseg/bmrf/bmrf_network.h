// This is brl/bseg/bmrf/bmrf_network.h
#ifndef bmrf_network_h_
#define bmrf_network_h_
//:
// \file
// \brief A Markov Random Field (MRF) network
// \author Matt Leotta, (mleotta@lems.brown.edu)
// \date 1/13/04
//
// The MRF network is templated on the type of node, and maintains a vector
// of all nodes in the network.
//
// \verbatim
//  Modifications
// \endverbatim


#include <vcl_vector.h>
#include <vcl_deque.h>
#include <vcl_set.h>
#include <vsl/vsl_binary_io.h>
#include <vbl/vbl_ref_count.h>
#include "bmrf_node_sptr.h"
#include "bmrf_node.h"
#include "bmrf_network_sptr.h"
#include "bmrf_epi_seg_sptr.h"

//: The MRF network
class bmrf_network : public vbl_ref_count
{
 public:
  typedef vcl_map<bmrf_epi_seg*, bmrf_node_sptr> node_map;

  typedef bmrf_node::neighbor_type neighbor_type;
  
  //: Constructor
  bmrf_network();

  //: Destructor
  ~bmrf_network(){}

  //: Adds a new to the network
  // \retval true if the node was added
  // \retval false if the node could not be added
  // \note every node in the network must have a unique epi_segment
  bool add_node(const bmrf_node_sptr& node);

  //: Deletes a node in the network
  // \retval true if the node was deleted
  // \retval false if the node was not found in the network
  bool remove_node(const bmrf_node_sptr& node);

  //: Add an arc between \param n1 and \param n2 of type \param type
  bool add_arc( const bmrf_node_sptr& n1, const bmrf_node_sptr& n2, neighbor_type type );

  //: Add an arc between \param n1 and \param n2 of type \param type
  bool remove_arc( const bmrf_node_sptr& n1, const bmrf_node_sptr& n2, neighbor_type type = bmrf_node::ALL );

  //: Remove all arcs to NULL nodes and node not found in this network
  // \retval true if any arcs have been purged
  // \retval false if all arcs were found to be valid
  bool purge();

  //: Look up the node corresponding to an epi-segment
  // Returns a null smart pointer if no node exists
  bmrf_node_sptr seg_to_node(const bmrf_epi_seg_sptr& seg) const;

  //: Returns the number of nodes in the network;
  int size();

  //: Returns the probability that the entire network is correct
  double probability();

  //: Returns all the nodes in frame \param frame
  vcl_vector<bmrf_node_sptr> nodes_in_frame(int frame) const;

  //: Binary save self to stream.
  void b_write(vsl_b_ostream &os) const;

  //: Binary load self from stream.
  void b_read(vsl_b_istream &is);

  //: Return IO version number;
  short version() const;

  //: Print an ascii summary to the stream
  void print_summary(vcl_ostream &os) const;

 private:

  //: The map of nodes in the network
  // \note indexed by epi_seg pointers for quick reverse lookup
  node_map nodes_;

 public:
  class iterator
  {
   public:
    //: Constructor
    iterator( bmrf_network_sptr network, bmrf_node_sptr node ) : network_(network), curr_node_(node) {}

    //: Destructor
    virtual ~iterator() {}

    //: Increment
    iterator& operator++ () { next_node(); return *this; }

    //: Dereference
    bmrf_node_sptr operator -> () const { return curr_node_; }

    //: Equality comparison
    bool operator == (const iterator& rhs) { return rhs.curr_node_ == this->curr_node_; }

   protected:
    //: Increment the current node
    virtual void next_node() = 0;
    
    bmrf_network_sptr network_;
    bmrf_node_sptr curr_node_;
  };

  // Depth first search iterator
  class depth_iterator : public iterator
  {
   public:
    //: Constructor
    depth_iterator( bmrf_network_sptr network, bmrf_node_sptr node ) : iterator(network, node){}

   protected:
    //: Increment the current node
    void next_node();
    
    vcl_deque<bmrf_node_sptr> eval_queue_;
    vcl_set<bmrf_node_sptr> visited_;
  };

  // Breadth first search iterator
  class breadth_iterator : public iterator
  {
   public:
    //: Constructor
    breadth_iterator( bmrf_network_sptr network, bmrf_node_sptr node ) : iterator(network, node){}

   protected:
    //: Increment the current node
    void next_node();

    vcl_deque<bmrf_node_sptr> eval_queue_;
    vcl_set<bmrf_node_sptr> visited_;
  };

  //: Depth first search begin iterator
  depth_iterator depth_begin(bmrf_node_sptr node) { return depth_iterator(this, node); }
  //: Depth first search end iterator
  depth_iterator depth_end()   { return depth_iterator(this, NULL); }

  //: Breadth first search begin iterator
  breadth_iterator breadth_begin(bmrf_node_sptr node) { return breadth_iterator(this, node); }
  //: Breadth first search end iterator
  breadth_iterator breadth_end()   { return breadth_iterator(this, NULL); }
};


//: Binary save bmrf_network* to stream.
void vsl_b_write(vsl_b_ostream &os, const bmrf_network* n);

//: Binary load bmrf_network* from stream.
void vsl_b_read(vsl_b_istream &is, bmrf_network* &n);

//: Print an ASCII summary to the stream
void vsl_print_summary(vcl_ostream &os, const bmrf_network* n);


#endif // bmrf_network_h_
