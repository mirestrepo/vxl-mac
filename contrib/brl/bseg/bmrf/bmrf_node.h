// This is contrib/brl/bseg/bmrf/bmrf_node.h
#ifndef bmrf_node_h_
#define bmrf_node_h_
//:
// \file
// \brief A node in a Markov Random Field (MRF) network
// \author Matt Leotta, (mleotta@lems.brown.edu)
// \date 1/13/04
//
// The MRF node contains a link to an epi-segment as well as
// links to other neighboring nodes in the network
//
// \verbatim
//  Modifications
// \endverbatim

#include <vcl_vector.h>
#include <vsl/vsl_binary_io.h>
#include <vbl/vbl_ref_count.h>
#include <bmrf/bmrf_epi_seg_sptr.h>

//: A Markov Random Field (MRF) node
class bmrf_node : public vbl_ref_count
{
public:
  //: The values of this enum categorize the neighbors
  enum neighbor_type { SPACE, TIME, ALPHA };
  //: The number of neighbor types defined in the neighbor_type enum
  static const int number_of_types;
  
  //: Constructor
  bmrf_node( int frame_num = 0, double probability = 0.0 );
  
  //: Destructor
  ~bmrf_node(){}

  //: Calculate the conditional probability that this node is correct give its neighbors  
  double probability();

  //: Add \param node as a neighbor of type \param type
  // \return true if the node was added successfully
  // \return false if the neighbor is not valid or already exists
  bool add_neighbor( bmrf_node *node, neighbor_type type );

  //: Remove \param node from the neighborhood
  // \return true if the node is removed successfully
  // \return false if the node was not a neighbor
  bool remove_neighbor( bmrf_node *node, neighbor_type type );

  //: Return the frame number at which this node is found
  int frame_num() const { return frame_num_; }

  //: Binary save self to stream.
  void b_write(vsl_b_ostream &os) const;

  //: Binary load self from stream.
  void b_read(vsl_b_istream &is);

  //: Return IO version number;
  short version() const;

  //: Print an ascii summary to the stream
  void print_summary(vcl_ostream &os) const;

  //: Return a platform independent string identifying the class
  vcl_string is_a() const;

  //: Return true if the argument matches the string identifying the class or any parent class
  bool is_class(const vcl_string& cls) const;

private:
  //: The frame number associated with this node
  int frame_num_;
  
  //: The cached probability value
  double probability_;
  
  //: A smart pointer to the underlying epi-segment data
  bmrf_epi_seg_sptr segment_;
  
  //: The pointers to neighboring nodes
  // \note these are not smart pointer because there will be many cycles in the network
  vcl_vector<vcl_vector<bmrf_node*> > neighbors_;
};

//: Binary save bmrf_node* to stream.
void vsl_b_write(vsl_b_ostream &os, const bmrf_node* n);

//: Binary load bmrf_node* from stream.
void vsl_b_read(vsl_b_istream &is, bmrf_node* &n);

#endif // bmrf_node_h_
