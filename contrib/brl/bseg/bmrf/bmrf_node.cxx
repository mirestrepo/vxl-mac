// This is contrib/brl/bseg/bmrf/bmrf_node.cxx
//:
// \file

#include "bmrf_node.h"
#include <vsl/vsl_binary_io.h>
#include <vbl/io/vbl_io_smart_ptr.h>
#include <vcl_algorithm.h>


//: Constructor
bmrf_node::bmrf_node( const bmrf_epi_seg_sptr& epi_seg, int frame_num, double probability )
  : segment_(epi_seg), frame_num_(frame_num), probability_(probability),
    out_arcs_(), in_arcs_(), boundaries_(ALL+1, out_arcs_.end()), sizes_(ALL,0)
{
}

//: Strip all of the arcs from this node
void
bmrf_node::strip()
{
  arc_iterator itr = out_arcs_.begin();
  for(; itr != out_arcs_.end(); ++itr){
    if((*itr)->to){
      bmrf_node_sptr other = (*itr)->to;
      arc_iterator itr2 = other->in_arcs_.begin();
      for(; itr2 != other->in_arcs_.end(); ++itr2)
        other->in_arcs_.erase(itr2);  
    }
  }
  out_arcs_.clear();
  for(int t = 0; t<=ALL; ++t)
    boundaries_[t] = out_arcs_.end();

  itr = in_arcs_.begin();
  for(; itr != in_arcs_.end(); ++itr){
    if((*itr)->from){
      (*itr)->from->remove_neighbor(this, ALL);
    }
  } 
}


//: Remove any arcs to or from NULL nodes
bool
bmrf_node::purge()
{
  bool retval = false;
  for(int t = 0; t<ALL; ++t){
    arc_iterator itr = boundaries_[t];
    for(; itr != boundaries_[t+1]; ++itr){
      if(!(*itr)->to){
        // adjust boundaries if necessary
        for(int t2=t; t2>=0 && (boundaries_[t2] == itr); --t2)
          ++boundaries_[t2];
        out_arcs_.erase(itr--); // remove the arc
        --sizes_[t]; // decrease count
        retval = true;
      }
    }
  }

  arc_iterator itr = in_arcs_.begin();
  for(; itr != in_arcs_.end(); ++itr){
    if(!(*itr)->from){
      in_arcs_.erase(itr--);
      retval = true;
    }
  }
  
  return retval;
}
  

//: Calculate the conditional probability that this node is correct give its neighbors
double
bmrf_node::probability(  )
{
  // Have not yet determined how this will be computed
  return probability_;
}


//: Add \param node as a neighbor of type \param type
bool
bmrf_node::add_neighbor( bmrf_node *node, neighbor_type type )
{
  if(!node || node == this || type == ALL)
    return false;
    
  // verify that this arc is not already present
  arc_iterator itr = boundaries_[type];
  for(; itr != boundaries_[type+1]; ++itr)
    if((*itr)->to == node) return false;
    
  // add the arc
  bmrf_arc_sptr new_arc = new bmrf_arc(this, node);
  out_arcs_.insert(boundaries_[type+1], new_arc);
  node->in_arcs_.push_back(new_arc);
  ++sizes_[type];

  // adjust boundaries if necessary
  for(int t=type; t>=0 && (boundaries_[type+1] == boundaries_[t]); --t)
    --boundaries_[t];

  return true;
}


//: Remove \param node from the neighborhood
bool
bmrf_node::remove_neighbor( bmrf_node *node, neighbor_type type )
{
  if(!node || node == this)
    return false;

  bool removed = false;
  int init_t = type;
  if(type == ALL) init_t=0;

  for(int t = init_t; t<ALL; ++t){
    arc_iterator itr = boundaries_[t];
    for(; itr != boundaries_[t+1]; ++itr){
      if((*itr)->to == node){
        // adjust boundaries if necessary
        for(int t2=t; t2>=0 && (boundaries_[t2] == itr); --t2)
          ++boundaries_[t2];
        arc_iterator back_itr = vcl_find((*itr)->to->in_arcs_.begin(), (*itr)->to->in_arcs_.end(), *itr);
        if(back_itr != (*itr)->to->in_arcs_.end())
          (*itr)->to->in_arcs_.erase(back_itr); // erase the pointer back from the other node
        out_arcs_.erase(itr--); // erase the arc
        --sizes_[t]; // decrease count
        if (type != ALL) return true;
        removed = true;
      }
    }
  }
  return removed;
}


//: Returns an iterator to the beginning of the type \param type neighbors
bmrf_node::arc_iterator
bmrf_node::begin( neighbor_type type )
{
  if (type == ALL) return out_arcs_.begin();
  return boundaries_[type];
}

  
//: Returns an iterator to the end of the type \param type neighbors
bmrf_node::arc_iterator
bmrf_node::end( neighbor_type type )
{
  if (type == ALL) return out_arcs_.end();
  return boundaries_[type+1];
}


//: Returns the number of outgoing neighbors to this node of type \param type
int
bmrf_node::num_neighbors( neighbor_type type )
{
  if (type == ALL) return out_arcs_.size();
  return sizes_[type];
}
    

//: Binary save self to stream.
void
bmrf_node::b_write( vsl_b_ostream& os ) const
{
  vsl_b_write(os, version());                                 
  vsl_b_write(os, this->segment_);
  vsl_b_write(os, this->frame_num_);
  vsl_b_write(os, this->probability_);

  // write the number of nodes of each type
  for(int t=0; t<ALL; ++t)
    vsl_b_write(os, sizes_[t]);

  // write all the outgoing arcs
  vcl_list<bmrf_arc_sptr>::const_iterator itr = out_arcs_.begin();
  for(; itr != out_arcs_.end(); ++itr){
    vsl_b_write(os, *itr);  // Save the arc
  }

  // write the number of incoming arcs
  vsl_b_write(os, in_arcs_.size());
  // write all the incoming arcs
  itr = in_arcs_.begin();
  for(; itr != in_arcs_.end(); ++itr){
    vsl_b_write(os, *itr);  // Save the arc
  }
}

  
//: Binary load self from stream.
void
bmrf_node::b_read( vsl_b_istream& is )
{
  if (!is) return;
  
  short ver;
  vsl_b_read(is, ver);
  switch(ver)
  {
  case 1:
    {
      vsl_b_read(is, this->segment_);
      vsl_b_read(is, this->frame_num_);
      vsl_b_read(is, this->probability_);

      out_arcs_.clear();
      boundaries_.clear();
      boundaries_.resize(ALL+1, out_arcs_.end());

      int num_neighbors = 0;
      for(int t=0; t<ALL; ++t){
        vsl_b_read(is, sizes_[t]);
        num_neighbors += sizes_[t];
      }
      int type = 0;
      int b_loc = sizes_[0];
      for(int n=0; n<num_neighbors; ++n){
        bmrf_arc_sptr arc_ptr;
        vsl_b_read(is, arc_ptr);
        arc_ptr->from = this;
        out_arcs_.push_back(arc_ptr);
        
        while(type < ALL && n == b_loc){
          boundaries_[++type] = out_arcs_.end();
          --boundaries_[type];
          b_loc += sizes_[type];
        }
      }
      boundaries_[0] = out_arcs_.begin();

      int num_incoming;
      vsl_b_read(is, num_incoming);
      for(int n=0; n<num_incoming; ++n){
        bmrf_arc_sptr arc_ptr;
        vsl_b_read(is, arc_ptr);
        arc_ptr->to = this;
        in_arcs_.push_back(arc_ptr);
      }
    }   
    break;

  default:
    vcl_cerr << "I/O ERROR: bmrf_node::b_read(vsl_b_istream&)\n"
             << "           Unknown version number "<< ver << '\n';
    is.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
    return;
  }
}


//: Return IO version number;
short
bmrf_node::version(  ) const
{
  return 1;
}


//: Print an ascii summary to the stream
void
bmrf_node::print_summary( vcl_ostream& os ) const
{
  os << "bmrf_node{ pr=" << probability_ << ", frame=" << frame_num_ << " }";
}


//-----------------------------------------------------------------------------------------
// bmrf_arc member functions
//-----------------------------------------------------------------------------------------


//: Binary save self to stream.
void
bmrf_node::bmrf_arc::b_write( vsl_b_ostream& os ) const
{
  // Nothing to write
}


//: Binary load self from stream.
void
bmrf_node::bmrf_arc::b_read( vsl_b_istream& is )
{
  // Nothing to read
}


//-----------------------------------------------------------------------------------------
// External functions
//-----------------------------------------------------------------------------------------

//: Binary save bmrf_node to stream.
void
vsl_b_write(vsl_b_ostream &os, const bmrf_node* n)
{
  if (n==0){
    vsl_b_write(os, false); // Indicate null pointer stored
  }
  else{
    vsl_b_write(os,true); // Indicate non-null pointer stored
    n->b_write(os);
  }
}


//: Binary load bmrf_node from stream.
void
vsl_b_read(vsl_b_istream &is, bmrf_node* &n)
{
  delete n;
  bool not_null_ptr;
  vsl_b_read(is, not_null_ptr);
  if (not_null_ptr){
    n = new bmrf_node();
    n->b_read(is);
  }
  else
    n = 0;
}



//: Binary save bmrf_node::bmrf_arc to stream.
void
vsl_b_write(vsl_b_ostream &os, const bmrf_node::bmrf_arc* a)
{
  if (a==0){
    vsl_b_write(os, false); // Indicate null pointer stored
  }
  else{
    vsl_b_write(os,true); // Indicate non-null pointer stored
    a->b_write(os);
  }
}


//: Binary load bmrf_node::bmrf_arc from stream.
void
vsl_b_read(vsl_b_istream &is, bmrf_node::bmrf_arc* &a)
{
  delete a;
  bool not_null_ptr;
  vsl_b_read(is, not_null_ptr);
  if (not_null_ptr){
    a = new bmrf_node::bmrf_arc();
    a->b_read(is);
  }
  else
    a = 0;
}
