// This is brl/bseg/bmrf/bmrf_node.cxx
#include "bmrf_node.h"
//:
// \file

#include <bmrf/bmrf_epi_transform.h>
#include <bmrf/bmrf_gamma_func.h>
#include <vsl/vsl_binary_io.h>
#include <vbl/io/vbl_io_smart_ptr.h>
#include <vcl_algorithm.h>
#include <vcl_cmath.h>

#undef MAX
#define MAX(a,b) ((a)>(b)?(a):(b))
#undef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))

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
  for (arc_iterator itr = out_arcs_.begin(); itr != out_arcs_.end(); ++itr)
  {
    if ((*itr)->to_) {
      bmrf_node_sptr other = (*itr)->to();
      for (arc_iterator itr2 = other->in_arcs_.begin();
           itr2 != other->in_arcs_.end(); ++itr2)
        other->in_arcs_.erase(itr2);
    }
  }
  out_arcs_.clear();
  for (int t = 0; t<=ALL; ++t)
    boundaries_[t] = out_arcs_.end();

  for (arc_iterator itr = in_arcs_.begin(); itr != in_arcs_.end(); ++itr)
    if ((*itr)->from_)
      (*itr)->from_->remove_neighbor(this, ALL);
}


//: Remove any arcs to or from NULL nodes
bool
bmrf_node::purge()
{
  bool retval = false;
  for (int t = 0; t<ALL; ++t)
  {
    for (arc_iterator itr = boundaries_[t]; itr != boundaries_[t+1]; ++itr) {
      if (!(*itr)->to_) {
        // adjust boundaries if necessary
        for (int t2=t; t2>=0 && (boundaries_[t2] == itr); --t2)
          ++boundaries_[t2];
        out_arcs_.erase(itr--); // remove the arc
        --sizes_[t]; // decrease count
        retval = true;
      }
    }
  }

  
  for (arc_iterator itr = in_arcs_.begin(); itr != in_arcs_.end(); ++itr) {
    if (!(*itr)->from_) {
      in_arcs_.erase(itr--);
      retval = true;
    }
  }

  return retval;
}


// Helper function to compute the average distance ratio in the overlapping alpha
static double
avg_distance_ratio( const bmrf_epi_seg_sptr& ep1, const bmrf_epi_seg_sptr& ep2)
{
  double min_alpha = MAX(ep1->min_alpha(), ep2->min_alpha());
  double max_alpha = MIN(ep1->max_alpha(), ep2->max_alpha());
  double d_alpha =  MIN( (ep1->max_alpha() - ep1->min_alpha())/ep1->n_pts() ,
                         (ep2->max_alpha() - ep2->min_alpha())/ep2->n_pts() );

  double s1 = 0.0, s2 = 0.0;
  for (double alpha = min_alpha; alpha <= max_alpha; alpha += d_alpha){
    s1 += ep1->s(alpha);
    s2 += ep2->s(alpha);
  }
  return s1 / s2;
}


// Helper function to compute match error between to segments
static double
bmrf_match_error( const bmrf_epi_seg_sptr& ep1, const bmrf_epi_seg_sptr& ep2 )
{
  double min_alpha = MAX(ep1->min_alpha(), ep2->min_alpha());
  double max_alpha = MIN(ep1->max_alpha(), ep2->max_alpha());

  double d_alpha =  MIN( (ep1->max_alpha() - ep1->min_alpha())/ep1->n_pts() ,
                         (ep2->max_alpha() - ep2->min_alpha())/ep2->n_pts() );
  // static double d_alpha = 0.0006;

  double s_error = 0.0;
  for (double alpha = min_alpha; alpha <= max_alpha; alpha += d_alpha) {
    double ds = ep1->s(alpha) - ep2->s(alpha);
    s_error += ds*ds;
  }
  return s_error * d_alpha / (max_alpha - min_alpha);
}


//: Calculate the conditional probability that this node is correct give its neighbors
double
bmrf_node::probability()
{
  if (weight_.empty())
    this->compute_weights();

  vcl_vector<vcl_pair<double,double> > pmf;
  for ( arc_iterator a_itr = this->begin(TIME); a_itr != this->end(TIME); ++a_itr ) {
    bmrf_node_sptr neighbor = (*a_itr)->to();
    double dist_ratio = avg_distance_ratio(this->epi_seg(), neighbor->epi_seg());
    int time_step = neighbor->frame_num() - this->frame_num();
    double gamma = (1.0 - dist_ratio) / time_step;

    bmrf_gamma_func_sptr gamma_func = new bmrf_const_gamma_func(gamma);
    pmf.push_back(vcl_pair<double,double>(this->probability(gamma_func),gamma));
  }
  vcl_sort(pmf.begin(), pmf.end());
#ifdef DEBUG
  vcl_cout << "Samples\n";
  for ( vcl_vector<vcl_pair<double,double> >::iterator p_itr = pmf.begin();
        p_itr != pmf.end();  ++p_itr )
    vcl_cout << p_itr->second <<'\t'<< p_itr->first << vcl_endl;
  vcl_cout << vcl_endl;
#endif

  probability_ = (pmf.empty()) ? 0 : pmf.back().first;
  return probability_;
}


//: Calculate the error in similarity between this trasformed by \p xform
double
bmrf_node::probability(const bmrf_gamma_func_sptr& gamma)
{
  if (weight_.empty())
    this->compute_weights();

  // precompute the segment in the next and previous frames since
  // this should make up most of the neighbors
  bmrf_epi_seg_sptr prev_seg = bmrf_epi_transform(this->epi_seg(), gamma, -1.0);
  bmrf_epi_seg_sptr next_seg = bmrf_epi_transform(this->epi_seg(), gamma, 1.0);
  double prob = 0.0;
  for ( arc_iterator a_itr = this->begin(TIME); a_itr != this->end(TIME); ++a_itr ) {
    bmrf_node_sptr neighbor = (*a_itr)->to();
    int time_step = neighbor->frame_num() - this->frame_num();
    double error;
    switch(time_step) {
     case -1:
      error = bmrf_match_error(prev_seg, neighbor->epi_seg());
      break;
     case 1:
      error = bmrf_match_error(next_seg, neighbor->epi_seg());
      break;
     default:
      // compute less likely transformations as needed
      bmrf_epi_seg_sptr xform_seg = bmrf_epi_transform(this->epi_seg(), gamma, double(time_step));
      error = bmrf_match_error(xform_seg, neighbor->epi_seg());
    }
    vcl_map<bmrf_node*, double>::iterator w_itr = weight_.find(neighbor.ptr());
    prob += w_itr->second * vcl_exp(-error/2.0);
  }
  return prob * 0.398942; // 1/sqrt(2*pi)
}


//: Compute the weights of each node for use in probability computation
// Nodes are weighted by alpha overlap and intesity similarity
void
bmrf_node::compute_weights()
{
  double int_var = 0.001; // intensity variance
  double total_wgt = 0.0;
  bmrf_epi_seg_sptr ep1 = this->epi_seg();
  for ( arc_iterator a_itr = this->begin(TIME); a_itr != this->end(TIME); ++a_itr ) {
    bmrf_node_sptr neighbor = (*a_itr)->to();
    bmrf_epi_seg_sptr ep2 = neighbor->epi_seg();

    double min_alpha = MAX(ep1->min_alpha(), ep2->min_alpha());
    double max_alpha = MIN(ep1->max_alpha(), ep2->max_alpha());
    double alpha_range = max_alpha - min_alpha;
    double d_alpha =  MIN( (ep1->max_alpha() - ep1->min_alpha())/ep1->n_pts() ,
                           (ep2->max_alpha() - ep2->min_alpha())/ep2->n_pts() );

    double l_error = 0.0, r_error = 0.0;
    for (double alpha = min_alpha; alpha <= max_alpha; alpha += d_alpha)
    {
      double dli = (ep1->left_int(alpha) - ep2->left_int(alpha));
      double dri = (ep1->right_int(alpha) - ep2->right_int(alpha));
      l_error += dli*dli;
      r_error += dri*dri;
    }
    double int_error = (l_error + r_error) * d_alpha / (alpha_range * int_var);
    double wgt = alpha_range*vcl_exp(-int_error/2.0);
    weight_[neighbor.ptr()] = wgt;
    total_wgt += alpha_range;
  }
  double scale = 1.0/total_wgt;
  for ( vcl_map<bmrf_node*, double>::iterator w_itr = weight_.begin();
        w_itr != weight_.end();  ++w_itr ) {
    w_itr->second *= scale;
  }
}


//: Add \param node as a neighbor of type \param type
bool
bmrf_node::add_neighbor( bmrf_node *node, neighbor_type type )
{
  if (!node || node == this || type == ALL)
    return false;

  // verify that this arc is not already present
  for (arc_iterator itr = boundaries_[type]; itr != boundaries_[type+1]; ++itr)
    if ((*itr)->to_ == node) return false;

  // add the arc
  bmrf_arc_sptr new_arc = new bmrf_arc(this, node);
  out_arcs_.insert(boundaries_[type+1], new_arc);
  node->in_arcs_.push_back(new_arc);
  ++sizes_[type];

  // adjust boundaries if necessary
  for (int t=type; t>=0 && (boundaries_[type+1] == boundaries_[t]); --t)
    --boundaries_[t];

  return true;
}


//: Remove \param node from the neighborhood
bool
bmrf_node::remove_neighbor( bmrf_node *node, neighbor_type type )
{
  if (!node || node == this)
    return false;

  bool removed = false;
  int init_t = type;
  if (type == ALL) init_t=0;

  for (int t = init_t; t<ALL; ++t)
  {
    for (arc_iterator itr = boundaries_[t]; itr != boundaries_[t+1]; ++itr) {
      if ((*itr)->to_ == node) {
        // adjust boundaries if necessary
        for (int t2=t; t2>=0 && (boundaries_[t2] == itr); --t2)
          ++boundaries_[t2];
        arc_iterator back_itr = vcl_find((*itr)->to_->in_arcs_.begin(), (*itr)->to_->in_arcs_.end(), *itr);
        if (back_itr != (*itr)->to_->in_arcs_.end())
          (*itr)->to_->in_arcs_.erase(back_itr); // erase the pointer back from the other node
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
  for (int t=0; t<ALL; ++t)
    vsl_b_write(os, sizes_[t]);

  // write all the outgoing arcs
  for (vcl_list<bmrf_arc_sptr>::const_iterator itr = out_arcs_.begin();
       itr != out_arcs_.end(); ++itr) {
    vsl_b_write(os, *itr);  // Save the arc
  }

  // write the number of incoming arcs
  vsl_b_write(os, (unsigned int) in_arcs_.size());
  // write all the incoming arcs
  for (vcl_list<bmrf_arc_sptr>::const_iterator itr = in_arcs_.begin();
       itr != in_arcs_.end(); ++itr) {
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
      for (int t=0; t<ALL; ++t) {
        vsl_b_read(is, sizes_[t]);
        num_neighbors += sizes_[t];
      }
      int type = 0;
      int b_loc = sizes_[0];
      for (int n=0; n<num_neighbors; ++n) {
        bmrf_arc_sptr arc_ptr;
        vsl_b_read(is, arc_ptr);
        arc_ptr->from_ = this;
        out_arcs_.push_back(arc_ptr);

        while (type < ALL && n == b_loc) {
          boundaries_[++type] = out_arcs_.end();
          --boundaries_[type];
          b_loc += sizes_[type];
        }
      }
      boundaries_[0] = out_arcs_.begin();

      unsigned int num_incoming;
      vsl_b_read(is, num_incoming);
      for (unsigned int n=0; n<num_incoming; ++n) {
        bmrf_arc_sptr arc_ptr;
        vsl_b_read(is, arc_ptr);
        arc_ptr->to_ = this;
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
  os << "pr=" << probability_ << ", frame=" << frame_num_;
}


//-----------------------------------------------------------------------------------------
// bmrf_arc member functions
//-----------------------------------------------------------------------------------------


//: Binary save self to stream.
void
bmrf_node::bmrf_arc::b_write( vsl_b_ostream& ) const
{
  // Nothing to write
}


//: Binary load self from stream.
void
bmrf_node::bmrf_arc::b_read( vsl_b_istream& )
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
  if (n==0) {
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
  if (not_null_ptr) {
    n = new bmrf_node();
    n->b_read(is);
  }
  else
    n = 0;
}


//: Print an ASCII summary to the stream
void
vsl_print_summary(vcl_ostream &os, const bmrf_node* n)
{
  os << "bmrf_node{ ";
  n->print_summary(os);
  os << " }";
}


//: Binary save bmrf_node::bmrf_arc to stream.
void
vsl_b_write(vsl_b_ostream &os, const bmrf_node::bmrf_arc* a)
{
  if (a==0) {
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
  if (not_null_ptr) {
    a = new bmrf_node::bmrf_arc();
    a->b_read(is);
  }
  else
    a = 0;
}


//: Print an ASCII summary to the stream
void
vsl_print_summary(vcl_ostream &os, const bmrf_node::bmrf_arc* )
{
  os << "brmf_arc{}";
}

