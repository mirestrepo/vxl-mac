// This is mul/clsfy/clsfy_direct_boost_builder.cxx
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma implementation
#endif
//:
// \file
// \brief Functions to train classifiers using AdaBoost algorithm
// \author dac
// \date   Fri Mar  1 23:49:39 2002
//
//  Functions to train classifiers using a direct boosting algorithm
//  DirectBoost combines a set of (usually simple, weak) classifiers into
//  a more powerful single classifier.  Essentially it selects the
//  classifiers one at a time, choosing the best at each step.
//  The classifiers are trained to distinguish the examples mis-classified
//  by the currently selected classifiers.
// \verbatim
// Modifications
// \endverbatim

#include "clsfy_direct_boost_builder.h"
#include "clsfy_direct_boost.h"
#include "clsfy_builder_1d.h"

#include <vcl_iostream.h>
#include <vcl_cstdlib.h> // for vcl_abort()
#include <vcl_cmath.h>
#include <vnl/vnl_math.h>
#include <vcl_ctime.h>
#include <vcl_algorithm.h>
#include <vcl_cassert.h>
#include <vbl/vbl_triple.h>
#include <mbl/mbl_file_data_collector.h>
#include <mbl/mbl_data_collector_list.h>
#include <mbl/mbl_index_sort.h>

//=======================================================================

clsfy_direct_boost_builder::clsfy_direct_boost_builder()
: save_data_to_disk_(false), bs_(-1), max_n_clfrs_(-1), weak_builder_(0)
{
}

//=======================================================================

clsfy_direct_boost_builder::~clsfy_direct_boost_builder()
{
}


//=======================================================================

bool clsfy_direct_boost_builder::is_class(vcl_string const& s) const
{
  return s == clsfy_direct_boost_builder::is_a() || clsfy_builder_base::is_class(s);
}

//=======================================================================

vcl_string clsfy_direct_boost_builder::is_a() const
{
  return vcl_string("clsfy_direct_boost_builder");
}


//: Calc similarity between two 1d input vectors
double clsfy_direct_boost_builder::calc_prop_same(
                   const vcl_vector<bool>& vec1,
                   const vcl_vector<bool>& vec2) const

{
  unsigned n = vec1.size();
  assert( n==vec2.size() );
  int sum= 0;
  for (unsigned i=0;i<n;++i)
  {
    if (vec1[i]==vec2[i]) 
      sum++;
  }
  return sum*1.0/n;
}



//: Calc threshold for current version of strong classifier
double clsfy_direct_boost_builder::calc_threshold(
                   clsfy_direct_boost& strong_classifier,
                   mbl_data_wrapper<vnl_vector<double> >& inputs,
                   const vcl_vector<unsigned>& outputs) const
{
  // calc classification score for each example
  unsigned long n = inputs.size();
  vcl_vector<double> scores(n);
  inputs.reset();
  for (unsigned i=0;i<n;++i)
  {
    scores[i]= strong_classifier.log_l( inputs.current() );
    inputs.next();
  }

  // calc number of negative examples
  int tot_pos=0;
  for (unsigned i=0;i<n;++i)
    if ( outputs[i]== 1 ) tot_pos++;

  // then find threshold that gives min_error over training set
  vcl_vector<int> index;
  mbl_index_sort(scores, index);

  int n_pos=0;
  int n_neg=0;
  int min_error= n+1;
  double min_thresh= -1;
  for (unsigned long i=0;i<n;++i)
  {
    //vcl_cout<<"scores[ index["<<i<<"] ]= "<<scores[ index[i] ]<<" : ";
    //vcl_cout<<"outputs[ index["<<i<<"] ]= "<<outputs[ index[i] ]<<vcl_endl;
    if ( outputs[ index[i] ] == 0 ) n_neg++;
    else if ( outputs[ index[i] ] == 1 ) n_pos++;
    else 
    {
      vcl_cout<<"ERROR: clsfy_direct_boost_basic_builder::calc_threshold()"<<vcl_endl;
      vcl_cout<<"Unrecognised output value"<<vcl_endl;
      vcl_cout<<"outputs["<<index[i]<<"]="<<outputs[index[i]]<<vcl_endl;
      vcl_abort();
    }

    //vcl_cout<<"n= "<<n<<vcl_endl;
    //vcl_cout<<"n_pos= "<<n_pos<<vcl_endl;
    //vcl_cout<<"n_neg= "<<n_neg<<vcl_endl;
    int error= n_neg+(tot_pos-n_pos);
    
    if ( error<= min_error ) 
    {
      min_error= error;
      min_thresh = scores[ index[i] ] + 0.001 ;
      //vcl_cout<<"error= "<<error<<vcl_endl;
      //vcl_cout<<"min_thresh= "<<min_thresh<<vcl_endl;
    }

  }

  assert( n_pos+ n_neg== n );
  //vcl_cout<<"min_error= "<<min_error<<vcl_endl;
  //vcl_cout<<"min_thresh= "<<min_thresh<<vcl_endl;

  return min_thresh;

}



//: Build classifier composed of 1d classifiers working on individual vector elements
// Builds an n-component classifier, each component of which is a 1D classifier
// working on a single element of the input vector.
double clsfy_direct_boost_builder::build(clsfy_classifier_base& model,
                       mbl_data_wrapper<vnl_vector<double> >& inputs,
                       unsigned nClasses,
                       const vcl_vector<unsigned> &outputs) const
{
  // nb  ignore nClasses=1, ie always binary classifier

  assert( model.is_class("clsfy_direct_boost") );
  clsfy_direct_boost &strong_classifier = (clsfy_direct_boost&) model;


  // check parameters are OK
  if ( max_n_clfrs_ < 0 )
  {
    vcl_cout<<"Error: clsfy_direct_boost_builder::build"<<vcl_endl;
    vcl_cout<<"max_n_clfrs_ = "<<max_n_clfrs_<<" ie < 0 "<<vcl_endl;
    vcl_cout<<"set using set_max_n_clfrs()"<<vcl_endl;
    vcl_abort();
  }
  else
  {
    vcl_cout<<"Maximum number of classifiers to be found by Adaboost ="
        <<max_n_clfrs_<<vcl_endl;
  }

  if ( weak_builder_ == 0 )
  {
    vcl_cout<<"Error: clsfy_direct_boost_builder::build"<<vcl_endl;
    vcl_cout<<"weak_builder_ pointer has not been set "<<vcl_endl;
    vcl_cout<<"need to provide a builder to build each weak classifier"<<vcl_endl;
    vcl_cout<<"set using set_weak_builder()"<<vcl_endl;
    vcl_abort();
  }
  else
  {
    vcl_cout<<"Weak learner used by AdaBoost ="
        <<weak_builder_->is_a()<<vcl_endl;
  }

  if ( bs_ < 0 )
  {
    vcl_cout<<"Error: clsfy_direct_boost_builder::build"<<vcl_endl;
    vcl_cout<<"bs_ = "<<bs_<<" ie < 0 "<<vcl_endl;
    vcl_cout<<"set using set_batch_size()"<<vcl_endl;
    vcl_abort();
  }
  else
  {
    vcl_cout<<"Batch size when sorting data =" <<bs_<<vcl_endl;
  }


  assert(bs_>0);
  assert(bs_!=1);
  assert (max_n_clfrs_ >= 0);

  // first arrange the data in the form
  // vcl_vector< < vcl_vector< vtl_triple<double,int,int> > > > data
  // + vnl_vector wts
  // then sort all data once, then build the classifier

  // number of examples
  unsigned n= inputs.size();
  //vcl_cout<<"n= "<<n<<vcl_endl;

  // Dimensionality of data
  inputs.reset();
  int d = inputs.current().size();

  //need file data wrapper instead of old vector
  //data stored on disk NOT ram
  //vcl_vector< vcl_vector<vbl_triple<double,int,int> > > data(d);

  vcl_string temp_path= "temp.dat";
  mbl_file_data_collector< vnl_vector<double> >
              file_collector( temp_path );

  mbl_data_collector_list< vnl_vector< double > >
              ram_collector;

  mbl_data_collector<vnl_vector< double> >*  collector;

  if (save_data_to_disk_)
  {
    vcl_cout<<"saving data to disk! "<<vcl_endl;
    collector= &file_collector;
  }
  else
  {
    //bs_ = n ;
    vcl_cout<<"saving data to ram! "<<vcl_endl;
    collector= &ram_collector;
  }

  
  // say load in and sort 100 at once?????
  // ie 100 features at once!

  //int bs= 100; //batch size
  vcl_vector< vnl_vector< double > >vec(bs_);
  
  vcl_cout<<"d= "<<d<<vcl_endl;
  int b=0;
  while ( b+1<d )
  {
    int r= vcl_min ( bs_, (d-b) );
    assert(r>0);

    vcl_cout<<"arranging weak classifier data = "<<b<<" to "
            <<(b+r)-1<<" of "<<d<<vcl_endl;

    // have to resize all vectors
    for (int i=0; i< bs_; ++i)
      vec[i].resize(n);

    // add data for both classes
    inputs.reset();
    for (unsigned int j=0;j<n;++j)
    {
      for (int i=0; i< r; ++i)
        vec[i](j)=( inputs.current()[b+i] );
      inputs.next();
    }


    for (int i=0; i< r; ++i)
    {
      // sort training data for each individual weak classifier
      assert (vec[i].size() == n);
      assert (n != 0);

      // store sorted vector of responses for each individual weak classifier
      collector->record(vec[i]);
    }

    b+=bs_;
  }


  mbl_data_wrapper< vnl_vector<double> >&
              wrapper=collector->data_wrapper();


  // now actually apply direct boost algorithm
  wrapper.reset();
  assert ( wrapper.current().size() == n );
  assert ( d == (int)wrapper.size() );

 
  // nb have to set builder as a member variable elsewhere
  clsfy_classifier_1d* c1d = weak_builder_->new_classifier();
 
  // wts not really used!!
  vnl_vector<double> wts(n,1.0/n);

  // need to train each weak classifier on the data 
  // and record the error and output responses of the weak classifiers
  vcl_vector< double > errors(0);
  vcl_vector< vcl_vector<bool> > responses(0);
  vcl_vector< clsfy_classifier_1d* > classifiers(0);

  wrapper.reset();
  for (int i=0; i<d; ++i )
  {
    const vnl_vector<double>& vec= wrapper.current();
    double error= weak_builder_->build(*c1d, vec, wts, outputs);
    
    vcl_vector<bool> resp_vec(n);
    // now get responses
    for (int k=0; k<n;++k)
    {
      unsigned int r= c1d->classify( vec(k) );
      if (r==0)
        resp_vec[k]=false;
      else
        resp_vec[k]=true;

    }

    responses.push_back( resp_vec );
    errors.push_back( error );
    classifiers.push_back( c1d->clone() );

    wrapper.next();

  }

  delete c1d;


  // now use the outputs and errors to define a strong classifier

  // create a sorted index of the errors
  vcl_vector<int> index;
  mbl_index_sort(errors, index);


  // need to pick best classifier and store index + mean & variance
  // whilst list of indices is NOT empty
  strong_classifier.clear();
  strong_classifier.set_n_dims(d);

  

  for (int k=0; k<max_n_clfrs_; ++k)
  {

    if (index.size() == 0 ) break;

    // store best classifier that is left in list
    int ind= index[0];
    vcl_cout<<"ind= "<<ind<<vcl_endl;
    vcl_cout<<"errors["<<ind<<"]= "<<errors[ind]<<vcl_endl;
    if (errors[ind]> 0.5 ) break;


    if (errors[ind]==0)
      strong_classifier.add_one_classifier( classifiers[ind], 1.0, ind);
    else
      strong_classifier.add_one_classifier( classifiers[ind], 1.0/errors[ind], ind);
   

    if (calc_all_thresholds_)
    {
      // calculating response from classifier so far
      // and using this to calc min_error threshold
      double t=calc_threshold( strong_classifier, inputs, outputs );
      strong_classifier.add_one_threshold(t);
    }
    else
    {
      // add dummy threshold, ie only calc properly at end
      strong_classifier.add_one_threshold(0.0);
    }

    if (errors[ind]==0) break;

    // find all classifiers that are similar to the selected 
    // classifier i
    vcl_vector<int> new_index(0);
    vcl_vector<bool>& i_vec=responses[ind];
    int n=index.size();
    int n_rejects=0;
    for (int j=0; j<n; ++j)
    {

      vcl_vector<bool>& j_vec=responses[ index[j] ];
      double prop_same= calc_prop_same(i_vec,j_vec);
      //vcl_cout<<"prop_same= "<<prop_same<<vcl_endl;
      //vcl_cout<<"prop_= "<<prop_<<vcl_endl;
      if( prop_same < prop_ )
        new_index.push_back( index[j] );
      else
        n_rejects++;
    }

    vcl_cout<<"number of rejects due to similarity= "<<n_rejects<<vcl_endl;
      
    //for (int p=0; p<new_index.size(); ++p)
    //  vcl_cout<<"new_index["<<p<<"]= "<<new_index[p]<<vcl_endl;

    index= new_index;


    //for (int p=0; p<index.size(); ++p)
    //  vcl_cout<<"index["<<p<<"]= "<<index[p]<<vcl_endl;
  }



  // calculating response from classifier so far
  // and using this to calc min_error threshold
  double t=calc_threshold( strong_classifier, inputs, outputs );
  strong_classifier.add_final_threshold(t);

  // does clsfy_test_error balk if have too much data?
  // should be OK because just passes mbl_data_wrapper and evaluates
  // one at a time, so if using mbl_file_data_wrapper should be OK!
  vcl_cout<<"calculating training error"<<vcl_endl;
  return clsfy_test_error(strong_classifier, inputs, outputs);


}


//: Create empty classifier
// Caller is responsible for deletion
clsfy_classifier_base* clsfy_direct_boost_builder::new_classifier() const
{
  return new clsfy_direct_boost();
}


//=======================================================================

#if 0
    // required if data stored on the heap is present in this derived class
clsfy_direct_boost_builder::clsfy_direct_boost_builder(const clsfy_direct_boost_builder& new_b):
  data_ptr_(0)
{
  *this = new_b;
}


//=======================================================================

    // required if data stored on the heap is present in this derived class
clsfy_direct_boost_builder& clsfy_direct_boost_builder::operator=(const clsfy_direct_boost_builder& new_b)
{
  if (&new_b==this) return *this;

  // Copy heap member variables.
  delete data_ptr_; data_ptr_=0;

  if (new_b.data_ptr_)
    data_ptr_ = new_b.data_ptr_->clone();

  // Copy normal member variables
  data_ = new_b.data_;

  return *this;
}
#endif // 0

//=======================================================================

clsfy_builder_base* clsfy_direct_boost_builder::clone() const
{
  return new clsfy_direct_boost_builder(*this);
}

//=======================================================================

    // required if data is present in this base class
void clsfy_direct_boost_builder::print_summary(vcl_ostream& os) const
{
  // clsfy_builder_base::print_summary(os); // Uncomment this line if it has one.
  // vsl_print_summary(os, data_); // Example of data output

  vcl_cerr << "clsfy_direct_boost_builder::print_summary() NYI" << vcl_endl;
}

//=======================================================================

  // required if data is present in this base class
void clsfy_direct_boost_builder::b_write(vsl_b_ostream& bfs) const
{
  //vsl_b_write(bfs, version_no());
  //clsfy_builder_base::b_write(bfs);  // Needed if base has any data
  //vsl_b_write(bfs, data_);
  vcl_cerr << "clsfy_direct_boost_builder::b_write() NYI" << vcl_endl;
}

//=======================================================================

  // required if data is present in this base class
void clsfy_direct_boost_builder::b_read(vsl_b_istream& bfs)
{
  vcl_cerr << "clsfy_direct_boost_builder::b_read() NYI" << vcl_endl;
#if 0
  if (!bfs) return;

  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
  case (1):
    //clsfy_builder_base::b_read(bfs);  // Needed if base has any data
    vsl_b_read(bfs,data_);
    break;
  default:
    vcl_cerr << "I/O ERROR: vsl_b_read(vsl_b_istream&, clsfy_direct_boost_builder&) \n";
    vcl_cerr << "           Unknown version number "<< version << "\n";
    bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
    return;
  }
#endif // 0
}
