// This is mul/clsfy/clsfy_adaboost_sorted_trainer2.h
#ifndef clsfy_adaboost_sorted_trainer2_h_
#define clsfy_adaboost_sorted_trainer2_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \brief Functions to train classifiers using AdaBoost algorithm
// \author dac
// \date   Fri Mar  1 23:49:39 2002
//  Functions to train classifiers using AdaBoost algorithm
//  AdaBoost combines a set of (usually simple, weak) classifiers into
//  a more powerful single classifier.  Essentially it selects the
//  classifiers one at a time, choosing the best at each step.
//  The classifiers are trained to distinguish the examples mis-classified
//  by the currently selected classifiers.

#include <vcl_string.h>
#include <vsl/vsl_binary_io.h>
#include <clsfy/clsfy_simple_adaboost.h>
#include <clsfy/clsfy_binary_threshold_1d_sorted_builder.h>
#include <mbl/mbl_data_wrapper.h>

//=======================================================================

//: A class for some purpose.
// The purpose of this class is to prove that 1+1=3.
class clsfy_adaboost_sorted_trainer2
{
 public:

  //: Dflt ctor
  clsfy_adaboost_sorted_trainer2();

  //: Destructor
  ~clsfy_adaboost_sorted_trainer2();

  //: Build classifier composed of 1d classifiers working on individual vector elements
  //  Builds an n-component classifier, each component of which is a 1D classifier
  //  working on a single element of the input vector.
  void build_strong_classifier(clsfy_simple_adaboost& strong_classifier,
                               int max_n_clfrs,
                               clsfy_binary_threshold_1d_sorted_builder& builder,
                               mbl_data_wrapper<vnl_vector<double> >& egs0,
                               mbl_data_wrapper<vnl_vector<double> >& egs1,
                               int bs);

  void apply_adaboost(
                            clsfy_simple_adaboost& strong_classifier,
                            int max_n_clfrs,
                            clsfy_binary_threshold_1d_sorted_builder& builder,
                            mbl_data_wrapper< 
                          vcl_vector< vbl_triple<double,int,int> >                
                                            >& wrapper,
                                            int n0, int n1);

  //: Version number for I/O
  short version_no() const;

  //: Name of the class
  vcl_string is_a() const;

  //: Print class to os
  void print_summary(vcl_ostream& os) const;

  //: Save class to binary file stream.
  void b_write(vsl_b_ostream& bfs) const;

  //: Load class from binary file stream
  void b_read(vsl_b_istream& bfs);

 protected:
#if 0
  // This is required if there are any references to objects
  // created on the heap. A deep copy should be made of anything
  // referred to by pointer during construction by copy. The copy
  // constructor is protected to stop its use for class
  // instantiation. It should be implemented in terms of the
  // assignment operator.

  //: Copy constructor
  clsfy_adaboost_sorted_trainer2( const clsfy_adaboost_sorted_trainer2& b );

  //: Assignment operator
  clsfy_adaboost_sorted_trainer2& operator=( const clsfy_adaboost_sorted_trainer2& b );
#endif
};

//=======================================================================

//: Binary file stream output operator for class reference
void vsl_b_write(vsl_b_ostream& bfs, const clsfy_adaboost_sorted_trainer2& b);

//: Binary file stream input operator for class reference
void vsl_b_read(vsl_b_istream& bfs, clsfy_adaboost_sorted_trainer2& b);

//: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const clsfy_adaboost_sorted_trainer2& b);

//: Stream output operator for class reference
void vsl_print_summary(vcl_ostream& os,const clsfy_adaboost_sorted_trainer2& b);

#endif // clsfy_adaboost_sorted_trainer2_h_
