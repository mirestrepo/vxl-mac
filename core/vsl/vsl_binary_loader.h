#ifndef vsl_binary_loader_h_
#define vsl_binary_loader_h_
#ifdef __GNUC__
#pragma interface
#endif

#include <vsl/vsl_binary_loader_base.h>
#include <vsl/vsl_binary_io.h>
#include <vcl_vector.h>
#include <vcl_string.h>

//: Class to load objects by baseclass pointer.
// An example of a singleton design pattern for loading
// a DerivedClass from a stream into a BaseClass*.
// All we are given is a BaseClass* into which
// the object has to be loaded but we can only tell
// what sort of object it is from the is_a() information
// stored in the stream. To handle this we define a loader
// which has a list of BaseClass pointers,
// and the ChainOfResponsibility (Design Pattern)
// approach is used to load the object i.e. each
// ptr->is_a() is matched against the string on the stream
// until we find a match or run out of pointers. If
// a pointer is found which matches the string on
// the stream, we clone it, use the clone to load the object
// from the stream, then return a pointer to the clone.
//
// We use a singleton so that there is only one list of
// concrete derived classes which can be added
// to for loading purposes. If you derive a new
// class you just have to append it to the list of
// classes of the singletion, viz:
//     vsl_binary_loader::instance().append(my_object)
// The BaseClass MUST implement is_a(),clone(),b_write() and b_read()
// virtual functions.  The b_write function MUST save is_a()
// as an string "is<<vcl_string(is_a());" first.
//
// To indicate a null pointer (0), save the string "VSL_NULL_PTR"
// to the stream.
//
// For instance:
// \verbatim
// inline void vsl_b_write(vsl_b_ostream& os, const base_class* base_ptr)
// {
//   if (base_ptr!=0)
//   {
//      vsl_b_write(os,base_ptr->is_a());
//      vsl_b_write(os,*base_ptr);
//   }
//   else vsl_b_write(os,string("VSL_NULL_PTR"));
// }
// \endverbatim
//
// See the Example section below.
// All loader singletons can be deleted using vsl_delete_all_loaders()
template<class BaseClass>
class vsl_binary_loader : public vsl_binary_loader_base {

private :

    //: the singleton object
  static vsl_binary_loader<BaseClass>* instance_;

    //: List of concrete classes that this loader can deal with
  vcl_vector<BaseClass*> object_;

public :
    //: Constructor
  vsl_binary_loader() {};

    //: Destructor
  ~vsl_binary_loader();

    //: Returns the instance variable for the singleton.
  static vsl_binary_loader<BaseClass>& instance();

    //: Remove all example objects
  void make_empty();

    //: Add example object to list of those that can be loaded
  void add( const BaseClass& b) { object_.push_back(b.clone());};

    //: Return current list of class objects which can be loaded
  const vcl_vector<BaseClass*>& object() { return object_; }

    //: Loads object and sets base class pointer
    //  Determines which derived class object on is belongs
    //  to, loads it and sets b to be a pointer to it.
    //  (Class must be one given to Loader by the add method).
    //  If is indicates a NULL pointer, b will be set to NULL.
    //  If b not initially NULL, *b will be deleted.
  void load_object( vsl_b_istream& is, BaseClass*& b);
};


//: Loads object and sets base class pointer
//  Determines which derived class object on bfs belongs
//  to, loads it and sets b to be a pointer to it.
//  (Class must be one given to Loader by the append method).
//  If bfs indicates a NULL pointer, b will be set to NULL.
//  If b not initially NULL, *b will be deleted.
template<class BaseClass>
inline void vsl_b_read( vsl_b_istream& bfs, BaseClass*& b)
{
  vsl_binary_loader<BaseClass>::instance().load_object(bfs,b);
}


#endif // vsl_binary_loader_h_
