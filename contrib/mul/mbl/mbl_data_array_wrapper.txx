#ifndef mbl_data_array_wrapper_txx_
#define mbl_data_array_wrapper_txx_

#include <mbl/mbl_data_array_wrapper.h>
#include <vcl_iostream.h>
#include <vcl_cstdlib.h>
#include <vcl_cassert.h>

//: Default constructor
template<class T>
mbl_data_array_wrapper<T>::mbl_data_array_wrapper()
  : data_(0),n_(0),index_(0)
{
}

//: Constructor
template<class T>
mbl_data_array_wrapper<T>::mbl_data_array_wrapper(const T* data, unsigned long n)
{
  set(data,n);
}

//: Constructor
// Sets up object to wrap a vcl_vector.
// The data must be kept in scope, this does not take a copy.
template<class T>
mbl_data_array_wrapper<T>::mbl_data_array_wrapper(const vcl_vector<T > &data)
{
  // There is nothing in the STL standard that says that vector<> has
  // to store its data in a contiguous memory block. However, most
  // implementations do store data this way.
  // Check this assumption holds.
  assert(data.size() == 0|| &data[data.size() - 1] - &data[0] + 1 == data.size());
  set(&data[0], data.size());
}



//: Initialise to return elements from data[i]
template<class T>
void mbl_data_array_wrapper<T>::set(const T* data, unsigned long n)
{
  assert (n != ((unsigned)-1)); // a common mistake
  data_    = data;
  n_       = n;
  index_ = 0;
}

//: Default destructor
template<class T>
mbl_data_array_wrapper<T>::~mbl_data_array_wrapper()
{
}

//: Number of objects available
template<class T>
unsigned long mbl_data_array_wrapper<T>::size() const
{
  return n_;
}

//: Reset so that current() returns first object
template<class T>
void mbl_data_array_wrapper<T>::reset()
{
  index_=0;
}

//: Return current object
template<class T>
const T& mbl_data_array_wrapper<T>::current()
{
  return data_[index_];
}

//: Move to next object, returning true if is valid
template<class T>
bool mbl_data_array_wrapper<T>::next()
{
  index_++;
  return (index_<n_);
}

//: Return current index
template<class T>
unsigned long mbl_data_array_wrapper<T>::index() const
{
  return index_;
}


//: Create copy on heap and return base pointer
template<class T>
mbl_data_wrapper< T >* mbl_data_array_wrapper<T>::clone() const
{
  return new mbl_data_array_wrapper<T>(*this);
}

//: Move to element n
//  First example has index 0
template<class T>
void mbl_data_array_wrapper<T>::set_index(unsigned long n)
{
  assert(n != ((unsigned)-1));
  if (n>=n_)
  {
    vcl_cerr<<"TC_VectorDataBase::set_index(n) ";
    vcl_cerr<<"n = "<<n<<" not in range 0<=n<"<<size()<<vcl_endl;
    vcl_abort();
  }

  index_=n;
}

template <class T>
vcl_string mbl_data_array_wrapper<T>::is_a() const
{
  return vcl_string("mbl_data_array_wrapper<T>");
}


#define MBL_DATA_ARRAY_WRAPPER_INSTANTIATE(T) \
template class mbl_data_array_wrapper< T >

#endif // mbl_data_array_wrapper_txx_
