#ifndef vbl_shared_pointer_h_
#define vbl_shared_pointer_h_
#ifdef __GNUC__
#pragma interface
#endif
/*
  fsm@robots.ox.ac.uk
*/

//:
// \file
// \brief Non-intrusive smart pointers

#include <vcl_compiler.h>

#define vbl_shared_pointer_zero(var) (var) = 0

//: Non-intrusive smart pointers
template </*typename*/class T>
struct vbl_shared_pointer
{
  typedef T element_type;
  typedef vbl_shared_pointer<T> self;

  struct data_t
  {
    T *pointer;    // pointer to object.
    int use_count; // number of shared_pointers using object.
    data_t(T *p, int u) : pointer(p), use_count(u) { }
  };

  vbl_shared_pointer() : data(0) { }

  explicit
  vbl_shared_pointer(T *p) {
    if (p)
      data = new data_t(p, 1);
    else
      data = 0;
  }

  vbl_shared_pointer(self const &that) {
    data = that.data;
    up_ref();
  }

#if VCL_HAS_MEMBER_TEMPLATES
  // if T has a constructor T::T(3, "foo") then it's nice
  // to be able to say
  //   vbl_shared_pointer<T> sp(3, "foo");
  // instead of
  //   vbl_shared_pointer<T> sp = new T(3, "foo");
  template <typename V1>
  explicit vbl_shared_pointer(V1 const &v1)
    : data(new data_t(new T(v1), 1)) { }

  template <typename V1, typename V2>
  explicit vbl_shared_pointer(V1 const &v1, V2 const &v2)
    : data(new data_t(new T(v1, v2), 1)) { }

  template <typename V1, typename V2, typename V3>
  explicit vbl_shared_pointer(V1 const &v1, V2 const &v2, V3 const &v3)
    : data(new data_t(new T(v1, v2, v3), 1)) { }

  template <typename V1, typename V2, typename V3, typename V4>
  explicit vbl_shared_pointer(V1 const &v1, V2 const &v2, V3 const &v3, V4 const &v4)
    : data(new data_t(new T(v1, v2, v3, v4), 1)) { }
#endif

  self &operator=(self const &that) {
    that.up_ref();
    down_ref();
    data = that.data;
    return *this;
  }

  ~vbl_shared_pointer() {
    down_ref();
    vbl_shared_pointer_zero(data);
  }

  // conversion to bool
  operator bool() const { return data != 0; }

  // conversion to pointer
  T const *operator->() const { return as_pointer(); }
  T       *operator->() { return as_pointer(); }

  // conversion to T
  T const &operator*() const { return *as_pointer(); }
  T       &operator*() { return *as_pointer(); }

  // relational
  bool operator==(self const &that) const { return data == that.data; }
  bool operator< (self const &that) const { return data <  that.data; }

  // use these if you like, but at your own risk.
  T *as_pointer() const {
    return data ? data->pointer : 0;
  }
  void up_ref() const {
    if (data)
      ++ data->use_count;
  }
  void down_ref() const {
    if (data && (-- data->use_count == 0)) {
      delete data->pointer;
      delete data;
    }
  }

private:
  data_t *data;
};

#endif // vbl_shared_pointer_h_
