#ifdef __GNUC__
#pragma implementation
#endif

// This is vxl/vbl/io/vbl_io_smart_ptr.txx

//:
// \file
// \brief Serialised binary IO functions for vbl_smart_ptr<T>
// \author Ian Scott (Manchester)
// \date 26-Mar-2001

#include <vsl/vsl_binary_io.h>
#include <vbl/vbl_smart_ptr.h>


//=========================================================================
//: Binary save self to stream.
template<class T>
void vsl_b_write(vsl_b_ostream & os, const vbl_smart_ptr<T> &p)
{

  // write version number
  const short io_version_no = 1;
  vsl_b_write(os, io_version_no);
  vsl_b_write(os, p.is_protected());


  // Get a serial_number for object being pointed to
  unsigned long id = os.get_serial_number(p.ptr());
  // Find out if this is the first time the object being pointed to is 
  // being saved
  if (id == 0)
  {
    // <rant> IMS
    // It is not clear how to deal fully satisfactorily with unprotected 
    // smart_ptrs. For example is we save and reload them without any error 
    // checks we could restore the object with a reference count of 0.
    // To be honest, I think the idea of an unprotected smart_ptr is 
    // not so smart. Either it is a smart pointer, in which case it should 
    // be protected, or it is unprotected in which case you should use a 
    // ordinary pointer. Cycles in the pointer network, are best dealt with
    // by avoiding them. You have to be aware they are happening to unprotect 
    // the pointer anyway.
    // <\rant> 
    if (!p.is_protected())
    {
        vcl_cerr << "vsl_b_write(vsl_b_ostream & os, const vbl_smart_ptr<T>:"
                    << " You cannot\n"
                    << "save unprotected smart pointers before saving "
                    << "a protected smart pointer\n"
                    << "to the same object. Either do not save unprotected "
                    << "smart pointers, or\n"
                    << "be very careful about the order." <<vcl_endl;
        vcl_abort();
    }

    id = os.add_serialisation_record(p.ptr());

      // Say that this is the first time
      // that this object is being saved.
      // This isn't really necessary but
      // it is useful as an error check
    vsl_b_write(os, true); 
    vsl_b_write(os, id);     // Save the serial number
// If you get an error in the next line, it could be because your type T
// has no vsl_b_write(vsl_b_ostream &,const T*)  defined on it.
// See the documentation in the .h file to see how to add it.
    vsl_b_write(os, p.ptr());    // Only save the actual object if it 
                                  //hasn't been saved before to this stream
  }
  else
  {
      // Say that this is not the first time
      // that this object is being saved.
      // This isn't really necessary but
      // it is useful as an error check

    vsl_b_write(os, false);
    vsl_b_write(os, id);         // Save the serial number
  }
  
}

//=====================================================================
//: Binary load self from stream.
template<class T>
void vsl_b_read(vsl_b_istream &is, vbl_smart_ptr<T> &p)
{
  short ver;
  vsl_b_read(is, ver);
  switch(ver)
  {
  case 1:
    {

      bool is_protected; // true if the smart_ptr is to be 
                         //responsible for the object
      vsl_b_read(is, is_protected);

      bool first_time; // true if the object is about to be loaded
      vsl_b_read(is, first_time);

      assert (!first_time || is_protected); // This should have been 
                                            //checked during saving

      unsigned long id; // Unique serial number indentifying object
      vsl_b_read(is, id);

      T * pointer = (T *) is.get_serialisation_pointer(id);
      assert(first_time == (pointer == 0));
      if (pointer == 0)
      {
// If you get an error in the next line, it could be because your type T
// has no vsl_b_read(vsl_b_ostream&,T*&)  defined on it.
// See the documentation in the .h file to see how to add it.
        vsl_b_read(is, pointer);
        is.add_serialisation_record(id, pointer);
      }
           
      p = pointer; // This operator method will set the internal 
                   //pointer in vbl_smart_ptr.
      if (!is_protected)
        p.unprotect();
    }
    break;
  default:

    vcl_cerr << "ERROR: vsl_b_read(s, vbl_smart_ptr&): Unknown version number "<< 
    ver << vcl_endl;
    abort();
  }
}

//=====================================================================
//: Output a human readable summary to the stream
template<class T>
void vsl_print_summary(vcl_ostream & os,const vbl_smart_ptr<T> & p)
{
  if (p.is_protected())
    os <<"Unprotected ";
  os << "Smart ptr to ";
  if (p.ptr())
  {

// If you get an error in the next line, it could be because your type T
// has no vsl_print_summary(vsl_b_ostream &, const T*)  defined on it.
// See the documentation in the .h file to see how to add it.
    vsl_print_summary(os, (p.ptr()));
  }
  else
    os << "NULL";
}


/*
//===========================================
// Deal with base class pointers
template<class T>
void vsl_b_read(vsl_b_istream& is, vbl_smart_ptr<T> * &p)
{
  delete p;
  bool not_null_ptr;
  vsl_b_read(is, not_null_ptr);
  if (not_null_ptr)
  {
    p = new vbl_smart_ptr<T>;
    vsl_b_read(is, *p);
  }
  else 
    p = 0;
} 

template<class T>
void vsl_b_write(vsl_b_ostream& os, const vbl_smart_ptr<T> *p)
{
  if (p==0)
  {
    vsl_b_write(os, false); // Indicate null pointer stored 
  }
  else
  {
    vsl_b_write(os,true); // Indicate non-null pointer stored 
    vsl_b_write(os,*p);
  }
}

template<class T>
void vsl_print_summary(vcl_ostream, const vbl_smart_ptr<T> *p)
{
  if (p==0)
    os << "NULL PTR";
  else
  {
    os << "vbl_smart_ptr: "; 
    vsl_print_summary(*p);
  } 
};
*/

#undef VBL_IO_SMART_PTR_INSTANTIATE
#define VBL_IO_SMART_PTR_INSTANTIATE(T) \
template void vsl_print_summary(vcl_ostream &, const vbl_smart_ptr<T > &); \
template void vsl_b_read(vsl_b_istream &, vbl_smart_ptr<T > &); \
template void vsl_b_write(vsl_b_ostream &, const vbl_smart_ptr<T > &); \

