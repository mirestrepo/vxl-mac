#ifndef mbl_exception_h_
#define mbl_exception_h_

#include <vcl_string.h>
#if VCL_HAS_EXCEPTIONS
# include <vcl_stdexcept.h>
#endif

//: Throw an exception indicating a real problem.
// If exceptions have been disabled, this function
// may abort.
template <class T>
void mbl_exception_error(T exception)
{
  vcl_cerr << "ERROR: " << exception.what() << vcl_endl;
#if !defined MBL_EXCEPTIONS_DISABLE  && VCL_HAS_EXCEPTIONS
  throw exception;
#else
  vcl_abort();
#endif
}

//: Throw an exception indicating a potential problem.
// If exceptions have been disabled, this function
// may return.
template <class T>
void mbl_exception_warning(T exception)
{
vcl_cerr << "WARNING: " << exception.what() << vcl_endl;
#if !defined MBL_EXCEPTIONS_DISABLE  && VCL_HAS_EXCEPTIONS
  throw exception;
#endif
}

#if !VCL_HAS_EXCEPTIONS

  //: Indicates that mbl_cloneables_factory has not heard of value name.
  class mbl_exception_no_name_in_factory
  {
    vcl_string msg;
  public:
    mbl_exception_no_name_in_factory(const vcl_string&  msg): msg_(msg) {}
    const char * what() const {return msg.c_str();}
  };

#else 
;
  //: Indicates that mbl_cloneables_factory has not heard of value name.
  class mbl_exception_no_name_in_factory : public vcl_logic_error
  {
  public:
    vcl_string failed_value, valid_values;
    mbl_exception_no_name_in_factory(const vcl_string& failed_name, const vcl_string& valid_names):
      vcl_logic_error(vcl_string("No such value: ") +failed_name + "\nValid values are: ["+valid_names+"]"),
        failed_value(failed_name), valid_values(valid_names) {}
    virtual ~mbl_exception_no_name_in_factory() throw() {};
  };

#endif



#endif // mbl_exception_h_

