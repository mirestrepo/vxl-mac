#ifndef vsl_binary_io_h_
#define vsl_binary_io_h_

//:
// \file
// \brief Set of functions, and objects to perform binary IO
// \author Ian Scott, Tim Cootes (Manchester) March 2001
//
// You should include this file if you want to do binary_io
// 
// Also included are a set of functions
// vsl_print_summary(vcl_ostream& os, bool b)
// for basic types to ensure that templated classes
// vsl_print_summaries can work with all types


#include <vcl_iosfwd.h>
#include <vcl_string.h>
#include <vcl_cassert.h>
#include <vcl_fstream.h>
#include <vcl_map.h>
#include <vcl_utility.h>
#include <vcl_cstdlib.h>

//: A binary output adaptor for any vcl_ostream 
// Currently the main use of this is to encourage streams to be opened
// in binary mode (ie. withour CR/LF conversion)
//
// These classes also provide basic support for serialisation. This allows an
// object which has multiple pointers to it to be saved only once. During
// reloading, the pointers can be all set up again to point to the single
// object. vsl_b_ostream does not do the serialisation itself, but instead
// keeps records of unique identifiers to allow the user's code to perform
// serialisation safely. For instance, a smart pointer type object will have
// to know how to safely save whatever it is pointing to.
class vsl_b_ostream
{
public:


  //: Create this adaptor using an existing stream
  // The stream (os) must be open (i.e. ready to receive insertions)
  // so that the
  // IO version and magic number can be written by this constructor.
  // User is responsible for deleting os after deleting the adaptor
  vsl_b_ostream(vcl_ostream *os);

  //: A reference to the adaptor's stream
  vcl_ostream& os() const;

  //: Virtual destructor.
  virtual ~vsl_b_ostream() {}

  //: Returns true if the underlying stream has its fail bit set.
  bool operator!() const ; 

  //: Clear the stream's record of any serialisation operations
  // Calling this function while outputing serialisable things to stream,
  // will mean 
  // that a second copy of an object may get stored to the stream.
  virtual void clear_serialisation_records();

  //: Adds an object pointer to the serialisation records.
  // Returns a unique identifier for the object.
  // You can optionally add some user-defined integer with each record
  // If error checking is on, and the object pointer is already in the records,
  // this function will abort()
  virtual unsigned long add_serialisation_record(void *pointer,
                                                  int other_data = 0);

  //: Returns a unique identifier for the object.
  // Returns 0 if there is no record of the object.
  virtual unsigned long get_serial_number(void *pointer) const;

  //: Set the user-defined data associated with the object
  // If there is no record of the object, this function will return 0.
  // However a retval of 0 does not necessarily imply that the object is
  // unrecorded.
  virtual int get_serialisation_other_data(void *pointer) const;

  //: Modify the user-defined data associated with the object
  // If there is no record of the object, this function will abort.
  virtual int set_serialisation_other_data(void *pointer, int other_data);

protected:
  //: The member stream
  vcl_ostream *os_;

  // Design notes: IMS
  // I used to think that a pointer and class name were needed to identify an
  // object. This is true if class your_class{my_class A}; your_class B;
  // then &B = &(B.A).
  // However this case doesn't arise in serialisation situations, because you
  // can't have shared ownership of A.

  // I could have used the pointer itself as the unique identifier, but it is
  // unreasonable to expect this to work cross-platform when the platforms have
  // different pointer sizes.
  
  //: The type of the serialisation records
  typedef vcl_map<void *, vcl_pair<unsigned long, int>, vcl_less<void *> >
    serialisation_records_type; 

  //: The serialisation records
  // Records a pointer, a unique identifier, and an integer
  // (user_defined data.)
  serialisation_records_type serialisation_records_;

  //: The version number of the IO scheme.
  static const unsigned short version_no_;
};


//: An adapter for a vcl_ofstream to make it suitable for binary IO
class vsl_b_ofstream: public vsl_b_ostream
{
public:
  //: Create this adaptor from a file.
  // The adapter will delete the internal stream automatically on destruction.
  vsl_b_ofstream(const vcl_string &filename,
    vcl_ios_openmode mode = vcl_ios_out | vcl_ios_trunc):
    vsl_b_ostream(new vcl_ofstream(filename.c_str(),
      mode | vcl_ios_binary)) {}

  //: Create this adaptor from a file.
  // The adapter will delete the internal stream automatically on destruction.
  vsl_b_ofstream(const char *filename,
    vcl_ios_openmode mode = vcl_ios_out | vcl_ios_trunc):
    vsl_b_ostream(new vcl_ofstream(filename, mode | vcl_ios_binary)) {}

  //: Virtual destructor.
  virtual ~vsl_b_ofstream();


  //: Close the stream
  void close();
};



//: An adaptor for any vcl_istream to make it suitable for binary input
// Currently the main use of this is to encourage file streams to be opened
// in binary mode (ie. withour CR/LF conversion)
// In future, if someone wants to add serialisation support, they
// can modify these classes to do that.
class vsl_b_istream
{
public:
  //: Create this adaptor using an existing stream.
  // The stream (is) must be open (i.e. ready to be read from) so that the
  // IO version and magic number can be read by this constructor.
  // User is responsible for deleting is after deleting the adaptor
  vsl_b_istream(vcl_istream *is);

  //: A reference to the adaptor's stream
  vcl_istream & is() const;

  //: Virtual destructor.so that it can be overloaded
  virtual ~vsl_b_istream() {}

  //: Returns true if the underlying stream has its fail bit set.
  bool operator!() const;

  //: Clear the stream's record of any serialisation operations
  // Calling this function while inputting serialisable things from a stream,
  // could cause errors during loading unless the records were cleared at a
  // similar point during output.
  virtual void clear_serialisation_records();

  //: Adds record of objects's unique serial number, and location in memory.
  //
  // Adding a pointer that already exists will cause the function to abort(),
  // if debugging is turned on;
  //
  // You can also store a single integer as other data.
  // Interpretation of this data is entirely up to the client code.
  virtual void add_serialisation_record(unsigned long serial_number,
    void *pointer, int other_data = 0);

  //: Returns the pointer to the object identified by the unique serial number.
  // Returns 0 if no record has been added.
  virtual void * get_serialisation_pointer(unsigned long serial_number) const;

  //: Returns the user defined data associated with the unique serial number
  // Returns 0 if no record has been added.
  virtual int get_serialisation_other_data(unsigned long serial_number) const;

  //: Modify the user-defined data associated with the unique serial number
  // If there is no record of the object, this function will return abort.
  virtual int set_serialisation_other_data(unsigned long serial_number,
    int other_data);


  //: Return the version number of the IO format of the file being read.
  unsigned short version_no() const;

protected:
  //: The member stream
  vcl_istream *is_;

  //: The type of the serialisation records.
  typedef vcl_map<unsigned long, vcl_pair<void *, int>, vcl_less<unsigned long> >
    serialisation_records_type;
  
  //: The serialisation records,
  // The record takes a unique identifier of the object (which would be
  // stored on the stream) and returns the pointer to the object, and
  // an other_data integer.
  serialisation_records_type serialisation_records_;

  // The version number of the IO format of the file being read.
  unsigned short version_no_;
};


//: An adapter for a vcl_ifstream to make it suitable for binary IO
class vsl_b_ifstream: public vsl_b_istream
{
public:
  //: Create this adaptor from a file.
  // The adapter will delete the stream automatically on destruction.
  vsl_b_ifstream(const vcl_string &filename, vcl_ios_openmode mode = vcl_ios_in):
    vsl_b_istream(new vcl_ifstream(filename.c_str(),
    mode | vcl_ios_binary)) {}

  //: Create this adaptor from a file.
  // The adapter will delete the stream automatically on destruction.
  vsl_b_ifstream(const char *filename, vcl_ios_openmode mode = vcl_ios_in):
    vsl_b_istream(new vcl_ifstream(filename, mode | vcl_ios_binary)) {}

  //: Virtual destructor.so that it can be overloaded
  virtual ~vsl_b_ifstream();

  //: Close the stream
  void close();
};

//: Write bool to vsl_b_ostream
void vsl_b_write(vsl_b_ostream& os,bool b);
//: Read bool from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,bool& b);
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, bool b )
{  os << b; }

//: Write char to vsl_b_ostream
void vsl_b_write(vsl_b_ostream& os,char n );
//: Read char from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,char& n );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, char n )
{  os << n; }

//: Write signed char to vsl_b_ostream
void vsl_b_write(vsl_b_ostream& os,signed char n );
//: Read  signed char from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,signed char& n );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, signed char n )
{  os << n; }

//: Write  to vsl_b_ostream
void vsl_b_write(vsl_b_ostream& os,unsigned char n );
//: Read  from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,unsigned char& n );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, unsigned char n )
{  os << n; }

//: Write  to vsl_b_ostream
void vsl_b_write(vsl_b_ostream& os,const vcl_string& n );
//: Read  from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,vcl_string& n );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, const vcl_string& n )
{  os << n; }


//: Write  to vsl_b_ostream
void vsl_b_write(vsl_b_ostream& os,const char* s );
//: Read  from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,char* s );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, const char* s )
{  os << s; }

//: Write  to vsl_b_ostream
void vsl_b_write(vsl_b_ostream& os, int n );
//: Read  from vsl_b_istream
void vsl_b_read(vsl_b_istream& is, int& n );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, int n )
{  os << n; }

//: Write  to vsl_b_ostream
void vsl_b_write(vsl_b_ostream& os,unsigned int n );
//: Read  from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,unsigned int& n );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, unsigned int n )
{  os << n; }

//: Write  to vsl_b_ostream
void vsl_b_write(vsl_b_ostream& os,short n );
//: Read  from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,short& n );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, short n )
{  os << n; }

//: Write  to vsl_b_ostream
void vsl_b_write(vsl_b_ostream& os,unsigned short n );
//: Read  from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,unsigned short& n );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, unsigned short n )
{  os << n; }

//: Write  to vsl_b_ostream
void vsl_b_write(vsl_b_ostream& os,long n );
//: Read  from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,long& n );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, long n )
{  os << n; }

//: Write  to vsl_b_ostream
void vsl_b_write(vsl_b_ostream& os,unsigned long n );
//: Read  from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,unsigned long& n );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, unsigned long n )
{  os << n; }

//: Write  to vsl_b_ostream
// Number is saved with ANSI/IEEE Standard 754-1985 single precision.
void vsl_b_write(vsl_b_ostream& os,float n );
//: Read  from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,float& n );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, float n )
{  os << n; }

//: Write  to vsl_b_ostream
// Number is saved with ANSI/IEEE Standard 754-1985 double precision.
void vsl_b_write(vsl_b_ostream& os,double n );
//: Read  from vsl_b_istream
void vsl_b_read(vsl_b_istream& is,double& n );
//: Print to a stream
inline void vsl_print_summary(vcl_ostream& os, double n )
{ os << n; }


//: Write a block of values to a vsl_b_ostream
// If you want to output a block of fundamental data types very efficiently,
// then just #include <vsl_binary_explicit_io.h>
template <class T>
inline void vsl_b_write_block(vsl_b_ostream &os, const T* begin, unsigned nelems)
{
  while (nelems--)
    vsl_b_write(os, *(begin++));
}

//: Read a block of values from a vsl_b_istream
// If you want to output a block of fundamental data types very efficiently,
// then just #include <vsl_binary_explicit_io.h>
template <class T>
inline void vsl_b_read_block(vsl_b_istream &is, T* begin, unsigned nelems)
{
  while (nelems--)
    vsl_b_read(is, *(begin++));
}


#endif // vsl_binary_complex_io_h_
