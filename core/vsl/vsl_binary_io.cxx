#include <vsl/vsl_binary_io.h>
#include <vsl/vsl_binary_explicit_io.h>
#include <vxl_config.h>
#include <vcl_map.txx>


// \file
// \brief Functions to perform consistent binary IO within vsl
// \author Tim Cootes and Ian Scott



void vsl_b_write(vsl_b_ostream& os, char n )
{
  os.os().write( ( char* )&n, sizeof( n ) );
}

void vsl_b_read(vsl_b_istream &is, char& n )
{
  is.is().read( ( char* )&n, sizeof( n ) );
}

void vsl_b_write(vsl_b_ostream& os, signed char n )
{
  os.os().write( ( char* )&n, sizeof( n ) );
}

void vsl_b_read(vsl_b_istream &is, signed char& n )
{
  is.is().read( ( char* )&n, sizeof( n ) );
}



void vsl_b_write(vsl_b_ostream& os,unsigned char n )
{
  os.os().write( ( char* )&n, 1 );
}

void vsl_b_read(vsl_b_istream &is,unsigned char& n )
{
  is.is().read( ( char* )&n, 1 );
}



void vsl_b_write(vsl_b_ostream& os, const vcl_string& str )
{
    vcl_string::const_iterator          it;

    vsl_b_write(os,(short)str.length());
    for ( it = str.begin(); it != str.end(); ++it )
        vsl_b_write(os,*it);
}

void vsl_b_read(vsl_b_istream &is, vcl_string& str )
{
    vcl_string::iterator                it;
    short                           length;

    vsl_b_read(is,length);
    str.resize( length );
    for ( it = str.begin(); it != str.end(); ++it )
        vsl_b_read(is,*it);
}




void vsl_b_write(vsl_b_ostream& os,const char *s )
{
  int i = -1;
  do {
     i++;
     vsl_b_write(os,s[i]);
  } while ( s[i] != 0 );
}

void vsl_b_read(vsl_b_istream &is,char *s )
{
  int i = -1;
  do {
    i++;
    vsl_b_read(is,s[i]);
  } while ( s[i] != 0 );
}




void vsl_b_write(vsl_b_ostream& os,bool b)
{
  if (b)
    vsl_b_write(os, (signed char) -1);
  else
    vsl_b_write(os, (signed char) 0);
}

void vsl_b_read(vsl_b_istream &is,bool& b)
{
  signed char c;
  vsl_b_read(is, c);
  b = (c != 0);
}




void vsl_b_write(vsl_b_ostream& os,int n )
{
  unsigned char buf[ VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(int)) ];
  unsigned long nbytes = vsl_convert_to_arbitrary_length(&n, buf);
  os.os().write((char*)buf, nbytes );
}

void vsl_b_read(vsl_b_istream &is,int& n )
{
  unsigned char buf[VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(int))];
  unsigned char *ptr=buf;
  do
  {
    vsl_b_read(is, *ptr);
    assert (ptr-buf < VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(int)));
  }
  while (!(*(ptr++) & 128));
  vsl_convert_from_arbitrary_length(buf, &n);
}



void vsl_b_write(vsl_b_ostream& os,unsigned int n )
{
  unsigned char
    buf[VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(unsigned int))];
  unsigned long nbytes = vsl_convert_to_arbitrary_length(&n, buf);
  os.os().write((char*)buf, nbytes );
}

void vsl_b_read(vsl_b_istream &is,unsigned int& n )
{
  unsigned char buf[
    VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(unsigned int))];
  unsigned char *ptr=buf;
  do
  {
    vsl_b_read(is, *ptr);
    assert (ptr-buf <
      VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(unsigned int)));
  }
  while (!(*(ptr++) & 128));
  vsl_convert_from_arbitrary_length(buf, &n);
}



void vsl_b_write(vsl_b_ostream& os,short n )
{
  unsigned char buf[VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(short))];
  unsigned long nbytes = vsl_convert_to_arbitrary_length(&n, buf);
  os.os().write((char*)buf, nbytes );
}

void vsl_b_read(vsl_b_istream &is,short& n )
{
  unsigned char buf[VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(short))];
  unsigned char *ptr=buf;
  do
  {
    vsl_b_read(is, *ptr);
    assert (ptr-buf < VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(short)));
  }
  while (!(*(ptr++) & 128));
  vsl_convert_from_arbitrary_length(buf, &n);
}

void vsl_b_write(vsl_b_ostream& os, unsigned short n )
{
  unsigned char buf[
    VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(unsigned short))];
  unsigned long nbytes = vsl_convert_to_arbitrary_length(&n, buf);
  os.os().write((char*)buf, nbytes );
}

void vsl_b_read(vsl_b_istream &is, unsigned short& n )
{
  unsigned char buf[
    VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(unsigned short))];
  unsigned char *ptr=buf;
  do
  {
    vsl_b_read(is, *ptr);
    assert (ptr-buf < 
      VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(unsigned short)));
  }
  while (!(*(ptr++) & 128));
  vsl_convert_from_arbitrary_length(buf, &n);
}




void vsl_b_write(vsl_b_ostream& os,long n )
{
  unsigned char buf[VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(long))];
  unsigned long nbytes = vsl_convert_to_arbitrary_length(&n, buf);
  os.os().write((char*)buf, nbytes );
}

void vsl_b_read(vsl_b_istream &is,long& n )
{
  unsigned char buf[VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(long))];
  unsigned char *ptr=buf;
  do
  {
    vsl_b_read(is, *ptr);
    assert (ptr-buf < VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(long)));
  }
  while (!(*(ptr++) & 128));
  vsl_convert_from_arbitrary_length(buf, &n);
}

void vsl_b_write(vsl_b_ostream& os,unsigned long n )
{
  unsigned char buf[
    VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(unsigned long))];
  unsigned long nbytes = vsl_convert_to_arbitrary_length(&n, buf);
  os.os().write((char*)buf, nbytes );
}

void vsl_b_read(vsl_b_istream &is,unsigned long& n )
{
  unsigned char buf[
    VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(unsigned long))];
  unsigned char *ptr=buf;
  do
  {
    vsl_b_read(is, *ptr);
    assert (ptr-buf < 
      VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(unsigned long)));
  }
  while (!(*(ptr++) & 128));
  vsl_convert_from_arbitrary_length(buf, &n);
}




void vsl_b_write(vsl_b_ostream& os,float n )
{
  vsl_swap_bytes(( char* )&n, sizeof( n ) );
  os.os().write( ( char* )&n, sizeof( n ) );
}

void vsl_b_read(vsl_b_istream &is,float& n )
{
  is.is().read( ( char* )&n, sizeof( n ) );
  vsl_swap_bytes(( char* )&n, sizeof( n ) );
}

void vsl_b_write(vsl_b_ostream& os,double n )
{
  vsl_swap_bytes(( char* )&n, sizeof( n ) );
  os.os().write( ( char* )&n, sizeof( n ) );
}

void vsl_b_read(vsl_b_istream &is,double& n )
{
  is.is().read( ( char* )&n, sizeof( n ) );
  vsl_swap_bytes(( char* )&n, sizeof( n ) );
}





  const unsigned short vsl_b_ostream::version_no_ = 1;
  static const unsigned short vsl_magic_number_part_1=0x2c4e;
  static const unsigned short vsl_magic_number_part_2=0x472b;


  //: Create this adaptor using an existing stream
  // The stream (os) must be open (i.e. ready to be written to) so that the
  // IO version number can be written by this constructor.
  // User is responsible for deleting os after deleting the adaptor
  vsl_b_ostream::vsl_b_ostream(vcl_ostream *os): os_(os)
  {
    assert(os_);
    vsl_b_write_uint_16(*this, version_no_);
    vsl_b_write_uint_16(*this, vsl_magic_number_part_1);
    vsl_b_write_uint_16(*this, vsl_magic_number_part_2);
  }

  //: A reference to the adaptor's stream
  vcl_ostream& vsl_b_ostream::os() const
  {
    assert(os_);
    return *os_;
  }

  //: Returns true if the underlying stream has its fail bit set.
  bool vsl_b_ostream::operator!() const
  {
    return os_->operator!();
  }

  


  //: Clear the stream's record of any serialisation operations
  // Calling this function while outputing serialisable things to stream,
  // will mean 
  // that a second copy of an object may get stored to the stream.
  void vsl_b_ostream::clear_serialisation_records()
  {
    serialisation_records_.clear();
  }


  //: Adds an object pointer to the serialisation records.
  // Returns a unique identifier for the object.
  // You can optionally add some user-defined integer with each record
  // If error checking is on, and the object pointer is already in the records,
  // this function will abort()
  unsigned long vsl_b_ostream::add_serialisation_record
                      (void *pointer, int other_data /*= 0*/)
  {
    assert (serialisation_records_.find(pointer) ==
      serialisation_records_.end());
    unsigned long id = serialisation_records_.size() + 1;
    serialisation_records_[pointer] = vcl_make_pair(id, other_data);
    return id;
  }


  //: Returns a unique identifier for the object.
  // Returns 0 if there is no record of the object.
  unsigned long vsl_b_ostream::get_serial_number(void *pointer) const
  {
    serialisation_records_type::const_iterator entry =
      serialisation_records_.find(pointer);
    if (entry == serialisation_records_.end())
    {
      return 0;
    }
    else
    {
      return entry->second.first;
    }
  }

  //: Set the user-defined data associated with the object
  // If there is no record of the object, this function will return 0.
  // However a retval of 0 does not necessarily imply that the object is
  // unrecorded.
  int vsl_b_ostream::get_serialisation_other_data(void *pointer) const
  {
    serialisation_records_type::const_iterator entry =
      serialisation_records_.find(pointer);
    if (entry == serialisation_records_.end())
    {
      return 0;
    }
    else
    {
      return entry->second.second;
    }
  }

  //: Modify the user-defined data associated with the object.
  // If there is no record of the object, this function will return abort.
  int vsl_b_ostream::set_serialisation_other_data
      (void *pointer, int other_data)
  {
    serialisation_records_type::iterator entry =
      serialisation_records_.find(pointer);
    if (entry == serialisation_records_.end())
    {
      vcl_cerr << "vsl_b_ostream::set_serialisation_other_data(): "
               << "No such value " << pointer << "in records." <<vcl_endl;
      vcl_abort(); return -1;
    }
    else
    {
      return entry->second.second;
    }
  }




  //: destructor.
  vsl_b_ofstream::~vsl_b_ofstream()
  {
    if (os_) delete os_;
  }


  //: Close the stream
  void vsl_b_ofstream::close()
  {
    assert(os_);
    ((vcl_ofstream *)os_)->close();
    clear_serialisation_records();
  }

  //: Create this adaptor using an existing stream.
  // The stream (is) must be open (i.e. ready to be read from) so that the
  // IO version number can be read by this constructor.
  // User is responsible for deleting is after deleting the adaptor
  vsl_b_istream::vsl_b_istream(vcl_istream *is): is_(is)
  {
    assert(is_);
    if (!(*is_)) return;
    unsigned long v, m1, m2;
    vsl_b_read_uint_16(*this, v);
    vsl_b_read_uint_16(*this, m1);
    vsl_b_read_uint_16(*this, m2);

    // If this test fails, either the file is missing, or it is not a
    // Binary VXL file, or it is a corrupted Binary VXL file
    if (m2 != vsl_magic_number_part_2 || m1 != vsl_magic_number_part_1)
    {
      vcl_cerr << "\nI/O WARNING: The input stream does not appear to be"
               << " a Binary VXL stream" << vcl_endl;
      is_->clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
    }

    if (v != 1)
    {
      vcl_cerr << "\nI/O WARNING: The stream's leading version number is "
               << v << ". Expected value 1." << vcl_endl;
      is_->clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
    }
    version_no_ = v;
  }

  //: A reference to the adaptor's stream
  vcl_istream & vsl_b_istream::is() const
  {
    assert(is_);
    return *is_;
  }


  //: Returns true if the underlying stream has its fail bit set.
  bool vsl_b_istream::operator!() const
  {
    return is_->operator!();
  }

  //: Clear the stream's record of any serialisation operations
  // Calling this function while inputting serialisable things from a stream,
  // could cause errors during loading unless the records were cleared at a
  // similar point during output.
  void vsl_b_istream::clear_serialisation_records()
  {
    serialisation_records_.clear();
  }

  //: Adds record of objects's unique serial number, and location in memory.
  //
  // Adding a pointer that already exists will cause the function to abort(),
  // if debugging is turned on;
  //
  // You can also store a single integer as other data.
  // Interpretation of this data is entirely up to the client code.
  void vsl_b_istream::add_serialisation_record(unsigned long serial_number,
      void *pointer, int other_data /*= 0*/)
  {
    assert (serialisation_records_.find(serial_number) == 
      serialisation_records_.end());
    serialisation_records_[serial_number] = vcl_make_pair(pointer, other_data);
  }

  //: Returns the pointer to the object identified by the unique serial number.
  // Returns 0 if no record has been added.
  void * vsl_b_istream::get_serialisation_pointer
      (unsigned long serial_number) const
  {
    serialisation_records_type::const_iterator entry =
          serialisation_records_.find(serial_number);
    if (entry == serialisation_records_.end())
    {
      return 0;
    }
    else
    {
      return entry->second.first;
    }
  }

  //: Returns the user defined data associated with the unique serial number
  // Returns 0 if no record has been added.
  int vsl_b_istream::get_serialisation_other_data
      (unsigned long serial_number) const
  {
    serialisation_records_type::const_iterator entry =
      serialisation_records_.find(serial_number);
    if (entry == serialisation_records_.end())
    {
      return 0;
    }
    else
    {
      return entry->second.second;
    }
  }

  //: Modify the user-defined data associated with the unique serial number
  // If there is no record of the object, this function will return abort.
  int vsl_b_istream::set_serialisation_other_data
      (unsigned long serial_number, int other_data)
  {
    serialisation_records_type::const_iterator entry =
      serialisation_records_.find(serial_number);
    if (entry == serialisation_records_.end())
    {
      vcl_cerr << "vsl_b_istream::set_serialisation_other_data(): "
               << "No such value " << serial_number << "in records."
               << vcl_endl;
      vcl_abort(); return -1;
    }
    else
    {
      return entry->second.second;
    }
  }




  //: destructor.so that it can be overloaded
  vsl_b_ifstream::~vsl_b_ifstream()
  {
    if (is_) delete is_;
  }

  //: Close the stream
  void vsl_b_ifstream::close()
  {
    assert(is_);
    ((vcl_ifstream *)is_)->close();
    clear_serialisation_records();
  }




// Explicit instantiation of map
typedef vcl_pair<unsigned long, int > vcl_pair_ulong_int;
typedef vcl_pair<void*, int > vcl_pair_vstar_int;
VCL_MAP_INSTANTIATE(void*, vcl_pair_ulong_int, vcl_less<void* > );
VCL_MAP_INSTANTIATE(unsigned long, vcl_pair_vstar_int,\
                    vcl_less<unsigned long > );

