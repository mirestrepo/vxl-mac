#ifndef vsl_quick_file_h_
#define vsl_quick_file_h_

//:
// \file
// \brief Functions for quickly loading and saving binary files.
// \author Ian Scott
//
// All the functions return true if successful.
// The functions will also output a success or failure message to stderr by
// default, although you may substitute any vcl_ostream, or null to avoid the
// message.
// 
// For these templated functions to work, the object must have vsl_b_read and
// vsl_b_write functions defined for them

#include <vcl_string.h>
#include <vsl/vsl_binary_io.h>
#include <vcl_iostream.h>



//: Load something from a file 
template <class T>
inline bool vsl_quick_file_load(T &data,
  const vcl_string& path,
  vcl_ostream* errorStream = &vcl_cerr)
{
  vsl_b_ifstream bfs(path);
  if( !(!bfs))
  {
    vsl_b_read(bfs,data);
    if (!(!bfs))
    {
      // Check that we have reached the end of the file.
      char dummy;
      vsl_b_read(bfs,dummy);
      if (bfs.is().eof())
      {
        bfs.close();
        if (errorStream)
          *errorStream << "Successfully loaded: " << path << vcl_endl;
        return true;
      }
    }
  }
  bfs.close();
  if (errorStream)
    *errorStream << "Unable to load: "<< path <<vcl_endl;
  return false;
}


//: Save something to a file 
template <class T>
inline bool vsl_quick_file_save(T &data,
  const vcl_string& path,
  vcl_ostream* errorStream = &vcl_cerr)
{
  vsl_b_ofstream bfs(path);
  if(!(!bfs))
  {
    vsl_b_write(bfs,data);
    if (!(!bfs))
    {
      bfs.close();
      if (errorStream)
        *errorStream << "Successfully saved: "<< path <<vcl_endl;
      return true;
    }
  }
  bfs.close();
  if (errorStream)
    *errorStream << "Unable to save: " << path << vcl_endl;
  return false;
} 

// Load two objects from a file 
template <class T, class S>
inline bool vsl_quick_file_load(T &data1,
  S &data2,
  const vcl_string& path,
  vcl_ostream* errorStream = &vcl_cerr)
{
  vsl_b_ifstream bfs(path);
  int reason = errno;
  if( !(!bfs))
  {
    vsl_b_read(bfs,data1);
    vsl_b_read(bfs,data2);
    if (!(!bfs))
    {
      // Check that we have reached the end of the file.
      char dummy;
      vsl_b_read(bfs,dummy);
      if (bfs.is().eof())
      {
        bfs.close();
        if (errorStream)
          *errorStream << "Successfully loaded: " << path << vcl_endl;
        return true;
      }
    }
  }
  reason = errno;
  bfs.close();
  if (errorStream)
    *errorStream << "Unable to load: "<< path <<vcl_endl;
  return false;
}

// Save two objects to a file 
template <class T, class S>
inline bool vsl_quick_file_save(T &data1,
  S &data2,
  const vcl_string& path,
  vcl_ostream* errorStream = &vcl_cerr)
{
  vsl_b_ofstream bfs(path);
  if(!(!bfs))
  {
    vsl_b_write(bfs,data1);
    vsl_b_write(bfs,data2);
    if (!(!bfs))
    {
      bfs.close();
      if (errorStream)
        *errorStream << "Successfully saved: "<< path <<vcl_endl;
      return true;
    }
  }
  bfs.close();
  if (errorStream)
    *errorStream << "Unable to save: " << path << vcl_endl;
  return false;
} 

// Load three objects from a file 
template <class T, class S, class U>
inline bool vsl_quick_file_load(T &data1,
  S &data2, U &data3,
  const vcl_string& path,
  vcl_ostream* errorStream = &vcl_cerr)
{
  vsl_b_ifstream bfs(path);
  if( !(!bfs))
  {
    vsl_b_read(bfs,data1);
    vsl_b_read(bfs,data2);
    vsl_b_read(bfs,data3);
    if (!(!bfs))
    {
      // Check that we have reached the end of the file.
      char dummy;
      vsl_b_read(bfs,dummy);
      if (bfs.is().eof())
      {
        bfs.close();
        if (errorStream)
          *errorStream << "Successfully loaded: " << path << vcl_endl;
        return true;
      }
    }
  }
  bfs.close();
  if (errorStream)
    *errorStream << "Unable to load: "<< path <<vcl_endl;
  return false;
}

// Save three objects to a file 
template <class T, class S, class U>
inline bool vsl_quick_file_save(T &data1,
  S &data2, U &data3,
  const vcl_string& path,
  vcl_ostream* errorStream = &vcl_cerr)
{
  vsl_b_ofstream bfs(path);
  if(!(!bfs))
  {
    vsl_b_write(bfs,data1);
    vsl_b_write(bfs,data2);
    vsl_b_write(bfs,data3);
    if (!(!bfs))
    {
      bfs.close();
      if (errorStream)
        *errorStream << "Successfully saved: "<< path <<vcl_endl;
      return true;
    }
  }
  bfs.close();
  if (errorStream)
    *errorStream << "Unable to save: " << path << vcl_endl;
  return false;
} 

// Load two objects from a file 
template <class T, class S, class U, class V>
inline bool vsl_quick_file_load(T &data1,
  S &data2, U &data3, V &data4,
  const vcl_string& path,
  vcl_ostream* errorStream = &vcl_cerr)
{
  vsl_b_ifstream bfs(path);
  if( !(!bfs))
  {
    vsl_b_read(bfs,data1);
    vsl_b_read(bfs,data2);
    vsl_b_read(bfs,data3);
    vsl_b_read(bfs,data4);
    if (!(!bfs))
    {
      // Check that we have reached the end of the file.
      char dummy;
      vsl_b_read(bfs,dummy);
      if (bfs.is().eof())
      {
        bfs.close();
        if (errorStream)
          *errorStream << "Successfully loaded: " << path << vcl_endl;
        return true;
      }
    }
  }
  bfs.close();
  if (errorStream)
    *errorStream << "Unable to load: "<< path <<vcl_endl;
  return false;
}

// Save four objects to a file 
template <class T, class S, class U, class V>
inline bool vsl_quick_file_save(T &data1,
  S &data2, U &data3, V &data4,
  const vcl_string& path,
  vcl_ostream* errorStream = &vcl_cerr)
{
  vsl_b_ofstream bfs(path);
  if(!(!bfs))
  {
    vsl_b_write(bfs,data1);
    vsl_b_write(bfs,data2);
    vsl_b_write(bfs,data3);
    vsl_b_write(bfs,data4);
    if (!(!bfs))
    {
      bfs.close();
      if (errorStream)
        *errorStream << "Successfully saved: "<< path <<vcl_endl;
      return true;
    }
  }
  bfs.close();
  if (errorStream)
    *errorStream << "Unable to save: " << path << vcl_endl;
  return false;
} 


#endif
