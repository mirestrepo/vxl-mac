// This is ./vxl/vul/vul_file.cxx
#ifdef __GNUC__
#pragma implementation
#endif

//:
// \file
//
// \author Andrew W. Fitzgibbon, Oxford RRG
// \date   02 Nov 98
//
//-----------------------------------------------------------------------------

#include "vul_file.h"

#include <sys/stat.h>
#include <vcl_cstring.h>
#include <vcl_cstdlib.h>

#ifdef VCL_WIN32
#include <direct.h> // for getcwd, mkdir
#else
#include <unistd.h>
#endif

#include <vul/vul_user_info.h>

vcl_string vul_file::get_cwd()
{
  const int BIG = 65536;
  char buf[BIG];
  getcwd(buf,BIG-1);
  return buf;
}

bool vul_file::make_directory(char const* name)
{
#ifdef VCL_WIN32
  return -1 != mkdir(name);
#else
  return -1 != mkdir(name, 0755);
#endif
}

bool vul_file::is_directory(char const* fn)
{
  struct stat fs;
  return (stat(fn, &fs) == 0) && ((fs.st_mode & S_IFMT) == S_IFDIR);
}

int vul_file::size(char const* fn)
{
  struct stat fs;
  if (stat(fn, &fs) == 0)
    return fs.st_size;
  else
    return 0;
}

bool vul_file::exists(char const* fn)
{
  struct stat fs;
  return (stat(fn, &fs) == 0);
}

vcl_string vul_file::dirname(char const* fn)
{
  vcl_string self(fn);

  vcl_string::size_type slash_index = self.rfind('/');
  if (slash_index == vcl_string::npos)
    return ".";

  return self.substr(0, slash_index);
}

vcl_string vul_file::extension(char const* fn)
{
  vcl_string self(fn);

  unsigned int dot_index = self.rfind('.');
  if (dot_index != vcl_string::npos)
    return self.substr(dot_index, vcl_string::npos);
  else
    return vcl_string();
}

vcl_string vul_file::strip_directory(char const* fn)
{
   vcl_string self(fn);

   unsigned int slash_index = self.rfind('/');
   if (slash_index != vcl_string::npos)
     self.erase(0, slash_index+1);

   return self;
}

vcl_string vul_file::strip_extension(char const* fn)
{
  vcl_string self(fn);

  unsigned int dot_index = self.rfind('.');
  if (dot_index != vcl_string::npos)
    self.erase(dot_index, vcl_string::npos);

  return self;
}

vcl_string vul_file::basename(char const* fn, char const * suffix)
{
  // First strip dir
  vcl_string self(fn);

  unsigned int slash_index = self.rfind('/');
  if (slash_index != vcl_string::npos)
    self.erase(0, slash_index+1);

  // Now strip suffix if any
  if (suffix) {
    int start = self.size() - vcl_strlen(suffix);
    if (start >= 0)
      // egcs, 2.95, 2.96 have no method which can do
      //   self.compare(start, vcl_string::npos, suffix) == 0
      if (vcl_string(self.begin()+start, self.end()) == suffix)
        self.erase(start, vcl_string::npos);
  }
  return self;
}


#ifdef VCL_WIN32
//: replace instances of 'from' in 's' with 'to'
static unsigned replace(char from, char to, vcl_string &s)
{
  unsigned c = 0;
  for (unsigned i=0; i<s.size(); ++i)
    if (s[i] == from)
    {
      c++;
      s[i] = to;
    }
    return c;
}
#endif

//: Delete 1 or more files using the Local OS prefered globbing.
// E.g. \c delete_file_glob("*"); will delete all the files in the
// current directory on most operating systems.
// Takes Posix path separators i.e. '/'
bool vul_file::delete_file_glob(char const* file_glob)
{
  vcl_string command = file_glob;
#ifdef VCL_WIN32
  replace('/', '\\', command);
  command = "del " + command;
#else
  command = "rm " + command;
#endif
  return vcl_system(command.c_str())==0;
}


vcl_string vul_file::expand_tilde(char const* vul_filename)
{
  if (!vul_filename || (vcl_strlen(vul_filename) == 0))
    return "";

#ifdef VCL_WIN32
  // ~ meaningless on win32
  return vcl_string(vul_filename);
#else

  if (vul_filename[0] != '~')
    return vcl_string(vul_filename);

  //// ** Have a tilde, go for it

  // 1. Strip to directory only, and remove the tilde itself
  vcl_string fn(vul_filename);
  vcl_string dir;
  unsigned int first_slash =  fn.find('/');
  if (first_slash != vcl_string::npos) {
    dir = fn.substr(1, first_slash-1);
    fn = fn.substr(first_slash, vcl_string::npos);
  } else {
    dir = fn.substr(1, vcl_string::npos);
    fn = "";
  }
  // Now, from original to  (dir, vul_filename) is one of
  //  ~            ""     ""
  //  ~fre         "fre"  ""
  //  ~/fred       ""     "/fred"
  //  ~user/fred   "user" "/fred"

  if (dir.size() == 0) {
    // Was just ~, use getenv(HOME)
    char const * home_directory = getenv("HOME");
    if (!home_directory) home_directory = "";
    return vcl_string(home_directory) + fn;
  }

  // Was ~user, Check password list for match
  vul_user_info user(dir);
  if (!user.ok)
    return vcl_string(vul_filename);

  // Got user info
  return user.home_directory + fn;
#endif
}
