#ifdef __GNUC__
#pragma implementation
#endif

#include <vcl_cassert.h>
#include <vcl_cstdio.h>    // perror()
#include <vcl_iostream.h>
#include <vil/vil_stream_fstream.h>

static int modeflags(char const* mode)
{
  if (*mode == 0)
    return 0;

  if (*mode == 'r') 
    return vcl_ios_in | modeflags(mode+1);
  
  if (*mode == 'w')
    return vcl_ios_out | modeflags(mode+1);

  vcl_cerr << "DODGY MODE " << mode << vcl_endl;
  return 0;
}

#define xerr if (true) ; else (vcl_cerr << "vcl_fstream#" << id_ << ": ")

static int id = 0;

vil_stream_fstream::vil_stream_fstream(char const* fn, char const* mode):
  flags_(modeflags(mode)),
  f_(fn, 
#ifdef GNU_LIBSTDCXX_V3
     std::ios::openmode
#endif
     (flags_ | vcl_ios_binary)) // need ios::binary on windows.
{
  id_ = ++id;
  xerr << "vil_stream_fstream(\"" << fn << "\", \""<<mode<<"\") = " << id_ << "\n";
  if (!f_.good()) {
    vcl_cerr << "vil_stream_fstream::Could not open [" << fn << "]\n";
  }
}

//vil_stream_fstream::vil_stream_fstream(fstream& f):
//  f_(f.rdbuf()->fd())
//{
//}

vil_stream_fstream::~vil_stream_fstream()
{
  xerr << "vil_stream_fstream# " << id_ << " being deleted\n";
}

bool vil_stream_fstream::ok()
{
  return f_.good();
}

int vil_stream_fstream::write(void const* buf, int n)
{
  assert(id > 0);
  if (!(flags_ & vcl_ios_out)) {
    vcl_cerr << "vil_stream_fstream: write failed, not an vcl_ostream\n";
    return 0;
  }

  vcl_streampos a = tell();
  xerr << "write " << n << vcl_endl;
  f_.write((char const*)buf, n);
  if (!f_.good())
    vcl_perror("vil_stream_fstream: ERROR: write failed!\n");
  vcl_streampos b = tell();
  f_.flush();
  return b-a;
}


int vil_stream_fstream::read(void* buf, int n)
{
  assert(id > 0);

  if (!(flags_ & vcl_ios_in))
    return 0;

  vcl_streampos a = tell();
  xerr << "read " << n << vcl_endl;
  f_.read((char *)buf, n);
  vcl_streampos b = tell();

  // fsm@robots  This is for gcc 2.95 :
  // If we try to read more data than is in the file, the good()
  // function will return false even though bad() returns false
  // too. The stream is actually fine but we need to clear the
  // eof flag to use it again.
  if (!f_.good() && !f_.bad() && f_.eof())
    f_.clear(); // allows subsequent operations

  int numread = b-a;
  if (numread < 0) { xerr << "urgh!" << vcl_endl; return -1; }
  if (numread != n) xerr << "only read " << numread << vcl_endl;
  return numread;
}

int vil_stream_fstream::tell()
{
  assert(id > 0);
  if (flags_ & vcl_ios_in) {
    xerr << "tellg\n";
    return f_.tellg();
  }

  if (flags_ & vcl_ios_out) {
    xerr << "tellp\n";
    return f_.tellp();
  }

  assert(false); // did you get here? use at least one of vcl_ios_in, vcl_ios_out.
  return -1;
}

void vil_stream_fstream::seek(int position)
{
  assert(id > 0);
  bool fi = (flags_ & vcl_ios_in)  != 0;
  bool fo = (flags_ & vcl_ios_out) != 0;
  
  if (fi && fo) {
    xerr << "seekg and seekp to " << position << vcl_endl;
    if (position != f_.tellg()) {
      f_.seekg(position);
      f_.seekp(position);
      assert(f_.good());
    }
  }
  
  else if (fi) {
    xerr << "seek to " << position << vcl_endl;
    if (position != f_.tellg()) {
      f_.seekg(position);
      assert(f_.good());
    }
  }

  else if (fo) {
    xerr << "seekp to " << position << vcl_endl;
    int at = f_.tellp();
    if (position != at) {
      xerr << "seekp to " << position << ", at " << f_.tellp() << vcl_endl;
      f_.seekp(position);
      assert(f_.good());
    }
  }
  else
    assert(false); // see above assert
}
