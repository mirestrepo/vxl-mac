// vxl/core/tests/test_build_info.cxx

// Based on ITK, Testing/Code/Common/itkSystemInformationTest.cxx

#include "test_build_info.h"
#include <vcl_ios.h>
#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_string.h>
#include <vcl_ctime.h>
#include <testlib/testlib_test.h>

// Construct the name of the notes file.
#define vxl_BUILD_INFO_NOTES \
  vxl_BUILD_DIR "/Testing/HTML/TestingResults/Sites/" \
  vxl_SITE "/" vxl_BUILD_NAME "/BuildNameNotes.xml"

static vcl_string
get_current_date_time (const char * format)
{
  char buf[1024];
  vcl_time_t t;
  vcl_time (&t);
  vcl_strftime (buf, sizeof(buf), format, vcl_localtime(&t));
  return buf;
}

static void
system_information_print_file (const char * name, vcl_ostream & os,
                               bool note = false )
{
  if ( ! note)
    os << "================================\n";

  vcl_ifstream fin (name, vcl_ios::in);

  if (fin)
  {
      if ( ! note)
          os << "Contents of \"" << name << "\":\n"
             << "--------------------------------\n";

      const int bufferSize = 4096;
      char buffer[bufferSize];

      // This copy loop is very sensitive on certain platforms with
      // slightly broken stream libraries (like HPUX).  Normally, it is
      // incorrect to not check the error condition on the fin.read()
      // before using the data, but the fin.gcount() will be zero if an
      // error occurred.  Therefore, the loop should be safe everywhere.

      while (fin)
      {
          fin.read (buffer, bufferSize);
          if (fin.gcount())
              os.write(buffer, fin.gcount());
      }
      os.flush();
  }
  else
  {
      os << "Could not open \"" << name << "\" for reading.\n";
  }
}

MAIN( test_build_info )
{
  START("build_info");

  const char* files[] =
    {
      // Note.txt is meant to be created by hand if desired for a build
      vxl_BUILD_DIR "/Note.txt",
      // files below all may be generated by CMake
      vxl_BUILD_DIR "/CMakeCache.txt",
      // can't include CMakeError.log because '&' is special to XML
      // vxl_BUILD_DIR "/CMakeError.log",
      // can't include these because #include <foo.h> looks like XML
      // vxl_BUILD_DIR "/core/vxl_config.h",
      // vxl_BUILD_DIR "/vcl/vcl_config_compiler.h",
      // vxl_BUILD_DIR "/vcl/vcl_config_headers.h",
      // vxl_BUILD_DIR "/vcl/vcl_config_manual.h",
      0
    };

  const char** f;
  for (f = files; *f; f++)
  {
      system_information_print_file (*f, vcl_cout);
  }

  vcl_ofstream outf (vxl_BUILD_INFO_NOTES, vcl_ios::out);
  if (outf)
  {
      vcl_cout << "Also writing this information to file "
               << vxl_BUILD_INFO_NOTES << "\n";

      outf << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
           << "<Site BuildName=\"" << vxl_BUILD_NAME
           << "\"  Name=\"" << vxl_SITE << "\">\n"
           << "<BuildNameNotes>\n";

      for (f = files; *f; ++f)
      {
          outf << "<Note Name=\"" << *f << "\">\n"
               << "<DateTime>"
               << get_current_date_time ("%a %b %d %Y %H:%M:%S %Z")
               << "</DateTime>\n"
               << "<Text>\n";

          system_information_print_file (*f, outf, true);

          outf << "</Text>\n"
               << "</Note>\n";
      }

      outf << "</BuildNameNotes>\n"
           << "</Site>\n";
  }
  else
  {
      vcl_cerr << "Error writing this information to file "
               << vxl_BUILD_INFO_NOTES << "\n";
      return 1;
  }

  SUMMARY();
}
