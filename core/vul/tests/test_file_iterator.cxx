// This is core/vul/tests/test_file_iterator.cxx
#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_map.h>
#include <vpl/vpl.h>
#include <vul/vul_file.h>
#include <vul/vul_timer.h>
#include <vul/vul_temp_filename.h>

#include <testlib/testlib_test.h>

// file iter
#include <vul/vul_file_iterator.h>

static void touch(char const* fn)
{
  vcl_ofstream f(fn); f << ".";
}

void test_file_iterator_unix()
{
  // Make a directory structure
  vpl_mkdir("/tmp/vxltest", 0777);

  vpl_mkdir("/tmp/vxltest/a", 0777);
  touch("/tmp/vxltest/a/123.dat");
  touch("/tmp/vxltest/a/123.txt");
  touch("/tmp/vxltest/a/13.dat");

  vpl_mkdir("/tmp/vxltest/b", 0777);
  touch("/tmp/vxltest/b/123.dat");
  touch("/tmp/vxltest/b/123.txt");
  touch("/tmp/vxltest/b/13.dat");

  // 0. Check "*"
  {
    vul_file_iterator f("/tmp/vxltest/a/*");

    // test for files - don't care about order?
    vcl_map<vcl_string, int> found;
    TEST("test_file_iterator 1", found[f()], 0); ++found[f()];
    ++f;
    TEST("test_file_iterator 2", found[f()], 0); ++found[f()];
    ++f;
    TEST("test_file_iterator 3", found[f()], 0); ++found[f()];
    ++f;
    TEST("test_file_iterator 4", found[f()], 0); ++found[f()];
    ++f;
    TEST("test_file_iterator 5", found[f()], 0); ++found[f()];
    ++f;
    TEST("F ran out 1", !f, true);

    TEST("test_file_iterator once 1", found["/tmp/vxltest/a/."], 1);
    TEST("test_file_iterator once 2", found["/tmp/vxltest/a/.."], 1);
    TEST("test_file_iterator once 3", found["/tmp/vxltest/a/123.dat"], 1);
    TEST("test_file_iterator once 4", found["/tmp/vxltest/a/123.txt"], 1);
    TEST("test_file_iterator once 5", found["/tmp/vxltest/a/13.dat"], 1);
  }

  // 1. Check file.*
  if (0)// This doesn't support more than * yet see note in vul_fil_iterator.cxx
  {
    vul_file_iterator f("/tmp/vxltest/a/123.*");
    // test for files - don't care about order?
    vcl_map<vcl_string, int> found;
    TEST("test_file_iterator 6", found[f()], 0); ++found[f()];
    ++f;
    TEST("test_file_iterator 7", found[f()], 0); ++found[f()];
    ++f;
    TEST("F ran out 2", !f, true);

    TEST("test_file_iterator once 6", found["/tmp/vxltest/a/123.dat"], 1);
    TEST("test_file_iterator once 7", found["/tmp/vxltest/a/123.txt"], 1);
  }

  // 2. Check *.ext
  if (0)// This doesn't support more than * yet see note in vul_fil_iterator.cxx
  {
    vul_file_iterator f("/tmp/vxltest/a/*.dat");
    // test for files - don't care about order?
    vcl_map<vcl_string, int> found;
    TEST("test_file_iterator 8", found[f()], 0); ++found[f()];
    ++f;
    TEST("test_file_iterator 9", found[f()], 0); ++found[f()];
    ++f;
    TEST("F ran out 3", !f, true);

    TEST("test_file_iterator once 8", found["/tmp/vxltest/a/123.dat"], 1);
    TEST("test_file_iterator once 9", found["/tmp/vxltest/a/13.dat"], 1);
  }

  // 3. Check ?.*
  if (0)// This doesn't support more than * yet see note in vul_fil_iterator.cxx
  {
    vul_file_iterator f("/tmp/vxltest/a/1?3.???");
    // test for files - don't care about order?
    vcl_map<vcl_string, int> found;
    TEST("test_file_iterator 10", found[f()], 0); ++found[f()];
    ++f;
    TEST("test_file_iterator 11", found[f()], 0); ++found[f()];
    ++f;
    TEST("F ran out 4", !f, true);

    TEST("test_file_iterator once 10", found["/tmp/vxltest/a/123.dat"], 1);
    TEST("test_file_iterator once 11", found["/tmp/vxltest/a/123.txt"], 1);
  }

#if 0 // "*/..." does not work (yet).  See note in vul_file_iterator.cxx
  {
    vul_file_iterator f("/tmp/vxltest/*/123.dat");
    TEST("test_file_iterator 12", found[f()], 0); ++found[f()];
    ++f;
    TEST("test_file_iterator 13", found[f()], 0); ++found[f()];
    ++f;
    TEST("F ran out 5", !f, true);

    TEST("test_file_iterator once 12", found["/tmp/vxltest/a/123.dat"], 1);
    TEST("test_file_iterator once 13", found["/tmp/vxltest/b/123.dat"], 1);
  }
#endif

  // 4. Remove and check removal
  vpl_unlink("/tmp/vxltest/b/13.dat");
  vpl_unlink("/tmp/vxltest/b/123.txt");
  vpl_unlink("/tmp/vxltest/b/123.dat");
  {
    vul_file_iterator f("/tmp/vxltest/b/*");
    // test for files - don't care about order?
    vcl_map<vcl_string, int> found;
    TEST("test_file_iterator 14", found[f()], 0); ++found[f()];
    ++f;
    TEST("test_file_iterator 15", found[f()], 0); ++found[f()];
    ++f;
    TEST("F ran out 6", !f, true);

    TEST("test_file_iterator once 14", found["/tmp/vxltest/b/."], 1);
    TEST("test_file_iterator once 15", found["/tmp/vxltest/b/.."], 1);
  }
  vpl_rmdir("/tmp/vxltest/b");

  vpl_unlink("/tmp/vxltest/a/13.dat");
  vpl_unlink("/tmp/vxltest/a/123.txt");
  vpl_unlink("/tmp/vxltest/a/123.dat");
  vpl_rmdir("/tmp/vxltest/a");
  {
    vul_file_iterator f("/tmp/vxltest/*");
    // test for files - don't care about order?
    vcl_map<vcl_string, int> found;
    TEST("test_file_iterator 16", found[f()], 0); ++found[f()];
    ++f;
    TEST("test_file_iterator 17", found[f()], 0); ++found[f()];
    ++f;
    TEST("F ran out 7", !f, true);

    TEST("test_file_iterator 16", found["/tmp/vxltest/."], 1);
    TEST("test_file_iterator 17", found["/tmp/vxltest/.."], 1);
  }

  vpl_rmdir("/tmp/vxltest");
}

void test_file_iterator_dos()
{
  // Make a directory structure
  vcl_string tempdir = vul_temp_filename();
  vpl_mkdir( tempdir.c_str(), 0777);

  vpl_mkdir( (tempdir+"\\a").c_str(), 0777);
  touch( (tempdir+"\\a\\123.dat").c_str() );
  touch( (tempdir+"\\a\\123.txt").c_str() );
  touch( (tempdir+"\\a\\13.dat").c_str() );

  vpl_mkdir( (tempdir+"\\b").c_str(), 0777);
  touch( (tempdir+"\\b\\123.dat").c_str() );
  touch( (tempdir+"\\b\\123.txt").c_str() );
  touch( (tempdir+"\\b\\13.dat").c_str() );

  // 0. Check "*"
  {
    vul_file_iterator f( (tempdir+"\\a\\*").c_str() );
    // Assume semantics are such that files appear in order of creation?
    TEST("test_file_iterator 1", f(), vcl_string("./."));
    ++f;
    TEST("test_file_iterator 2", f(), vcl_string("./.."));
    ++f;
    TEST("test_file_iterator 3", f(), vcl_string("./123.dat"));
    ++f;
    TEST("test_file_iterator 4", f(), vcl_string("./123.txt"));
    ++f;
    TEST("test_file_iterator 5", f(), vcl_string("./13.dat"));
    ++f;
    TEST("F ran out 1", !f, true);
  }

  // 1. Check file.*
  {
    vul_file_iterator f( (tempdir+"\\a\\123.*").c_str() );
    // Assume semantics are such that files appear in order of creation?
    TEST("test_file_iterator 6", f(), vcl_string("./123.dat"));
    ++f;
    TEST("test_file_iterator 7", f(), vcl_string("./123.txt"));
    ++f;
    TEST("F ran out 2", !f, true);
  }

  // 2. Check *.ext
  {
    vul_file_iterator f( (tempdir+"\\a\\*.dat").c_str() );
    // Assume semantics are such that files appear in order of creation?
    TEST("test_file_iterator 8", f(), vcl_string("./123.dat"));
    ++f;
    TEST("test_file_iterator 9", f(), vcl_string("./13.dat"));
    ++f;
    TEST("F ran out 3", !f, true);
  }

  // 3. Check ?.*
  {
    vul_file_iterator f( (tempdir+"\\a\\1?3.???").c_str() );
    // Assume semantics are such that files appear in order of creation?
    TEST("test_file_iterator 10", f(), vcl_string("./123.dat"));
    ++f;
    TEST("test_file_iterator 11", f(), vcl_string("./123.txt"));
    ++f;
    TEST("F ran out 4", !f, true);
  }

#if 0 // "*/..." does not work (yet).  See note in vul_file_iterator.cxx
  {
    vul_file_iterator f( (tempdir+"\\*\\123.dat").c_str() );
    TEST("test_file_iterator 12", f(), (tempdir+"\\a\\123.dat").c_str() );
    ++f;
    TEST("test_file_iterator 13", f(), (tempdir+"\\b\\123.dat").c_str() );
    ++f;
    TEST("F ran out 5", !f, true);
  }
#endif

  // 4. Remove and check removal
  vpl_unlink( (tempdir+"\\b\\13.dat").c_str() );
  vpl_unlink( (tempdir+"\\b\\123.txt").c_str() );
  vpl_unlink( (tempdir+"\\b\\123.dat").c_str() );
  {
    vul_file_iterator f( (tempdir+"\\b\\*").c_str() );
    TEST("test_file_iterator 14", f(), vcl_string("./."));
    ++f;
    TEST("test_file_iterator 15", f(), vcl_string("./.."));
    ++f;
    TEST("F ran out 6", !f, true);
  }
  vpl_rmdir( (tempdir+"\\b").c_str() );

  vpl_unlink( (tempdir+"\\a\\13.dat").c_str() );
  vpl_unlink( (tempdir+"\\a\\123.txt").c_str() );
  vpl_unlink( (tempdir+"\\a\\123.dat").c_str() );
  vpl_rmdir( (tempdir+"\\a").c_str() );
  {
    vul_file_iterator f( (tempdir+"\\*").c_str() );
    TEST("test_file_iterator 16", f(), vcl_string("./."));
    ++f;
    TEST("test_file_iterator 17", f(), vcl_string("./.."));
    ++f;
    TEST("F ran out 7", !f, true);
  }

  vpl_rmdir( tempdir.c_str() );
}


void test_vul_file()
{
  // vul_file::basename
  TEST("basename 1", vul_file::basename("fred.txt"), "fred.txt");
  TEST("basename 2", vul_file::basename("/awf/fred.txt"), "fred.txt");
  TEST("basename 3", vul_file::basename("fred.txt", ".txt"), "fred");
  TEST("basename 4", vul_file::basename("/awf/fred.txt", ".txt"), "fred");
  TEST("basename 5", vul_file::basename("t", ".txt"), "t");
  TEST("basename 6", vul_file::basename(".txt", ".txt"), "");
  TEST("basename 7", vul_file::basename("t.txt", ".txt"), "t");
  TEST("basename 8", vul_file::basename(".ttt", ".txt"), ".ttt");
  TEST("basename 9", vul_file::basename("/awf/t", ".txt"), "t");

  // vul_file::dirname
  TEST("dirname 1", vul_file::dirname("fred.txt"), ".");
  TEST("dirname 2", vul_file::dirname("/awf/fred.txt"), "/awf");
}

void test_vul_sleep_timer()
// vul_timer, vpl_sleep
{
  vul_timer tic;
  vpl_sleep(1);
  double t = tic.real() / 1000.0;

  vcl_cout << "vul_timer: sleep lasted " << t << " seconds, expected 1.0\n";

  TEST("Sleep for between 0.5 and 3 seconds", t> 0.5 && t < 3.0, true);
}


void test_file_iterator()
{
#ifndef VCL_WIN32
  test_file_iterator_unix();
#else
  test_file_iterator_dos();
#endif

  test_vul_file();
  test_vul_sleep_timer();
}


TESTMAIN(test_file_iterator);
