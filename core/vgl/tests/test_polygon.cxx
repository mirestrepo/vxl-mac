// Some tests for vgl_polygon
// Amitha Perera, Sep 2001.
#include <testlib/testlib_test.h>
#include <vgl/vgl_polygon.h>
#include <vcl_iostream.h>


void test_simple_polygon()
{
  vcl_cout << "Simple polygon" << vcl_endl;

  // Simple triangle
  vgl_polygon p;
  p.new_sheet();
  p.push_back( 0.0f, 0.0f );
  p.push_back( 5.0f, 0.0f );
  p.push_back( 5.0f, 1.0f );

  TEST("inside",      p.contains(  2.5f,  0.3f ), true );
  TEST("outside (1)", p.contains(  2.5f,  0.6f ), false );
  TEST("outside (2)", p.contains(  5.1f,  0.1f ), false );
  TEST("outside (3)", p.contains(  5.1f,  0.0f ), false );
  TEST("outside (4)", p.contains(  2.0f, -1.0f ), false );
  TEST("outside (5)", p.contains( -2.5f, -0.5f ), false );
}


void test_disjoint_polygon()
{
  vcl_cout << "Disjoint polygons" << vcl_endl;

  // Simple triangle
  vgl_polygon p;
  p.new_sheet();
  p.push_back( 0.0f, 0.0f );
  p.push_back( 5.0f, 0.0f );
  p.push_back( 5.0f, 1.0f );

  // Another disjoint triangle
  p.new_sheet();
  p.push_back( 10.0f, 10.0f );
  p.push_back( 15.0f, 10.0f );
  p.push_back( 15.0f, 11.0f );

  TEST("inside poly1",p.contains(  2.5f,  0.3f ), true );
  TEST("inside poly2",p.contains( 12.5f, 10.3f ), true );
  TEST("outside (1)", p.contains(  2.5f,  0.6f ), false );
  TEST("outside (2)", p.contains(  5.1f,  0.1f ), false );
  TEST("outside (3)", p.contains(  5.1f,  0.0f ), false );
  TEST("outside (4)", p.contains(  2.0f, -1.0f ), false );
  TEST("outside (5)", p.contains( -2.5f, -0.5f ), false );
  TEST("oustide (6)", p.contains( 12.5f, 10.6f ), false );
  TEST("outside (7)", p.contains( 15.1f, 10.0f ), false );
}

void test_holey_polygon()
{
  vcl_cout << "Polygon with holes" << vcl_endl;

  // Simple triangle
  vgl_polygon p;
  p.new_sheet();
  p.push_back( 0.0f, 0.0f );
  p.push_back( 5.0f, 0.0f );
  p.push_back( 5.0f, 1.0f );

  // A hole
  p.new_sheet();
  p.push_back( 3.0f, 0.5f );
  p.push_back( 4.0f, 0.5f );
  p.push_back( 4.0f, 0.1f );

  TEST("inside",      p.contains(  2.5f,  0.3f ), true );
  TEST("oustide (1)", p.contains(  2.5f,  0.6f ), false );
  TEST("outside (2)", p.contains(  5.1f,  0.1f ), false );
  TEST("outside (3)", p.contains(  5.1f,  0.0f ), false );
  TEST("outside (4)", p.contains(  2.0f, -1.0f ), false );
  TEST("outside (5)", p.contains( -2.5f, -0.5f ), false );
  TEST("oustide (6)", p.contains(  3.9f,  0.4f ), false );
}


MAIN( test_polygon )
{
  START( "test polygon" );

  test_simple_polygon();
  test_disjoint_polygon();
  test_holey_polygon();

  SUMMARY();
}
