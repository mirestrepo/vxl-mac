// This is gel/vtol/tests/vtol_test_timing.cxx
#include <testlib/testlib_test.h>
#include <vul/vul_timer.h>
#include <vtol/vtol_vertex_2d_sptr.h>
#include <vtol/vtol_vertex_2d.h>
#include <vtol/vtol_edge_sptr.h>
#include <vtol/vtol_edge_2d.h>
#include <vtol/vtol_face_sptr.h>
#include <vtol/vtol_face_2d.h>

static void vtol_test_timing()
{
  vcl_cout << "testing topology timing\n";

  vul_timer t;
  // make a network of s^2 vertices and edges on an s by s grid
  const int s = 100;
  // make the vertices
  vcl_vector<vtol_vertex_2d_sptr> verts(s*s);
  t.mark();
  for (int y = 0, off = 0; y<s; ++y)
    {
      for (int x = 0; x<s; ++x, ++off)
        {
          verts[off] = new vtol_vertex_2d(x,y);
        }
    }
  vcl_cout << "Time to construct " << s*s << " vertices " << t.real()
           << " msecs\n" << vcl_flush;

  // make the edges
  vcl_vector<vtol_edge_sptr> edges(2*s*s);
  int nedges = 0;
  t.mark();
  for (int y = 0, yoff = 1; y<s; ++y, yoff+=s)
    {
     for (int x = 1, off = yoff; x<s; ++x, ++off)
        {
          edges[nedges++] = new vtol_edge_2d(verts[off-1], verts[off]);
        }
    }
  for (int x = 0; x<s; ++x)
    {
     for (int y = 1, off = x+s; y<s; ++y, off+=s)
        {
          edges[nedges++] = new vtol_edge_2d(verts[off-s], verts[off]);
        }
    }
  vcl_cout << "Time to construct " << nedges << " edges " << t.real()
           << " msecs\n" << vcl_flush;
  TEST("number of edges", nedges, 2*(s-1)*s);

  // count the outgoing edges of all vertices
  t.mark();
  int n_vedges = 0;
  for (vcl_vector<vtol_vertex_2d_sptr>::iterator vit = verts.begin();
       vit != verts.end(); vit++)
    {
      vcl_vector<vtol_edge_sptr>* vedges = (*vit)->edges();
      n_vedges += vedges->size();
      delete vedges;
    }
  vcl_cout << "Time to count edges per vertex " << t.real()
           << " msecs\n" << vcl_flush;
  TEST("number of edges per vertex", n_vedges, 4*(s-1)*s);

  // make the faces
  vcl_vector<vtol_face_sptr> faces(s*s);
  int nfaces = 0;
  t.mark();
  for (int y = 1; y<s; ++y)
    {
     int yoff = s*y;
     for (int x = 1; x<s; ++x)
       {
         int off = x+yoff;
         vertex_list l(4);
         l[0] = verts[off]->cast_to_vertex();
         l[1] = verts[off-s]->cast_to_vertex();
         l[2] = verts[off-s-1]->cast_to_vertex();
         l[3] = verts[off-1]->cast_to_vertex();
         faces[nfaces++]= new vtol_face_2d(l);
      }
    }
  vcl_cout << "Time to construct " << nfaces << " faces " << t.real()
           << " msecs\n" << vcl_flush;
  TEST("number of faces", nfaces, (s-1)*(s-1));

  // count the faces adjacent to all vertices
  t.mark();
  int n_vfaces = 0;
  for (vcl_vector<vtol_vertex_2d_sptr>::iterator vit = verts.begin();
       vit != verts.end(); vit++)
    {
      vcl_vector<vtol_face_sptr>* vfaces = (*vit)->faces();
      n_vfaces += vfaces->size();
      delete vfaces;
    }
  vcl_cout << "Time to count faces per vertex " << t.real() << " msecs\n";
  TEST("number of faces per vertex", n_vfaces, 4*(s-1)*(s-1));
}

TESTLIB_DEFINE_MAIN(vtol_test_timing);
