/*
  crossge@crd.ge.com
*/
#ifdef __GNUC__
#pragma implementation
#endif
#include "gst_polygon_2d_operators.h"




vcl_vector<gst_polygon_2d_ref> gst_make_polygons_2d( const vcl_vector<gst_edge_2d_ref> edges)
{
  // flags showing edges already used
  vcl_vector<int> used( edges.size(), 0);
  
  // repository of polygons as they are created
  vcl_vector<gst_polygon_2d_ref> polygons;

  // start a polygon with each edge, and look for a closed cycle
  //  hopefully using a NEW edge
  for( int i=0; i< edges.size(); i++)
    {
      bool newface= false;
      bool closed= false;
      gst_polygon_2d_ref thispoly= new gst_polygon_2d;

      // flags showing edges already used in this polygon
      vcl_vector<int> pused( edges.size(), 0);

      thispoly->add( edges[i]);

      if( !used[i]) 
	{
	  newface= true;
	}

      used[i]= 1;
      pused[i]= 1;

      gst_vertex_2d_ref start= edges[i]->get_start();
      gst_vertex_2d_ref end  = edges[i]->get_end();

      // repeatedly look for the next edge in the cycle
      //  until we do a complete pass without finding any further
      //  edges
      bool added= true;

      while(( added) && ( !closed))
	{
	  added= false;

	  for( int j=0; ((j< edges.size()) && (!closed)); j++)
	    {
	      if(( edges[j]->get_start().ptr()== end.ptr()) && ( !pused[j]))
		{
		  thispoly->add( edges[j]);
		  added= true;

		  end= edges[j]->get_end();
		  
		  if( !used[j]) newface= true;

		  used[j]= 1;
		  pused[j]= 1;
		  
		  if( end.ptr()== start.ptr())
		    {
		      closed= true;
		    }
		}
	    }
	}

      if(( newface) && ( closed))
	{
	  polygons.push_back( thispoly);
	}
    }

  return polygons;
}


vcl_vector<gst_polygon_2d_ref> gst_make_polygons_2d_unoriented( const vcl_vector<gst_edge_2d_ref> edges)
{
  // flags showing edges already used
  vcl_vector<int> used( edges.size(), 0);
  
  // repository of polygons as they are created
  vcl_vector<gst_polygon_2d_ref> polygons;

  // start a polygon with each edge, and look for a closed cycle
  //  hopefully using a NEW edge
  for( int i=0; i< edges.size(); i++)
    {
      bool newface= false;
      bool closed= false;
      gst_polygon_2d_ref thispoly= new gst_polygon_2d;

//       cerr << "Starting face by adding edge" << endl;
//       cerr << *edges[i] << endl;
      thispoly->add( edges[i]);

      if( !used[i]) 
	{
	  newface= true;
	}

      used[i]= 1;

      gst_vertex_2d_ref start= edges[i]->get_start();
      gst_vertex_2d_ref end  = edges[i]->get_end();

      // repeatedly look for the next edge in the cycle
      //  until we do a complete pass without finding any further
      //  edges
      bool added= true;

      while(( added) && ( !closed))
	{
	  added= false;

	  for( int j=0; ((j< edges.size()) && (!closed)); j++)
	    {
	      if(( edges[j]->get_start().ptr()== end.ptr()) && ( !used[j]))
		{
// 		  cerr << "Found unflip-necessary edge..." << endl;
// 		  cerr << *edges[j] << endl;

		  thispoly->add( edges[j]);
		  added= true;

		  end= edges[j]->get_end();
		  
		  if( !used[j]) newface= true;

		  used[j]= 1;
		  
		  if( end.ptr()== start.ptr())
		    {
		      closed= true;
		    }
		}
	      else if(( edges[j]->get_end().ptr()== end.ptr()) && ( !used[j])) 
		{
// 		  cerr << "Found flip-necessary edge..." << endl;
// 		  cerr << *edges[j] << " -- ";

		  edges[j]->flip();

		  //		  cerr << *edges[j] << endl;

		  thispoly->add( edges[j]);
		  added= true;

		  end= edges[j]->get_end();
		  
		  if( !used[j]) newface= true;

		  used[j]= 1;
		  
		  if( end.ptr()== start.ptr())
		    {
		      closed= true;
		    }
		}

	    }
	}

      if(( newface) && ( closed))
	{
	  polygons.push_back( thispoly);
	}
    }

//   for( int i=0; i< polygons.size(); i++)
//     {
//       gst_polygon_2d *p= polygons[i].ptr();
//       cerr << "Polygon " << i << endl;
//       cerr << *p << endl;
//     }

  return polygons;
}
