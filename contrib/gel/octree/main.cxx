//-*- c++ -*-------------------------------------------------------------------
//
// Module: main
// Purpose: None, totally pointless
// Author: Geoffrey Cross, Oxford RRG
// Created: 11 May 99
// Modifications:
//
//-----------------------------------------------------------------------------

#include <vcl_cassert.h>
#include <vcl_cmath.h> // for sqrt()
#include <vcl_cstdlib.h> // for exit()
#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_list.h>
#include <vcl_vector.h>

#include <vbl/vbl_array_2d.h>
#include <vbl/vbl_array_3d.h>
#include <vbl/vbl_sparse_array_1d.h>

#include <vul/vul_arg.h>
#include <vul/vul_sprintf.h>

#include <vnl/vnl_int_3.h>
#include <vnl/vnl_double_3.h>

#include <vgl/vgl_box_2d.h>
#include <vgl/vgl_polygon_scan_iterator.h>

//#include <Geometry/IUPoint.h>
//#include <Topology/Vertex.h>
//#include <CompGeom/Triangulation.h>
//#include <Topology/PolygonScanIterator.h>

#include <vil/vil_load.h>
#include <vil/vil_byte.h>
#include <vil/vil_image.h>
#include <vil/vil_memory_image_of.h>
#include <vil/vil_file_image.h>

#include <mvl/PMatrix.h>

#include "ConvexHull.h"
#include "VoxmapImagePoints.h"
#include "Voxel.h"
#include "BigSparseArray3D.h"
#include "BitArray3D.h"


#define MAXIMAGES 500

vil_memory_image_of<vil_byte> *imagestore[MAXIMAGES];
PMatrix           *pmatrixstore[MAXIMAGES];
vbl_array_2d<short>    *distancestore[MAXIMAGES];
int                updatecounter= 0;

enum cubetest_t
{ outsideimage, surface, inside, outside };


///////////////////////////////////////////////
int minimum5(int a, int b, int c, int d, int e)
{
    if( (a<=b) && (a<=c) && (a<=d) && (a<=e) )
        return(a);
    else if( (b<=c) && (b<=d) && (b<=e) )
        return(b);
    else if( (c<=d) && (c<=e) )
        return(c);
    else if( d<=e )
        return(d);
    else
        return(e);
}

//////////////////////////////////////////////////////////////////////
void computeborgefors( const vbl_array_2d<bool> &edges, vbl_array_2d<short> &distance)
{
  assert( edges.rows()== distance.rows());
  assert( edges.columns()== distance.columns());

  distance.fill( short(vcl_sqrt((double)distance.rows()*distance.rows()+ distance.columns()*distance.columns())+1));

  for( int i=1; i< (edges.rows()-1); i++)
    {
      for( int j=1; j< (edges.columns()-1); j++)
        {
          if( edges(i,j))
            distance(i,j)= 0;

          distance(i,j)= minimum5( distance(i-1,j-1)+4,
                                   distance(i-1,j  )+3,
                                   distance(i-1,j+1)+4,
                                   distance(i  ,j-1)+3,
                                   distance(i  ,j));

        }
    }

  for( int i=(edges.rows()-2); i> 0; i--)
    for( int j=(edges.columns()-2); j> 0; j--)
      distance(i,j)= minimum5( distance(i  ,j  )  ,
                               distance(i  ,j+1)+3,
                               distance(i+1,j-1)+4,
                               distance(i+1,j  )+3,
                               distance(i+1,j+1)+4);

}


///////////////////////////////////////////////////////////////////
void computeedgemap( vil_memory_image_of<vil_byte> imbuf, vbl_array_2d<bool> &edges)
{
  assert( edges.rows()== int(imbuf.width()));
  assert( edges.columns()== int(imbuf.height()));

  edges.fill( false);

  for( int i=1; i< (int(imbuf.width())-1); i++)
    {
      for( int j=1; j< (int(imbuf.height())-1); j++)
        {
          if( !imbuf(i,j) &&
              ( imbuf(i-1,j-1) || imbuf(i,j-1) || imbuf(i+1,j-1) ||
                imbuf(i-1,j  )                 || imbuf(i+1,j) ||
                imbuf(i-1,j+1) || imbuf(i,j+1) || imbuf(i+1,j+1)))
            edges(i,j)= true;
        }
    }
}

///////////////////////////////////////////////////////////////////
double computevoxelradius( VoxmapImagePoints &voxmap, Voxel &voxel, int imageindex)
{
  vnl_double_2 *centre( voxmap.GetCentreImage( voxel.x, voxel.y, voxel.z, voxel.depth, imageindex));
  vnl_double_2 *c1( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 0, 0, 0, voxel.depth, imageindex));
  vnl_double_2 *c2( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 0, 0, 1, voxel.depth, imageindex));
  vnl_double_2 *c3( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 0, 1, 0, voxel.depth, imageindex));
  vnl_double_2 *c4( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 0, 1, 1, voxel.depth, imageindex));
  vnl_double_2 *c5( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 1, 0, 0, voxel.depth, imageindex));
  vnl_double_2 *c6( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 1, 0, 1, voxel.depth, imageindex));
  vnl_double_2 *c7( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 1, 1, 0, voxel.depth, imageindex));
  vnl_double_2 *c8( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 1, 1, 1, voxel.depth, imageindex));

  double dist= (*c1-*centre).squared_magnitude();
  double d2  = (*c2-*centre).squared_magnitude(); if( d2> dist) dist= d2;
  double d3  = (*c3-*centre).squared_magnitude(); if( d3> dist) dist= d3;
  double d4  = (*c4-*centre).squared_magnitude(); if( d4> dist) dist= d4;
  double d5  = (*c5-*centre).squared_magnitude(); if( d5> dist) dist= d5;
  double d6  = (*c6-*centre).squared_magnitude(); if( d6> dist) dist= d6;
  double d7  = (*c7-*centre).squared_magnitude(); if( d7> dist) dist= d7;
  double d8  = (*c8-*centre).squared_magnitude(); if( d8> dist) dist= d8;

  return vcl_sqrt(dist);
}

///////////////////////////////////////////////////////////////////
cubetest_t DoScan( VoxmapImagePoints &voxmap, Voxel &voxel, int imageindex)
{
  vnl_double_2 *c1( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 0, 0, 0, voxel.depth, imageindex));
  vnl_double_2 *c2( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 0, 0, 1, voxel.depth, imageindex));
  vnl_double_2 *c3( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 0, 1, 0, voxel.depth, imageindex));
  vnl_double_2 *c4( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 0, 1, 1, voxel.depth, imageindex));
  vnl_double_2 *c5( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 1, 0, 0, voxel.depth, imageindex));
  vnl_double_2 *c6( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 1, 0, 1, voxel.depth, imageindex));
  vnl_double_2 *c7( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 1, 1, 0, voxel.depth, imageindex));
  vnl_double_2 *c8( voxmap.GetCornerImage( voxel.x, voxel.y, voxel.z, 1, 1, 1, voxel.depth, imageindex));

  ConvexHull cv( 8);
  cv.set_point( 0, (*c1)[0], (*c1)[1]);
  cv.set_point( 1, (*c2)[0], (*c2)[1]);
  cv.set_point( 2, (*c3)[0], (*c3)[1]);
  cv.set_point( 3, (*c4)[0], (*c4)[1]);
  cv.set_point( 4, (*c5)[0], (*c5)[1]);
  cv.set_point( 5, (*c6)[0], (*c6)[1]);
  cv.set_point( 6, (*c7)[0], (*c7)[1]);
  cv.set_point( 7, (*c8)[0], (*c8)[1]);
  cv.compute();

  vcl_vector<vgl_point_2d<float> > points;
  //  vgl_polygon_scan_iterator::Point2 points[cv.get_npoints()];
  vgl_box_2d<float> box( 0, imagestore[imageindex]->width()-1, 0, imagestore[imageindex]->height()-1);

  for( int i=0; i< cv.get_npoints(); i++)
    {
      vgl_point_2d<float> f( cv.get_pointx( i),
                             cv.get_pointy( i));

      points.push_back( f);
    }

  //  box.Protect();
  vgl_polygon polygon( points);
  vgl_polygon_scan_iterator polyscan( polygon, false, box);
  //  box.UnProtect();

  // 0 = nothing expected yet... no scan points
  // 1 = inside expected and got
  // 2 = outside expected and got
  // 3 = conflict
  int expecting=0;

  for( polyscan.reset() ; (polyscan.next()) && (expecting!= 3) ; )
    {
      int y= polyscan.scany();

      for( int x=polyscan.startx(); (x<= polyscan.endx()) && (expecting!=3); x++)
        {
          vil_byte pix= (*imagestore[imageindex])(int(x),int(y));
          int t;
          if( pix) t= 1;
          else t= 2;

          if( expecting==0)
            expecting= t;
          else
            if( expecting!= t)
              expecting= 3;
        }
    }

  if( expecting== 3)
    return surface;
  else if( expecting== 2)
    return outside;
  else if( expecting== 1)
    return inside;

  return outsideimage;
}

////////////////////////////////
int main(int argc, char ** argv)
{
  vul_arg<char *> colorimagefilename( "-c", "Color image filenames (only for pipe)", 0);
  vul_arg<char *> imagefilename( "-i", "Thresholded images", "/home/crossge/images/dino/full/thresh.%03d.pgm");
  vul_arg<char *> pmatrixfilename( "-p", "PMatrix basename", "/home/crossge/images/dino/full/viff.%03d.sa.P");
  vul_arg<char *> outputfilename( "-o", "Piped output filename", 0);
  vul_arg<char *> outputvrml( "-v", "Output VRML filename", "/tmp/out.wrl");
  vul_arg<int>    iterations( "-s", "Number of iterations", 5);
  vul_arg<int>    startiteration( "-start", "Starting iteration", 0);
  vul_arg<double> fiddlefactor( "-fiddle", "Fiddle factor for cube radii", 1.5);
  vul_arg<char *> voxmapfilename( "-vf", "Voxmap filename", 0);

 //  Arg<double> sx( "-sx", "Starting x position" , -0.0011);
//   Arg<double> sy( "-sy", "Starting y position" , -0.0226);
//   Arg<double> sz( "-sz", "Starting z position" ,  0.5607);

  vul_arg<double> sx( "-sx", "Starting x position" , -0.0044);
  vul_arg<double> sy( "-sy", "Starting y position" , -0.0136);
  vul_arg<double> sz( "-sz", "Starting z position" ,  0.5764);

  //  Arg<double> ss( "-ss", "Starting size" , 0.4);
  vul_arg<double> ss( "-ss", "Starting size" , 0.2);

  vul_arg<vcl_list<int> > imagenumbersarg( "-n", "Image numbers");

  vul_arg_parse( argc, argv);

  // Create voxmap
  VoxmapImagePoints voxmap(iterations()-1, vnl_double_3( double(sx()), double(sy()), double(sz())), double(ss()), MAXIMAGES);

  ///////////////////////////////////////////////////////////////////

  vcl_ofstream *fout= NULL;

  if( ( char *)( outputfilename()))
    {
      vcl_cerr << "Opening pipe..." << vcl_endl;
      fout= new vcl_ofstream( outputfilename());
    }
  else
    {
      vcl_cerr << "Not opening pipe..." << vcl_endl;
    }

  ///////////////////////////////////////////////////////////////////

  // useful otherwise we have to cast every time we want to use it
  vcl_list<int> imagenumbers( imagenumbersarg());

  // all the pre-computation setup
  for( vcl_list<int>::iterator it= imagenumbers.begin(); it!= imagenumbers.end(); ++it)
    {
      vcl_cerr << "Loading image : " << *it << vcl_endl;

      // load all images and pmatrices
      vil_image image= vil_load( vul_sprintf(( char *) imagefilename(), *it));
      imagestore[*it]= new vil_memory_image_of<vil_byte>( image);
      assert( *imagestore[*it]);

      vcl_ifstream pmatrixin( vul_sprintf(( char *) pmatrixfilename(), *it));
      assert( pmatrixin.good());
      pmatrixstore[*it]= new PMatrix;
      pmatrixstore[*it]->read_ascii( pmatrixin);

      vcl_cerr << "Computing borgefor transform..." << vcl_endl;

      // compute borgefors transform
      vbl_array_2d<bool> edges( imagestore[*it]->width(),
                           imagestore[*it]->height());
      computeedgemap( *imagestore[*it], edges);
      distancestore[*it]= new vbl_array_2d<short>( imagestore[*it]->width(),
                                                               imagestore[*it]->height());
      computeborgefors(  edges,
                        *distancestore[*it]);

      // setup voxmap projection cache
      voxmap.SetPMatrix( *pmatrixstore[*it], *it);
    }

  ///////////////////////////////////////////////////////////////////

  vcl_vector<Voxel> voxels;
  vcl_vector<Voxel> insidevoxels;
  //  CoolArray<Voxel> voxels;
  //  CoolArray<Voxel> insidevoxels;

  {
    int d= 1<< int(startiteration());

    for( int x=0; x< d; x++)
      {
        for( int y=0; y< d; y++)
          {
            for( int z=0; z< d; z++)
              {
                voxels.push_back( Voxel(int(startiteration()),x,y,z));
              }
          }
      }
  }

  for( int count=int(startiteration()); count< int(iterations()); count++)
    {
      vcl_cerr << "Iterations to do: " << (int(iterations())-count) << vcl_endl;

      if( fout)
        {
          (*fout) << "c" << vcl_endl;

          for( int i=0; i< voxels.size(); i++)
            (*fout) << "a" << voxels[i] << vcl_endl;
        }

      ///////////////////////////////////////////////////////////////////
      bool outsideobject[voxels.size()];
      bool insideimage  [voxels.size()];
      bool insideobject [voxels.size()];

      vcl_cerr << "Setting up variables... ";

      // note: this assumes that the radius of each voxel in each image is
      //       approximately the same.  Not true in general, but for the turntabl
      //       case this is 'ok'
      Voxel tempvoxel( count,
                       int((1<<count)/2),
                       int((1<<count)/2),
                       int((1<<count)/2));

      double radius = computevoxelradius( voxmap, tempvoxel,*(imagenumbers.begin()))* double(fiddlefactor());
      //          radius       [i]= computevoxelradius( voxmap, voxels[i], imagenumbers[0])*1.6;

      for( int i=0; i< voxels.size(); i++)
        {
          outsideobject[i]= false;
          insideimage  [i]= true;
          insideobject [i]= true;
        }

      vcl_cerr << "Done. " << vcl_endl;

      for( vcl_list<int>::iterator it= imagenumbers.begin(); it!= imagenumbers.end(); ++it)
        {
          vcl_cerr << *it << " ";

          if( fout)
            {
              if( ( char *) colorimagefilename())
                {
                  (*fout) << "i" << vul_sprintf( ( char *) colorimagefilename(), *it) << vcl_endl;
                  (*fout) << "u" << vcl_endl;
                }
            }

          int xsize= imagestore[*it]->width();
          int ysize= imagestore[*it]->height();

          for( int i=0; i< voxels.size(); i++)
            {
              if( !outsideobject[i] && insideimage[i])
                {
                  //          double radius ( computevoxelradius( voxmap, voxels[i], *it));
                  vnl_double_2 centre( *voxmap.GetCentreImage( voxels[i].x,
                                                          voxels[i].y,
                                                          voxels[i].z,
                                                          voxels[i].depth,
                                                          *it));

                  // centre is outside the image
                  if(( centre[0]< 0) || ( centre[1]< 0) || ( centre[0]>= xsize) || (centre[1]>= ysize))
                    {
                      double disttoim= vnl_double_2( double(xsize)/2-centre[0], double(ysize)/2-centre[1]).magnitude();

                      // cube does actually look 'close' to the image
                      if( disttoim< (radius+(vcl_sqrt((double)xsize*xsize/4+ysize*ysize/4))))
                        {
                          cubetest_t t= DoScan( voxmap, voxels[i], *it);

                          if( t== outsideimage)
                            insideimage[i]= false;

                          // outsideimage, surface, inside, outside
                          if( t== outside)
                            {
                              if( fout) (*fout) << "d " << i << vcl_endl;

                              outsideobject[i]= true;
                            }
                          else if( t== surface)
                            insideobject[i] = false;
                        } // inside image, we think
                      else
                        insideimage[i]= false;

                    } // if centre outside image
                  else
                    {
                      // either completely inside or completely outside object
                      if( (double((*distancestore[*it])(centre[0],centre[1]))/3)> radius)
                        {
                          // if completely outside object
                          if( !(*imagestore[*it])(centre[0],centre[1]))
                            {
                              if( fout)
                                {
                                  (*fout) << "d " << i << vcl_endl;
                                  if( updatecounter++>= (2<<count))
                                    {
                                      (*fout) << "u" << vcl_endl;
                                      updatecounter= 0;
                                    }
                                }

                              outsideobject[i]= true;
                            }
                        }
                      else
                        {
                          if( count== int(iterations())-1)
                            {
                              // if close to surface
                              cubetest_t t= DoScan( voxmap, voxels[i], *it);

                              if( t== outside)
                                {
                                  if( fout) (*fout) << "d " << i << vcl_endl;

                                  outsideobject[i]= true;
                                }
                              else if( t== surface)
                                insideobject[i]= false;
                            }
                          else
                            {
                              insideobject[i]= false;
                            }
                        }
                    } // if centre inside image
                } // if( !outsideobject[i])
            } // for( int i=0; i< voxels.length(); i++)
        } // for( imagenumbers.reset(); imagenumbers.next(); )

      vcl_cerr << vcl_endl;

      vcl_vector<Voxel> newvoxels;
      //      CoolArray<Voxel> newvoxels;

      vcl_cerr << "No voxels = " << voxels.size() << vcl_endl;
      vcl_cerr << "Making new level..." << vcl_endl;

      for( int i=0; i< voxels.size(); i++)
        {
          //      if( !outsideobject[i] && insideimage[i] && !insideobject[i])
          if( !outsideobject[i] && insideimage[i] && !insideobject[i])
            {
              if( count!= (int(iterations())-1))
                {
                  newvoxels.push_back( Voxel( voxels[i].depth+1, voxels[i].x*2  , voxels[i].y*2  , voxels[i].z*2  ));
                  newvoxels.push_back( Voxel( voxels[i].depth+1, voxels[i].x*2  , voxels[i].y*2  , voxels[i].z*2+1));
                  newvoxels.push_back( Voxel( voxels[i].depth+1, voxels[i].x*2  , voxels[i].y*2+1, voxels[i].z*2  ));
                  newvoxels.push_back( Voxel( voxels[i].depth+1, voxels[i].x*2  , voxels[i].y*2+1, voxels[i].z*2+1));
                  newvoxels.push_back( Voxel( voxels[i].depth+1, voxels[i].x*2+1, voxels[i].y*2  , voxels[i].z*2  ));
                  newvoxels.push_back( Voxel( voxels[i].depth+1, voxels[i].x*2+1, voxels[i].y*2  , voxels[i].z*2+1));
                  newvoxels.push_back( Voxel( voxels[i].depth+1, voxels[i].x*2+1, voxels[i].y*2+1, voxels[i].z*2  ));
                  newvoxels.push_back( Voxel( voxels[i].depth+1, voxels[i].x*2+1, voxels[i].y*2+1, voxels[i].z*2+1));
                }
              else
                {
                  newvoxels.push_back( voxels[i]);
                }
            }
          else
            {
              if(( insideobject[i]) && ( !outsideobject[i]) && ( insideimage[i]))
                insidevoxels.push_back( voxels[i]);

              if( fout) (*fout) << "d " << i << vcl_endl;
            }
        }

      vcl_cerr << "Copying..." << vcl_endl;

      //      if( count!= (int(iterations)-1))
      voxels= newvoxels;
    } // for( int i=0; i< int(iterators); i++)

  vcl_cerr << "Deleting images and pmatrices..." << vcl_endl;

  ///////////////////////////////////////////////////////////////////
  // delete all images and pmatrices
  for( vcl_list<int>::iterator it= imagenumbers.begin(); it!= imagenumbers.end(); ++it)
    {
      delete imagestore[*it];
      delete pmatrixstore[*it];
      delete distancestore[*it];
    }

  ///////////////////////////////////////////////////////////////////
  // save to a voxmap file if necessary

  if(( char *) voxmapfilename())
    {
      int size= 1<<(int(iterations())-1);
      BitArray3D v( size, size, size, false);
      for( int i=0; i< voxels.size(); i++)
        {
          v.set( voxels[i].x, voxels[i].y, voxels[i].z, true);
        }

      for( int i=0; i< insidevoxels.size(); i++)
        {
          int depth= 1<<((int(iterations())-1)-insidevoxels[i].depth);

          int startx= insidevoxels[i].x*depth;
          int starty= insidevoxels[i].y*depth;
          int startz= insidevoxels[i].z*depth;

          int endx= (insidevoxels[i].x+1)*depth;
          int endy= (insidevoxels[i].y+1)*depth;
          int endz= (insidevoxels[i].z+1)*depth;

          for( int x=startx; x< endx; x++)
            {
              for( int y=starty; y< endy; y++)
                {
                  for( int z=startz; z< endz; z++)
                    {
                      v.flip( x,y,z);
                    }
                }
            }
        }

      vcl_ofstream fout(( char *) voxmapfilename());
      fout << double(ss()) << " " << double(sx()) << " " << double(sy()) << " " << double(sz()) << vcl_endl;
      fout << size << vcl_endl;

      for( int x=0; x< size; x++)
        {
          for( int y=0; y< size; y++)
            {
              for( int z=0; z< size; z++)
                {
                  fout << v(x,y,z);
                }
            }
        }

      vcl_exit(0);
    }

  ///////////////////////////////////////////////////////////////////
  // make VRML

  vcl_cerr << "Making VRML..." << vcl_endl;

  int size= 1<<(int(iterations())-1);
  BitArray3D edges1( size, size, size+1);
  BitArray3D edges2( size, size+1, size);
  BitArray3D edges3( size+1, size, size);
  BitArray3D voxint( size, size, size);

  voxint.fill(false);
  edges1.fill(false);
  edges2.fill(false);
  edges3.fill(false);

  vcl_cerr << "Surface voxels..." << vcl_endl;
  for( int i=0; i< voxels.size(); i++)
    {
      voxint.set( voxels[i].x, voxels[i].y, voxels[i].z, true);

      edges1.flip( voxels[i].x, voxels[i].y, voxels[i].z);
      edges1.flip( voxels[i].x, voxels[i].y, voxels[i].z+1);

      edges2.flip( voxels[i].x, voxels[i].y, voxels[i].z);
      edges2.flip( voxels[i].x, voxels[i].y+1, voxels[i].z);

      edges3.flip( voxels[i].x, voxels[i].y, voxels[i].z);
      edges3.flip( voxels[i].x+1, voxels[i].y, voxels[i].z);
    }

  vcl_cerr << "Interior voxels..." << vcl_endl;
  for( int i=0; i< insidevoxels.size(); i++)
    {
      int depth= 1<<((int(iterations())-1)-insidevoxels[i].depth);

      int startx= insidevoxels[i].x*depth;
      int starty= insidevoxels[i].y*depth;
      int startz= insidevoxels[i].z*depth;

      int endx= (insidevoxels[i].x+1)*depth;
      int endy= (insidevoxels[i].y+1)*depth;
      int endz= (insidevoxels[i].z+1)*depth;

      for( int x=startx; x< endx; x++)
        {
          for( int y=starty; y< endy; y++)
            {
              for( int z=startz; z< endz; z++)
                {
                  voxint.set( x,y,z, true);

                  edges1.flip(x,y,z);
                  edges1.flip(x,y,z+1);

                  edges2.flip(x,y,z);
                  edges2.flip(x,y+1,z);

                  edges3.flip(x,y,z);
                  edges3.flip(x+1,y,z);
                }
            }
        }
    }

  vcl_cerr << "Clearing up..." << vcl_endl;
  voxels.clear();
  insidevoxels.clear();

  vcl_cerr << "Output VRML..." << vcl_endl;

  vbl_sparse_array_1d<int>      indexlist;
  vcl_vector<vnl_double_3 *> points;
  vcl_vector<vnl_int_3> faces;
  //  CoolArrayP<vnl_double_3 *> points;
  //  CoolArray<vnl_int_3>       faces;

  vcl_cerr << "z - direction" << vcl_endl;

  // z direction
  for( int x=0; x< size; x++)
    {
      for( int y=0; y< size; y++)
        {
          for( int z=1; z< size; z++)
            {
              if( edges1(x,y,z))
                {
                  int i[4];
                  int ii[4];

                  for( int k=0; k<4; k++)
                    {
                      int q1=0,q2=0,q3=0;

                      switch( k)
                        {
                        case 0:
                          {
                            break;
                          }
                        case 1:
                          {
                            q1=1;
                            break;
                          }
                        case 2:
                          {
                            q1=1;q2=1;
                            break;
                          }
                        case 3:
                          {
                            q2=1;
                            break;
                          }
                        }

                      i[k]= voxmap.GetCornerIndex( x,y,z, q1,q2,q3, int(iterations())-1);

                      if( !indexlist.fullp( i[k]))
                        {
                          points.push_back( voxmap.GetCorner( x,y,z, q1,q2,q3, int(iterations())-1));
                          indexlist[ i[k]]= points.size()-1;
                          ii[k]= points.size()-1;
                        }
                      else
                        ii[k]= indexlist[ i[k]];
                    }

                  if( voxint( x,y,z))
                    {
                      faces.push_back( vnl_int_3( ii[2], ii[1], ii[0]));
                      faces.push_back( vnl_int_3( ii[3], ii[2], ii[0]));
                    }
                  else
                    {
                      faces.push_back( vnl_int_3( ii[0], ii[1], ii[2]));
                      faces.push_back( vnl_int_3( ii[0], ii[2], ii[3]));

                    }
                }
            }
        }
    }

  vcl_cerr << "y - direction" << vcl_endl;

  // y direction
  for( int x=0; x< size; x++)
    {
      for( int y=1; y< size; y++)
        {
          for( int z=0; z< size; z++)
            {
              if( edges2(x,y,z))
                {
                  int i[4];
                  int ii[4];

                  for( int k=0; k<4; k++)
                    {
                      int q1=0,q2=0,q3=0;

                      switch( k)
                        {
                        case 0:
                          {
                            break;
                          }
                        case 1:
                          {
                            q1=1;
                            break;
                          }
                        case 2:
                          {
                            q1=1;q2=1;
                            break;
                          }
                        case 3:
                          {
                            q2=1;
                            break;
                          }
                        }

                      i[k]= voxmap.GetCornerIndex( x,y,z, q1,q3,q2, int(iterations())-1);

                      if( !indexlist.fullp( i[k]))
                        {
                          points.push_back( voxmap.GetCorner( x,y,z, q1,q3,q2, int(iterations())-1));
                          indexlist[ i[k]]= points.size()-1;
                          ii[k]= points.size()-1;
                        }
                      else
                        ii[k]= indexlist[ i[k]];
                    }

                  if( voxint( x,y,z))
                    {
                      faces.push_back( vnl_int_3( ii[0], ii[1], ii[2]));
                      faces.push_back( vnl_int_3( ii[0], ii[2], ii[3]));
                    }
                  else
                    {
                      faces.push_back( vnl_int_3( ii[2], ii[1], ii[0]));
                      faces.push_back( vnl_int_3( ii[3], ii[2], ii[0]));
                    }
                }
            }
        }
    }

  vcl_cerr << "x - direction" << vcl_endl;

  // x direction
  for( int x=1; x< size; x++)
    {
      for( int y=0; y< size; y++)
        {
          for( int z=0; z< size; z++)
            {
              if( edges3(x,y,z))
                {
                  int i[4];
                  int ii[4];

                  for( int k=0; k<4; k++)
                    {
                      int q1=0,q2=0,q3=0;

                      switch( k)
                        {
                        case 0:
                          {
                            break;
                          }
                        case 1:
                          {
                            q1=1;
                            break;
                          }
                        case 2:
                          {
                            q1=1;q2=1;
                            break;
                          }
                        case 3:
                          {
                            q2=1;
                            break;
                          }
                        }

                      i[k]= voxmap.GetCornerIndex( x,y,z, q3,q2,q1, int(iterations())-1);

                      if( !indexlist.fullp( i[k]))
                        {
                          points.push_back( voxmap.GetCorner( x,y,z, q3,q2,q1, int(iterations())-1));
                          indexlist[ i[k]]= points.size()-1;
                          ii[k]= points.size()-1;
                        }
                      else
                        ii[k]= indexlist[ i[k]];
                    }

                  if( voxint( x,y,z))
                    {
                      faces.push_back( vnl_int_3( ii[0], ii[1], ii[2]));
                      faces.push_back( vnl_int_3( ii[0], ii[2], ii[3]));
                    }
                  else
                    {
                      faces.push_back( vnl_int_3( ii[2], ii[1], ii[0]));
                      faces.push_back( vnl_int_3( ii[3], ii[2], ii[0]));
                    }
                }
            }
        }
    }

  ///////////////////////////////////////////////////////////////////////////

  vcl_ofstream vfout(( char *) outputvrml());
  assert( vfout.good());

  vfout << "#VRML V1.0 ascii\n\nSeparator {\nCoordinate3 { point [\n";

  for( int i=0; i< points.size(); i++)
    {
      vfout << (*points[i])[0] << " "
            << (*points[i])[1] << " "
            << (*points[i])[2] << "," << vcl_endl;
    }

  vfout << "]}\nIndexedFaceSet { coordIndex [\n";

  for( int i=0; i< faces.size(); i++)
    {
      vfout << faces[i][0] << ", "
            << faces[i][1] << ", "
            << faces[i][2] << ", -1," << vcl_endl;
    }

  vfout << "]}}" << vcl_endl;

  vfout.close();

  ///////////////////////////////////////////////////////////////////
  // close output file
  if( fout)
    delete fout;
}

