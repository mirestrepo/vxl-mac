#include <vcl_vector.h>
#include <vcl_algorithm.h>
#include <vcl_iostream.h>

#include <vnl/vnl_vector_fixed.h>
#include <vnl/vnl_double_2.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_math.h>
#include <testlib/testlib_test.h>

#include <mbl/mbl_mz_random.h>

#include <rsdl/rsdl_bins.h>

// Test 2D bins
static
void
test_bins_2()
{
  vcl_cout << "Testing 2D bins\n";
  typedef rsdl_bins< 2, double, int > bin_type;

  vnl_vector_fixed< double, 2 > min_pt, max_pt, bin_sizes;
  min_pt.x() = 1.0;   min_pt.y() = -2.5;
  max_pt.x() = 75.0;  max_pt.y() = 32;
  bin_sizes.x() = 6; bin_sizes.y() = 4;

  bin_type bins( min_pt, max_pt, bin_sizes );
  double dist_tol = 0.0001;
  bins.set_distance_tolerance( dist_tol );
  vnl_vector_fixed< double, 2 >  pt( 12.5, 3 );
  int value = 0;
  bins.add_point( pt, value ); // should be 1, 0

  int stored_value;

  pt.x() = 100; pt.y() = -30;
  int added_value = 24;
  testlib_test_begin( "adding point outside range 1" );
  bins.add_point( pt, added_value );
  testlib_test_perform( bins.get_value( pt, stored_value )
                        && stored_value == added_value );

  pt.x() = -50; pt.y() = 77.7;
  added_value = 13;
  testlib_test_begin( "adding point outside range 2" );
  bins.add_point( pt, added_value );
  testlib_test_perform( bins.get_value( pt, stored_value )
                        && stored_value == added_value );

  {
    //  Generate a bunch of points:
    const int M=60;
    vcl_vector< vnl_vector_fixed< double, 2 > > points( M );
    vcl_vector< int > indices( M );
    mbl_mz_random mz_rand;
    bin_type bins( min_pt, max_pt, bin_sizes );

    for ( int i=0; i<M; ++i ) {
      indices[ i ] = i;
      points[ i ].x() = (max_pt.x() - min_pt.x()) * mz_rand.drand32() + min_pt.x();
      points[ i ].y() = (max_pt.y() - min_pt.y()) * mz_rand.drand32() + min_pt.y();
      bins.add_point( points[ i ], i );
    }

  //  test  "is_any_point_within_radius" and "points_with_radius"

    const int num_tests = 5;
    for ( int t=0; t<num_tests; ++t ) {
      vnl_vector_fixed< double, 2 > q;
      q.x() = (max_pt.x() - min_pt.x()) * mz_rand.drand32() + min_pt.x();
      q.y() = (max_pt.y() - min_pt.y()) * mz_rand.drand32() + min_pt.y();
      double radius = (t+1) * bin_sizes.x() / 2;  // increasing size, starting smallest

      bool bin_answer = bins.is_any_point_within_radius( q, radius );
    
      vcl_vector< int > all_close_indices;
      for ( int i=0; i<M; ++i ) {
        if ( vnl_vector_ssd( points[ i ], q ) < vnl_math_sqr( radius ) ) {
          all_close_indices.push_back( i );
        }
      }

      testlib_test_begin( "is_any_point_within_radius" );
      testlib_test_perform( bin_answer == (all_close_indices.size() > 0) );
      
      vcl_vector< int > bin_close_indices;
      bins.points_within_radius( q, radius, bin_close_indices );
      bool correct = all_close_indices.size() ==  bin_close_indices.size();
      vcl_sort( all_close_indices.begin(), all_close_indices.end() );
      vcl_sort( bin_close_indices.begin(), bin_close_indices.end() );

      for ( unsigned int i=0; correct &&
              i < vnl_math_min(all_close_indices.size(), bin_close_indices.size()); ++i )
        correct = all_close_indices[ i ] == bin_close_indices[ i ];

      testlib_test_begin( "points_within_radius" );
      testlib_test_perform( correct );
    }
  }

  { //new test with bin size 5,5
    vnl_double_2 a(0,0), b(100,100), c(5,5), d(4.9,0), e(8.8,0), f(1,0);
    bin_type  bins(a,b,c);//min, max, size
 
    bins.add_point(e, 1);
    bins.add_point(f, 2);
 
    vcl_vector<int> answer;
    bins.n_nearest(d, 1, answer);
 
    testlib_test_begin( "Second bin test bin size 5,5" );
    testlib_test_perform( answer.size() >= 1 && (answer[0] == 1 ||  answer[0] == 2) );
  }
 
  {//same as new test with bin size 2,2
    vnl_double_2 a(0,0), b(100,100), c(2,2), d(4.9,0), e(8.8,0), f(1,0);
    bin_type  bins(a,b,c);//min, max, size
 
    bins.add_point(e, 1);
    bins.add_point(f, 2);
 
    vcl_vector<int> answer;
    bins.n_nearest(d, 1, answer);
 
    testlib_test_begin( "Second bin test bin size 2,2" );
    testlib_test_perform( answer.size() == 1 && (answer[0] == 1 ||  answer[0] == 2) );
  }

  {                                                                             
      vnl_double_2 size(5,5), min(0,0), max(640,480), query_pt(329.69, 241.57);                                                                        
      vnl_double_2 data_pt_1(343.10, 225.90), data_pt_2(344.94, 227.06);        
      bin_type  bins(min,max,size);                            
                                                                                
      bins.add_point(data_pt_1, 1);                                             
      bins.add_point(data_pt_2, 2);                                             
                                                                                
      vcl_vector<int> answer;                                                   
      bins.n_nearest(query_pt, 2, answer);
      testlib_test_begin( "Nearest 2" );
      testlib_test_perform( answer.size() == 2 && answer[0] == 1 && answer[1] == 2 );
  }             

  {                                                                             
      vnl_double_2 size(5,5), min(0,0), max(640,480), query_pt(329.69, 241.57);                                                                        
      vnl_double_2 data_pt_1(343.10, 225.90), data_pt_2(344.94, 227.06);        
      bin_type  bins(min,max,size);                            
                                                                                
      bins.add_point(data_pt_1, 1);                                             
      bins.add_point(data_pt_2, 2);                                             
                                                                                
      vcl_vector<int> answer;                                                   
      bins.n_nearest(query_pt, 5, answer);
      testlib_test_begin( "Nearest 5 w/o enough data" );
      testlib_test_perform( answer.size() == 2 && answer[0] == 1 && answer[1] == 2 );
  }             

}


// Test 3D bins
static
void
test_bins_3()
{
  vcl_cout << "Testing 3D bins\n";

  typedef rsdl_bins< 3, double, int > bin_type;

  vnl_vector_fixed< double, 3 > min_pt, max_pt, bin_sizes;
  min_pt[0] = 1.0;   min_pt[1] = -2.5;  min_pt[2] = -5.0;
  max_pt[1] = 75.0;  max_pt[1] = 32;    max_pt[2] = 10.0;
  bin_sizes[0] = 6.0; bin_sizes[1] = 4.0; bin_sizes[2] = 5.0;

  {
    bin_type bins( min_pt, max_pt, bin_sizes );
    double dist_tol = 0.0001;
    bins.set_distance_tolerance( dist_tol );
    vnl_double_3  pt( 12.5, 3.0, -5.0 );
    int value = 0;
    bins.add_point( pt, value ); // should be 1, 0

    int stored_value;

    pt[0] = 100; pt[1] = -30; pt[2] = -50;
    int added_value = 24;
    testlib_test_begin( "adding point outside range 1" );
    bins.add_point( pt, added_value );
    testlib_test_perform( bins.get_value( pt, stored_value )
                          && stored_value == added_value );

    pt[0] = -50; pt[1] = 77.7; pt[3] = 0;
    added_value = 13;
    testlib_test_begin( "adding point outside range 2" );
    bins.add_point( pt, added_value );
    testlib_test_perform( bins.get_value( pt, stored_value )
                          && stored_value == added_value );
  }

  {
    //  Generate a bunch of points:
    const int M=60;
    vcl_vector< vnl_vector_fixed< double, 3 > > points( M );
    vcl_vector< int > indices( M );
    mbl_mz_random mz_rand;
    bin_type bins( min_pt, max_pt, bin_sizes );

    for ( int i=0; i<M; ++i ) {
      indices[ i ] = i;
      points[ i ][0] = (max_pt[0] - min_pt[0]) * mz_rand.drand32() + min_pt[0];
      points[ i ][1] = (max_pt[1] - min_pt[1]) * mz_rand.drand32() + min_pt[1];
      points[ i ][2] = (max_pt[2] - min_pt[2]) * mz_rand.drand32() + min_pt[2];
      bins.add_point( points[ i ], i );
    }

    const int num_tests = 5;
    for ( int t=0; t<num_tests; ++t ) {
      vnl_vector_fixed< double, 3 > q;
      q[0] = (max_pt[0] - min_pt[0]) * mz_rand.drand32() + min_pt[0];
      q[1] = (max_pt[1] - min_pt[1]) * mz_rand.drand32() + min_pt[1];
      q[2] = (max_pt[2] - min_pt[2]) * mz_rand.drand32() + min_pt[2];
      double radius = (t+1) * bin_sizes.x() / 2;  // increasing size, starting smallest

      bool bin_answer = bins.is_any_point_within_radius( q, radius );
    
      vcl_vector< int > all_close_indices;
      for ( int i=0; i<M; ++i ) {
        if ( vnl_vector_ssd( points[ i ], q ) < vnl_math_sqr( radius ) ) {
          all_close_indices.push_back( i );
        }
      }

      testlib_test_begin( "is_any_point_within_radius" );
      testlib_test_perform( bin_answer == (all_close_indices.size() > 0) );
      
      vcl_vector< int > bin_close_indices;
      bins.points_within_radius( q, radius, bin_close_indices );
      bool correct = all_close_indices.size() ==  bin_close_indices.size();
      vcl_sort( all_close_indices.begin(), all_close_indices.end() );
      vcl_sort( bin_close_indices.begin(), bin_close_indices.end() );

      for ( unsigned int i=0; correct &&
              i < vnl_math_min(all_close_indices.size(), bin_close_indices.size()); ++i )
        correct = all_close_indices[ i ] == bin_close_indices[ i ];

      testlib_test_begin( "points_within_radius" );
      testlib_test_perform( correct );
    }
  }

  {
    vnl_double_3 min(0.0,0.0,0.0);
    vnl_double_3 max(4.0,3.0,6.0);
    vnl_double_3 size(10.0,10.0,10.0);
    bin_type bins( min, max, size );

    bins.add_point( vnl_double_3(0.0,0.0,0.0), 0 );
    bins.add_point( vnl_double_3(1.0,0.0,0.0), 1 );
    bins.add_point( vnl_double_3(0.0,1.0,0.0), 2 );
    bins.add_point( vnl_double_3(0.0,1.0,1.0), 3 );
    bins.add_point( vnl_double_3(4.0,3.0,6.0), 4 );

    testlib_test_begin( "Retrieve nearest 2, large bins" );
    vcl_vector<int> nearest;
    bins.n_nearest( vnl_double_3(8.0,3.0,6.0), 2, nearest );
    testlib_test_perform( nearest.size() == 2 &&
                          ( ( nearest[0] == 4 && nearest[1] == 3 ) ||
                            ( nearest[0] == 3 && nearest[1] == 4 ) ) );
  }

  // same test with smaller bin sizes
  {
    vnl_double_3 min(0.0,0.0,0.0);
    vnl_double_3 max(4.0,3.0,6.0);
    vnl_double_3 size(1.0,1.0,1.0);
    bin_type bins( min, max, size );

    bins.add_point( vnl_double_3(0.0,0.0,0.0), 0 );
    bins.add_point( vnl_double_3(1.0,0.0,0.0), 1 );
    bins.add_point( vnl_double_3(0.0,1.0,0.0), 2 );
    bins.add_point( vnl_double_3(0.0,1.0,1.0), 3 );
    bins.add_point( vnl_double_3(4.0,3.0,6.0), 4 );

    testlib_test_begin( "Retrieve nearest 2, small bins" );
    vcl_vector<int> nearest;
    bins.n_nearest( vnl_double_3(8.0,3.0,6.0), 2, nearest );
    testlib_test_perform( nearest.size() == 2 &&
                          ( ( nearest[0] == 4 && nearest[1] == 3 ) ||
                            ( nearest[0] == 3 && nearest[1] == 4 ) ) );
  }

}


                    
MAIN( test_bins )
{
  START( "rsdl_bins" );

  test_bins_2();
  test_bins_3();

  SUMMARY();
}
