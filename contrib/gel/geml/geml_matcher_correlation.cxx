//-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation "geml_matcher_correlation.h"
#endif
//
// Class: geml_matcher_correlation
// Author: Geoffrey Cross, Oxford RRG
// Created: ${DATE}
// Modifications:
//   @(eval (strftime "%y%m%d")) Geoff Initial version.
//
//-----------------------------------------------------------------------------

#include <vnl/vnl_math.h>
#include <vnl/vnl_matrix.h>

#include <vil/vil_memory_image_window.h>

#include "geml_matcher_correlation.h"


#define MAX_CORNER_ERROR 1
#define CORRELATION_KERNEL 3
#define NORMALISED_CORRELATION false
#define SEARCH_WINDOW_X 50
#define SEARCH_WINDOW_Y 50
#define NO_SCORE -2

geml_matcher_correlation::geml_matcher_correlation( const vil_memory_image_of<vil_byte> image1, 
						    const vil_memory_image_of<vil_byte> image2, 
						    const vcl_vector< vcl_pair<float,float> > &corners1,
						    const vcl_vector< vcl_pair<float,float> > &corners2)
  : geml_matcher( image1, image2, corners1, corners2)
{
}


vcl_vector< vcl_pair<int,int> > geml_matcher_correlation::get_matches()
{
  // correlate each corner against each corner
  vnl_matrix<double> scores1to2(corners1_.size(),corners2_.size());
  vnl_matrix<double> scores2to1(corners2_.size(),corners1_.size());
  
  for( int i=0; i< corners1_.size(); i++)
    {
      double x1= corners1_[i].first;
      double y1= corners1_[i].second;

      for( int j=0; j< corners2_.size(); j++)
	{
	  double x2= corners2_[j].first;
	  double y2= corners2_[j].second;

	  if(( vnl_math_abs( x1-x2)< SEARCH_WINDOW_X) && ( vnl_math_abs( y1-y2)< SEARCH_WINDOW_Y))
	    {
	      vcl_pair<double,double> scores= best_local_correlation_score( i, j);
	      
	      scores1to2.put( i, j, scores.first);
	      scores2to1.put( j, i, scores.second);
	    }
	  else
	    {
	      scores1to2.put( i, j, NO_SCORE);
	      scores2to1.put( i, j, NO_SCORE);
	    }
	}
    }

  // look for best match to first image corners
  vcl_vector<int> bestimage1match( corners1_.size());
  vcl_vector<double> bestimage1score( corners1_.size());

  for( int i=0; i< corners1_.size(); i++)
    {
      double bestscore= NO_SCORE;
      double bestmatch= -1;

      for( int j=0; j< corners2_.size(); j++)
	{
	  if( bestscore== NO_SCORE)
	    {
	      bestscore= scores1to2( i, j);
	      bestmatch= j;
	    }
	  else if(( bestscore> scores1to2( i, j)) && ( scores1to2( i, j)!= NO_SCORE))
	    {
	      bestscore= scores1to2( i, j);
	      bestmatch= j;
	    }
	}

      bestimage1match[i]= bestmatch;
      bestimage1score[i]= bestscore;
    }


  // look for best match to second image corners
  vcl_vector<int> bestimage2match( corners2_.size());
  vcl_vector<double> bestimage2score( corners2_.size());

  for( int i=0; i< corners2_.size(); i++)
    {
      double bestscore= NO_SCORE;
      double bestmatch= -1;

      for( int j=0; j< corners1_.size(); j++)
	{
	  if( bestscore== NO_SCORE)
	    {
	      bestscore= scores2to1( i, j);
	      bestmatch= j;
	    }
	  else if(( bestscore> scores2to1( i, j)) && ( scores2to1( i, j)!= NO_SCORE))
	    {
	      bestscore= scores2to1( i, j);
	      bestmatch= j;
	    }
	}

      bestimage2match[i]= bestmatch;
      bestimage2score[i]= bestscore;
    }

  // and check that the best match from image 1 to 2 is the
  //  same as the best match from image 2 to 1
  
  for( int i=0; i< corners1_.size(); i++)
    {
      int a= bestimage1match[i];
      int b= bestimage2match[a];

      if( i==b)
	{
	  cerr << i << " " << a << endl;
	  cout << corners1_[i].first << " " << corners1_[i].second << " "
	       << corners2_[a].first << " " << corners2_[a].second << endl;
	}
    

    }


  //  cerr << bestimage1match << endl;
  //  cerr << bestimage2match << endl;

  // dummy return value for the moment
  vcl_vector< vcl_pair<int,int> > l;
  return l;
}


// search in a small window (about 3x3) for the best correlation between a pair of corners
vcl_pair<double,double> geml_matcher_correlation::best_local_correlation_score( const int c1, const int c2)
{
  double x1= corners1_[c1].first;
  double y1= corners1_[c1].second;
  double x2= corners2_[c2].first;
  double y2= corners2_[c2].second;
  double bestscore1= -1;
  double bestscore2= -1;

  vil_memory_image_window w1( im1_, x1, y1, CORRELATION_KERNEL);
  vil_memory_image_window w2( im2_, x2, y2, CORRELATION_KERNEL);

  for( int x= -MAX_CORNER_ERROR; x<= MAX_CORNER_ERROR; x++)
    {
      for( int y= -MAX_CORNER_ERROR; y<= MAX_CORNER_ERROR; y++)
	{
	  double score1, score2;

	  //	  score1= w1.sum_squared_differences(im2_, x2+ x, y2+ y);
	  //	  score2= w2.sum_squared_differences(im1_, x1+ x, y1+ y);
	  score1= w1.normalised_cross_correlation(im2_, x2+ x, y2+ y);
	  score2= w2.normalised_cross_correlation(im1_, x1+ x, y1+ y);



	  if(( score1< bestscore1) || ( bestscore1< 0))
	    bestscore1= score1;

	  if(( score2< bestscore2) || ( bestscore2< 0))
	    bestscore2= score2;

	}
    }


  return vcl_pair<double,double>(bestscore1,bestscore2);
}
