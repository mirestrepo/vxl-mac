#include "vsl_harris.h"

#include <vcl/vcl_cmath.h>
#include <vcl/vcl_cstdlib.h>
#include <vcl/vcl_cassert.h>
#include <vcl/vcl_iostream.h>
#include <vcl/vcl_iomanip.h>
#include <vcl/vcl_fstream.h>
#include <vcl/vcl_vector.h>
#include <vcl/vcl_algorithm.h>
#include <vcl/vcl_functional.h>

#include <vil/vil_image_as.h>
#include <vil/vil_memory_image_of.h>

#include <vsl/vsl_roi_window.h>
#include <vsl/vsl_convolve.h>
#include <vsl/internals/droid.h> //name

//------------------------------------------------------------

void vsl_harris::prepare_buffers(int w, int h)
{
  // remember size.
  image_h = h;
  image_w = w;

  if (params_.verbose)
    cerr << "Doing harris on image region "
	 << image_h << " by " << image_w << endl
	 << "Maximum no of corners                     = " << params_.corner_count_max << endl
	 << "Gaussian sigma                            = " << params_.gauss_sigma << endl
	 << "Expected ratio lowest/max corner strength = " << params_.relative_minimum << endl
	 << "Auto-correlation scale factor             = " << params_.scale_factor << endl
	 << "Computing cornerness operator response...." << flush;
  
  // response images (no realloc performed unless size actually changes).
  image_buf           .resize(image_w, image_h);
  image_gradx_buf     .resize(image_w, image_h);
  image_grady_buf     .resize(image_w, image_h);
  image_fxx_buf       .resize(image_w, image_h);
  image_fxy_buf       .resize(image_w, image_h);
  image_fyy_buf       .resize(image_w, image_h);
  image_cornerness_buf.resize(image_w, image_h);
  image_cornermax_buf .resize(image_w, image_h);
  
  // set up window
  window_str.row_start_index = 0;
  window_str.col_start_index = 0;
  window_str.row_end_index = image_h-1;
  window_str.col_end_index = image_w-1;
}
  
void vsl_harris::compute_gradients(vil_image const &image) 
{
  // copy input image to byte buffer
  vil_image_as_byte(image).get_section(image_buf.get_buffer(), 0, 0, image_w, image_h);
  
  // compute gradients
  if (params_.verbose)
    cerr << " gradient" << flush;

  // trim the window
  window_str.row_start_index += 2;
  window_str.col_start_index += 2;
  window_str.row_end_index   -= 2;
  window_str.col_end_index   -= 2;
  droid::compute_gradx_grady (&window_str,
			      &image_buf,
			      &image_gradx_buf,
			      &image_grady_buf);
}

void vsl_harris::compute_2nd_moments()
{
  // compute 2nd moment matrices
  if (params_.verbose)
    cerr << " fxx,fxy,fyy" << flush;
  window_str.row_start_index += 2;
  window_str.col_start_index += 2;
  window_str.row_end_index   -= 2;
  window_str.col_end_index   -= 2;
  droid::compute_fxx_fxy_fyy (&window_str,
			      &image_gradx_buf,
			      &image_grady_buf,
			      &image_fxx_buf,
			      &image_fxy_buf,
			      &image_fyy_buf);
  
  // create smoothing kernel
  vsl_1d_half_kernel<double> gauss_mask;
  vsl_create_gaussian (double(params_.gauss_sigma), &gauss_mask);

  // smoothe the 2nd moment matrix maps
  if (params_.verbose)
    cerr << " convolution" << flush;
  { // we use the cornerness map as a temporary scratch area.
    vil_memory_image_of<float> *tmp = &image_cornerness_buf;
    vsl_convolve(&window_str, &gauss_mask, &image_fxx_buf, tmp);
    vsl_convolve(&window_str, &gauss_mask, &image_fxy_buf, tmp);
    vsl_convolve(&window_str, &gauss_mask, &image_fyy_buf, tmp);
  }

  // trim the window :
  window_str.row_start_index += (gauss_mask.count-1);
  window_str.row_end_index   -= (gauss_mask.count-1);
  window_str.col_start_index += (gauss_mask.count-1);
  window_str.col_end_index   -= (gauss_mask.count-1);
}

void vsl_harris::compute_cornerness()
{
  // compute cornerness map
  if (params_.verbose)
    cerr << " cornerness" << flush;
  corner_max = droid::compute_cornerness (&window_str,
					  &image_fxx_buf,
					  &image_fxy_buf,
					  &image_fyy_buf,
					  params_.scale_factor,
					  &image_cornerness_buf);
  //
  if (params_.verbose)
    cerr << "  done" << endl;
}

void vsl_harris::compute_corners()
{
  // do the relevant harris flavours :
  double corner_min;
  if (params_.adaptive) {
    corner_min = 0.0;
    do_adaptive(); 
  }
  else {
    corner_min = params_.relative_minimum * corner_max;
    do_non_adaptive(&corner_min);
  }


  // store the corners found.
  cc.clear();
  for (int row = window_str.row_start_index; row < window_str.row_end_index; row++) {
    for (int col = window_str.col_start_index; col < window_str.col_end_index; col++) {
      if (image_cornermax_buf[row][col] && image_cornerness_buf[row][col] > corner_min) {
	double x, y;
	if (droid::compute_subpixel_max (&image_cornerness_buf, row, col, x,y, params_.pab_emulate))
	  cc.push_back(vcl_pair<float, float>(params_.col_start_index+x, 
					      params_.row_start_index+y));
      }
    }
  }
  cerr << "vsl_harris: Final corner count " << cc.size() << endl;
}

//--------------------------------------------------------------------------------

//: internal
void vsl_harris::do_non_adaptive(double *corner_min) {
  int maxima_count = droid::find_corner_maxima (*corner_min,
						&window_str,
						&image_cornerness_buf,
						&image_cornermax_buf);

  // iterate if not enough corners.
  
  if (params_.verbose) 
    cerr << "Found " << maxima_count << " corners\n";
  
  if (maxima_count < (float) params_.corner_count_max * 0.9) {
    for (int i=0 ; i<10 && maxima_count < (float) params_.corner_count_max * 0.9; i++) {
      params_.relative_minimum *= 0.5;
      *corner_min = params_.relative_minimum * corner_max;
      if (params_.verbose) 
	cerr << "Found " << maxima_count
	     << "... iterating with relmin = " << params_.relative_minimum
	     << endl;
      maxima_count = droid::find_corner_maxima (*corner_min,
						&window_str,
						&image_cornerness_buf,
						&image_cornermax_buf);
    }
  }
  
  // too many corners - reset parameters to get max number.

  if (maxima_count > params_.corner_count_max) {
    *corner_min = droid::compute_corner_min (*corner_min, 
					     corner_max,
					     params_.corner_count_max,
					     &window_str,
					     &image_cornerness_buf,
					     &image_cornermax_buf);
    
    params_.relative_minimum = *corner_min / corner_max;
    if (params_.verbose) 
      cerr << "vsl_harris: Too many: " << maxima_count
	   << "... iterating with relmin = " << params_.relative_minimum
	   << endl;
  }
}

//--------------------------------------------------------------------------------

//: internal
void vsl_harris::do_adaptive() {
  if (params_.verbose)
    cerr << "No. of corners before density thresholding= " << params_.corner_count_max << endl;

  double corner_min = params_.relative_minimum * corner_max;
  int maxima_count = droid::find_corner_maxima (corner_min,
						&window_str,
						&image_cornerness_buf,
						&image_cornermax_buf);
  cerr << "harris: " << maxima_count << " corners with response above " << corner_min << endl;

  // Store all corners in an array.
  int TILE_WIDTH = params_.adaptive_window_size; // 32
  int row_min = window_str.row_start_index;
  int col_min = window_str.col_start_index;
  int row_max = window_str.row_end_index;
  int col_max = window_str.col_end_index;

  // number of tiles to use in the x and y directions.
  int n_tiles_x = (int) ceil(double(col_max-col_min) / TILE_WIDTH);
  int n_tiles_y = (int) ceil(double(row_max-row_min) / TILE_WIDTH);

  double TILE_AREA = TILE_WIDTH*TILE_WIDTH;

  int IDEAL_NUM_PER_TILE = params_.corner_count_max / (n_tiles_x*n_tiles_y);

  if (params_.corner_count_max == 0)
    IDEAL_NUM_PER_TILE = params_.corner_count_low;

  cerr << "Tiles " << n_tiles_x << " x " << n_tiles_y 
       << ", NUM_PER_TILE " << IDEAL_NUM_PER_TILE << endl;
  
  vcl_vector<double> cornerness(maxima_count, 0.0);

  for(int tile_y = 0; tile_y < n_tiles_y; ++tile_y) {
    int window_row_start_index = tile_y * TILE_WIDTH+row_min;
    int window_row_end_index = vcl_min(window_row_start_index+TILE_WIDTH, row_max);
    
    for(int tile_x = 0; tile_x < n_tiles_x; ++tile_x) {
      int window_col_start_index = tile_x * TILE_WIDTH+col_min;
      int window_col_end_index = vcl_min(window_col_start_index+TILE_WIDTH, col_max);

      // get corner strengths in this tile :
      vil_memory_image_of<bool>        &corner_present  = image_cornermax_buf;
      vil_memory_image_of<float> const &corner_strength = image_cornerness_buf;
      int n = 0;
      for(int row = window_row_start_index; row < window_row_end_index; row++)
	for(int col = window_col_start_index; col < window_col_end_index; col++)
	  if (corner_present[row][col])
	    cornerness[n++] = corner_strength[row][col];
      cerr << setw(3) << n << ' ';

      //
      double THIS_TILE_AREA = 
	(window_row_end_index-window_row_start_index) *
	(window_col_end_index-window_col_start_index);

      int NUM_PER_TILE = (int) ( IDEAL_NUM_PER_TILE * (THIS_TILE_AREA/TILE_AREA) );

      if (n > NUM_PER_TILE) {
	// Sort corners to get thresholds
	vcl_sort(cornerness.begin(), cornerness.begin()+n);
	double thresh = cornerness[n-1-NUM_PER_TILE];

	// Zap corners over thresh
	for(int row = window_row_start_index; row < window_row_end_index; row++)
	  for(int col = window_col_start_index; col < window_col_end_index; col++)
	    if (corner_present[row][col])
	      if (corner_strength[row][col] < thresh)
		corner_present[row][col] = false;
      }
    }
    cerr << endl;
  }
}

//-----------------------------------------------------------------------------

// ?? should we append, and not assign ??
void vsl_harris::get_corners(vcl_vector<vcl_pair<float, float> > &cor) const {
  cor = cc;
}
void vsl_harris::get_corners(vcl_vector<float> &corx, vcl_vector<float> &cory) const {
  for (unsigned i=0; i<cc.size(); ++i) {
    corx.push_back(cc[i].first);
    cory.push_back(cc[i].second);
  }
}

//: convenience method
void vsl_harris::save_corners(ostream &f) const {
  for (unsigned i=0; i<cc.size(); ++i)
    f << cc[i].first << ' ' << cc[i].second << endl;
}
void vsl_harris::save_corners(char const *filename) const {
  ofstream f(filename);
  assert(f.good());
  save_corners(f);
  f.close();
}

//-----------------------------------------------------------------------------
