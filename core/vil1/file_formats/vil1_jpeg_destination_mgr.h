#ifndef vil_jpeg_destination_mgr_h_
#define vil_jpeg_destination_mgr_h_
#ifdef __GNUC__
#pragma interface
#endif
/*
  fsm@robots.ox.ac.uk
*/

#include <vil/vil_jpeglib.h>
class vil_stream;

//: this is the data source structure which allows JPEG to write to a vil_stream.
struct vil_jpeg_stream_destination_mgr {
  struct jpeg_destination_mgr base;
  
  vil_stream *stream;           /* target stream */
  JOCTET * buffer;              /* start of buffer */
};

void
vil_jpeg_init_destination (j_compress_ptr cinfo);

jpeg_boolean
vil_jpeg_empty_output_buffer (j_compress_ptr cinfo);

void
vil_jpeg_term_destination (j_compress_ptr cinfo);

void
vil_jpeg_stream_dst_set (j_compress_ptr cinfo, vil_stream *vs);

void 
vil_jpeg_stream_dst_rewind(j_compress_ptr cinfo, vil_stream *vs);

#endif
