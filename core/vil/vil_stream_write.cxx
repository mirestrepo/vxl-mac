#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma implementation
#endif
//:
// \file
// \author fsm

#include "vil2_stream_write.h"
#include <vil2/vil2_stream.h>
#include <vxl_config.h>


void vil2_stream_write_big_endian_uint_16(vil2_stream *s, vxl_uint_16 w)
{
  vxl_uint_8 bytes[2];
  bytes[0] = vxl_uint_8(w >> 8);
  bytes[1] = vxl_uint_8(w & 0xff);
  s->write(bytes, sizeof bytes);
}

void vil2_stream_write_little_endian_uint_16(vil2_stream *s, vxl_uint_16 w)
{
  vxl_uint_8 bytes[2];
  bytes[0] = vxl_uint_8(w & 0xff);
  bytes[1] = vxl_uint_8(w >> 8);
  s->write(bytes, sizeof bytes);
}

void vil2_stream_write_big_endian_uint_32(vil2_stream *s, vxl_uint_32 w)
{
  vxl_byte bytes[4];
  bytes[0] = w >> 24;
  bytes[1] = w >> 16;
  bytes[2] = w >> 8;
  bytes[3] = w >> 0;
  s->write(bytes, sizeof bytes);
}

void vil2_stream_write_little_endian_uint_32(vil2_stream *s, vxl_uint_32 w)
{
  vxl_byte bytes[4];
  bytes[0] = w >> 0;
  bytes[1] = w >> 8;
  bytes[2] = w >> 16;
  bytes[3] = w >> 24;
  s->write(bytes, sizeof bytes);
}
