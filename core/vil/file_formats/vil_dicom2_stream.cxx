#include "vil_dicom2_stream.h"

#include <vil/vil_stream.h>

#include <dcerror.h>


#include <vcl_cassert.h>

// ===========================================================================
//                                                             stream producer

vil_dicom2_stream_producer::
vil_dicom2_stream_producer( vil_stream* in_vs )
  : vs_( in_vs )
{
  vs_->ref();
}


vil_dicom2_stream_producer::
~vil_dicom2_stream_producer()
{
  vs_->unref();
}


OFBool
vil_dicom2_stream_producer::
good() const
{
  return vs_->ok();
}


OFCondition
vil_dicom2_stream_producer::
status() const
{
  return good() ? EC_Normal : EC_InvalidStream;
}


OFBool
vil_dicom2_stream_producer::
eos() const
{
  return vs_->tell() >= vs_->file_size();
}


Uint32
vil_dicom2_stream_producer::
avail() const
{
  vil_streampos n = vs_->file_size() - vs_->tell();
  assert( n >= 0 );
  return n;
}


Uint32
vil_dicom2_stream_producer::
read( void *buf, Uint32 buflen )
{
  return vs_->read( buf, buflen );
}


Uint32
vil_dicom2_stream_producer::
skip(Uint32 skiplen)
{
  vs_->seek( vs_->tell() + skiplen );
  return skiplen;
}


void
vil_dicom2_stream_producer::
putback(Uint32 num)
{
  vs_->seek( vs_->tell() - (long int)num );
}


// ===========================================================================
//                                                              stream factory

vil_dicom2_stream_factory::
vil_dicom2_stream_factory( vil_stream* in_vs )
  : vs_( in_vs )
{
  vs_->ref();
}

vil_dicom2_stream_factory::
~vil_dicom2_stream_factory()
{
  vs_->unref();
}


DcmInputStream*
vil_dicom2_stream_factory::
create() const
{
  return new vil_dicom2_stream_input( vs_ );
}




// ===========================================================================
//                                                                stream input

vil_dicom2_stream_input::
vil_dicom2_stream_input( vil_stream* vs )
  : DcmInputStream( new vil_dicom2_stream_producer( vs ) ),
    producer_( 0 )
{
}


vil_dicom2_stream_input::
~vil_dicom2_stream_input()
{
  delete producer_;
}


DcmInputStreamFactory*
vil_dicom2_stream_input::
newFactory() const
{
  return 0;
}
