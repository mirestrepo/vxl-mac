// This is vxl/vil/vil_stream_section.cxx

/*
  fsm@robots.ox.ac.uk
*/
#ifdef __GNUC__
#pragma implementation
#endif
#include "vil_stream_section.h"
#include <vcl_cassert.h>
#include <vcl_iostream.h>

// underlying_: pointer to underlying stream.
// begin_     : start of section in the underlying stream.
// end_       : end of section in the underlying stream. -1 if there is no (explicit) end.
// current_   : current position (in the underlying stream) of the adapted stream.

vil_stream_section::vil_stream_section(vil_stream *underlying, int begin)
  : underlying_(underlying)
  , begin_(begin)
  , end_(-1)
  , current_(begin)
{
  assert(underlying);
  assert(begin >= 0);
  underlying_->ref();
}

vil_stream_section::vil_stream_section(vil_stream *underlying, int begin, int end)
  : underlying_(underlying)
  , begin_(begin)
  , end_(end)
  , current_(begin)
{
  assert(underlying);
  assert(begin >= 0);
  assert(begin <= end);
  underlying->ref();
}

vil_stream_section::~vil_stream_section()
{
  // unreffing the underlying stream might cause deletion of *this, so
  // zero out the pointer first.
  vil_stream *u = underlying_;
  underlying_ = 0;
  u->unref();
}

bool vil_stream_section::ok()
{
  return underlying_->ok();
}

int vil_stream_section::write(void const* buf, int n)
{
  assert(n >= 0); // wouldn't you want to be told?

  // huh? this should never happen, even if someone else is
  // manipulating the underlying stream too.
  assert(begin_<=current_);
  if (end_ != -1)
    assert(current_<=end_);

  // shrink given buffer so it fits into our section.
  if (end_ != -1  &&  current_ + n > end_)
    n = end_ - current_;

  // seek to where we have been telling the clients we are.
  underlying_->seek(current_);

  // this could be a bug in the caller's code or merely a
  // failure to seek on underlying stream.
  assert(underlying_->tell() == current_);

  int nb = underlying_->write(buf, n);
  if (nb >= 0)
    current_ += nb;
  return nb;
}

int vil_stream_section::read(void* buf, int n)
{
  assert(n >= 0); // wouldn't you want to be told?

  // huh? this should never happen, even if someone else is
  // manipulating the underlying stream too.
  assert(begin_<=current_);
  if (end_ != -1)
    assert(current_<=end_);

  // shrink given buffer so it fits into our section.
  if (end_ != -1  &&  current_ + n > end_)
    n = end_ - current_;

  // seek to where we have been telling the clients we are.
  underlying_->seek(current_);

  // this could be a bug in the caller's code or merely a
  // failure to seek on underlying stream.
  assert(underlying_->tell() == current_);

  int nb = underlying_->read(buf, n);
  if (nb >= 0)
    current_ += nb;
  return nb;
}

int vil_stream_section::tell()
{
  return current_; // regardless of what the underlying stream is doing.
}

void vil_stream_section::seek(int position)
{
  assert(position >= 0); // I would want to be told about this.

  if (end_ != -1  &&  begin_ + position > end_) {
    vcl_cerr << __FILE__ << ": attempt to seek past given section (failed)." << vcl_endl;
    return;
  }
  else
    current_ = begin_ + position;
}
