#ifdef __GNUC__
#pragma implementation
#endif

// This is vxl/vil/vil_load.cxx

#include "vil_load.h"

#include <vcl_cstdio.h>   // sprintf()
#include <vcl_cstdlib.h>  // atoi()
#include <vcl_cstring.h>

#include <vil/vil_file_format.h>
#include <vil/vil_stream_fstream.h>
#include <vil/vil_stream_core.h>
#include <vil/vil_stream_url.h>
#include <vil/vil_image.h>
#include <vil/vil_property.h> // for vil_property_top_row_first
#include <vil/vil_flipud.h>
#include <vil/vil_flip_components.h>

vil_image vil_load_raw(vil_stream *is)
{
  for (vil_file_format** p = vil_file_format::all(); *p; ++p) {
#if 0 // debugging
    vcl_cerr << __FILE__ " : trying \'" << (*p)->tag() << "\'" << vcl_endl;
#endif
    is->seek(0);
    vil_image i = (*p)->make_input_image(is);
    if (i)
      return i;
  }

  // failed.
  vcl_cerr << "vil_load: Tried";
  for (vil_file_format** p = vil_file_format::all(); *p; ++p)
    // 'flush' in case of segfault next time through loop. Else, we
    // will not see those printed tags still in the stream buffer.
    vcl_cerr << " \'" << (*p)->tag() << "\'" << vcl_flush;
  vcl_cerr << vcl_endl;

  return 0;
}

vil_image vil_load_raw(char const* filename)
{
  // check for null pointer or empty strings.
  if (!filename || !*filename)
    return 0;

  vil_stream *is = new vil_stream_fstream(filename, "r");
  is->ref();
  // current block scope is owner of stream.

  if (!is->ok()) {
    int l = vcl_strlen(filename);

    // hacked check for filenames beginning "gen:".
    if (l > 4 && vcl_strncmp(filename, "gen:", 4) == 0) {
      is->unref();
      // Make an in-core stream...
      vil_stream_core *cis = new vil_stream_core();
      cis->ref();
      cis->m_transfer((char*)filename, 0, l+1, false/*write*/);
      is = cis;
    }

    // maybe it's a URL?
    if (l > 4 && vcl_strncmp(filename, "http://", 7) == 0) {
      is->unref();
      is = new vil_stream_url(filename);
      is->ref();
    }

    // or a picture of a pig?
    if (l > 4 && vcl_strncmp(filename, "pig:", 4) == 0) {
      is->unref();
      char buf[4096];
      vcl_sprintf(buf, "http://www.robots.ox.ac.uk/~vxl/images/pig%d.jpg", vcl_atoi(filename+4));
      is = new vil_stream_url(buf);
      is->ref();
    }
  }

  if (!is->ok()) {
    is->unref();
    // end of block scope coming up.
    return 0;
  }

  vil_image img = vil_load_raw(is);
  // stream has up to two owners now.

  is->unref();
  is = 0;
  // block scope no longer owns stream.

  if (! img)
    vcl_cerr << "vil_load: Failed to load [" << filename << "]" << vcl_endl;

  return img;
}

vil_image vil_load(char const* filename)
{
  vil_image i = vil_load_raw(filename);
  bool top_first;
  if (i.get_property(vil_property_top_row_first, &top_first) && !top_first)
    i = vil_flipud(i);
  if (i.components() == 3 && i.get_property(vil_property_component_order_is_BGR))
    i = vil_flip_components(i);
  return i;
}
