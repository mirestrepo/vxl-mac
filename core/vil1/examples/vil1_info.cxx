/*
  fsm@robots.ox.ac.uk
*/
#include <vcl_cstdlib.h>  // abort()
#include <vcl_iostream.h>

#include <vil/vil_image.h>
#include <vil/vil_load.h>

int main(int argc, char **argv)
{
  for (int i=1; i<argc; ++i)
  {
    vcl_cerr << argv[i] << " :" << vcl_endl;
    vil_image I = vil_load(argv[i]);
    vcl_cerr << "  planes             : " << I.planes() << vcl_endl
         << "  width              : " << I.width() << vcl_endl
         << "  height             : " << I.height() << vcl_endl
         << "  components         : " << I.components() << vcl_endl
         << "  bits per component : " << I.bits_per_component() << vcl_endl
         << "  component format   : ";
    switch (I.component_format()) {
    case VIL_COMPONENT_FORMAT_UNKNOWN: vcl_cerr << "unknown" << vcl_endl; break;
    case VIL_COMPONENT_FORMAT_UNSIGNED_INT: vcl_cerr << "unsigned" << vcl_endl; break;
    case VIL_COMPONENT_FORMAT_SIGNED_INT: vcl_cerr << "signed" << vcl_endl; break;
    case VIL_COMPONENT_FORMAT_IEEE_FLOAT: vcl_cerr << "float" << vcl_endl; break;
    case VIL_COMPONENT_FORMAT_COMPLEX: vcl_cerr << "complex" << vcl_endl; break;
    default: return 1; // Note: abort() requires #include <vcl_cstdlib.h> - PVr
    }
  }
  return 0;
}
