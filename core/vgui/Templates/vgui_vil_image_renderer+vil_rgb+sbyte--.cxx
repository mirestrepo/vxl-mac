#include <vgui/vgui_vil_image_renderer.txx>
#include <vxl_config.h>
#include <vil/vil_rgb.h>

typedef vil_rgb<vxl_sbyte> Type;

template class vgui_vil_image_renderer<Type>;
