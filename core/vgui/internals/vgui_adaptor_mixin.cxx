// This is core/vgui/internals/vgui_adaptor_mixin.cxx
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma implementation
#endif
//:
// \file
// \author fsm

#include "vgui_adaptor_mixin.h"

vgui_adaptor_mixin::vgui_adaptor_mixin()
  : width(0)
  , height(0)
  //
  , popup_button(vgui_RIGHT)
  , popup_modifier(vgui_MODIFIER_NULL)
{
}

vgui_adaptor_mixin::~vgui_adaptor_mixin()
{
}
