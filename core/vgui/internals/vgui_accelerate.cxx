//-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation
#endif
//
// Author: David Capel
// Created: 01 Apr 2000
//
//-----------------------------------------------------------------------------

#include <vcl_compiler.h>

#ifdef VCL_WIN32
#include <vgui/impl/mfc/stdafx.h>
extern CDC *vgui_mfc_adaptor_global_dc;
#endif

#include "vgui_accelerate.h"

#include <vcl_iostream.h>
#include <vcl_cassert.h>

#include <vgui/vgui_gl.h>
#include <vgui/vgui_utils.h>

bool vgui_accelerate::vgui_no_acceleration = false;
bool vgui_accelerate::vgui_mfc_acceleration = true;

static int accelerator_level = 0;
static vgui_accelerate* vgui_accelerator = 0;
vgui_accelerate* vgui_accelerate::instance()
{
  if (!vgui_accelerator) vgui_accelerator = new vgui_accelerate;
  return vgui_accelerator;
}

void vgui_accelerate::register_accelerator(vgui_accelerate* p, int level)
{
  if (level > accelerator_level) {
    delete vgui_accelerator;
    vgui_accelerator = p;
    accelerator_level = level;
  }
}

// Default implementations (return false to indicate that a non-accelerated path was used.)

vgui_accelerate::vgui_accelerate()
{
  //  vcl_cerr << __FILE__ ": vgui_accelerate::vgui_accelerate()" << vcl_endl;
}

vgui_accelerate::~vgui_accelerate()
{
  //  vcl_cerr << __FILE__ ": vgui_accelerate::~vgui_accelerate()" << vcl_endl;
}

bool
vgui_accelerate::vgui_glClear( GLbitfield mask )
{
  glClear(mask);
  return false;
}

bool
vgui_accelerate::vgui_glDrawPixels( GLsizei width, GLsizei height, GLenum format, GLenum type, const GLvoid *pixels )
{
  glDrawPixels( width, height, format, type, pixels);
  return false;
}

// 32 bit RGBA seems to be acceptable/fast on most platforms.
// - u97mb RGBA is not acceptable on Mesa(too slow) so we use GL_RGB instead
bool
vgui_accelerate::vgui_choose_cache_format( GLenum* format, GLenum* type)
{
#if VGUI_MESA
  (*format) = GL_RGB;
#else
  (*format) = GL_RGBA;
#endif
  (*type) = GL_UNSIGNED_BYTE;
  return false;
}

// These functions are used in X11/Mesa to speed up overlay emulation. They
// return false to indicate to overlay_biscuit that a default emulation must be used.
bool vgui_accelerate::vgui_copy_back_to_aux()
{
  return false;
}

bool vgui_accelerate::vgui_copy_aux_to_back()
{
  return false;
}
