// This is core/vgui/vgui_key.h
#ifndef vgui_key_h_
#define vgui_key_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \brief Enumeration of possible key strokes.

#include <vcl_iosfwd.h>

//: Enumeration of possible key strokes.
enum vgui_key {
  vgui_KEY_NULL = 0,
  vgui_ESC=27, /* */ vgui_ESCAPE=27,
  vgui_TAB='\t',
  vgui_RETURN='\r',
  vgui_NEWLINE='\n',
  vgui_KEY_XXX=0x100, // non-ascii encoded keys below :
  vgui_F1, vgui_F2 , vgui_F3 , vgui_F4 , // function keys
  vgui_F5, vgui_F6 , vgui_F7 , vgui_F8 ,
  vgui_F9, vgui_F10, vgui_F11, vgui_F12,
  vgui_CURSOR_LEFT, vgui_CURSOR_UP, vgui_CURSOR_RIGHT, vgui_CURSOR_DOWN,  // cursor movement
  vgui_PAGE_UP, /* */ vgui_PGUP=vgui_PAGE_UP,
  vgui_PAGE_DOWN, /* */ vgui_PGDN = vgui_PAGE_DOWN,
  vgui_HOME, vgui_END,
  vgui_DELETE, vgui_INSERT  // other things
};

//: Given a letter character, this function returns the control character.
//
//  Use vgui_key_CTRL('a') or vgui_key_CTRL('A') to signify control+A.
//  Note, the vgui_key in a vgui_event is always a lower case letter (see
//  vgui_event.h), so don't compare event.key to a vgui_key_CTRL'ed letter.
//  You may compare a vgui_key_CTRL'ed to an event.ascii_key.
vgui_key vgui_key_CTRL(int character);

vcl_ostream& operator<<(vcl_ostream& s, vgui_key k);

#endif // vgui_key_h_
