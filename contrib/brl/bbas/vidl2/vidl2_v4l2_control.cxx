//:
// \file
//
// \author Antonio Garrido
// \verbatim
//  Modifications
//     10 Nov 2008 Created (A. Garrido)
//\endverbatim


#include "vidl2_v4l2_control.h"
#include <vcl_cstring.h>
#include <vcl_iostream.h>
#include <vcl_cstdio.h>

extern "C" {
#include <sys/ioctl.h>
};

vidl2_v4l2_control * vidl2_v4l2_control::new_control(const v4l2_queryctrl& ctr, int f)
{

  if ( (ctr.flags & V4L2_CTRL_FLAG_DISABLED) || (ctr.flags & V4L2_CTRL_FLAG_INACTIVE))
    return 0; 
  switch (ctr.type) { 
    case V4L2_CTRL_TYPE_INTEGER:
      return new vidl2_v4l2_control_integer(ctr,f);
      break;
    case V4L2_CTRL_TYPE_BOOLEAN:
      return new vidl2_v4l2_control_boolean(ctr,f);
      break;
    case V4L2_CTRL_TYPE_MENU:
      {
      vidl2_v4l2_control_menu *p= new vidl2_v4l2_control_menu(ctr,f);
      if (p->n_items()==0) {
        delete p;
        p= 0;
      }
      return p;
      }
      break;
    case V4L2_CTRL_TYPE_BUTTON:
      return new vidl2_v4l2_control_button(ctr,f);
      break;
  }

  return 0;
}

void vidl2_v4l2_control::set_value(int v) const
{
  struct v4l2_control control;
  vcl_memset (&control, 0, sizeof (control));
  control.id = ctrl_.id;
  control.value = v;
  ioctl (fd, VIDIOC_S_CTRL, &control); // error ignored
}

int vidl2_v4l2_control::get_value() const
{
  struct v4l2_control control;
  vcl_memset (&control, 0, sizeof (control));
  control.id = ctrl_.id;
  ioctl (fd, VIDIOC_G_CTRL, &control); // error ignored
  return control.value;
}

// ----------------- Control integer ---------------

void vidl2_v4l2_control_integer::set(int value) const
{
  if (value<ctrl_.minimum) value= ctrl_.minimum;
  else if (value>ctrl_.maximum) value= ctrl_.maximum;
       else value= ctrl_.minimum+(value-ctrl_.minimum)/ctrl_.step * ctrl_.step;
  set_value(value);
}

void vidl2_v4l2_control_integer::set_100(int value) const
{
  if (value<=0) value= ctrl_.minimum;
  else if (value>=100) value= ctrl_.maximum;
       else value= ctrl_.minimum+(ctrl_.maximum-ctrl_.minimum)*value/100;
  set_value(value);
}

vcl_string vidl2_v4l2_control_integer::description() const
{
  char cad[256];
  snprintf(cad,256,"Control \"%s\": integer (min: %d, max: %d, step: %d, default: %d)",
                  (const char *) ctrl_.name, minimum(), maximum(), step(), default_value());
  return cad;
}

// ----------------- Control menu ---------------

vidl2_v4l2_control_menu::vidl2_v4l2_control_menu(const v4l2_queryctrl& ctr, int f):
                                                                vidl2_v4l2_control(ctr,f) 
{
  struct v4l2_querymenu menu;
  vcl_memset(&menu, 0, sizeof (menu));
  menu.id= ctrl_.id;
  for (menu.index = 0; menu.index <= ctrl_.maximum;menu.index++) {
                if (0 == ioctl (fd, VIDIOC_QUERYMENU, &menu)) {
                        items.push_back((char *)menu.name);
                } else {
                        vcl_cerr << "VIDIOC_QUERYMENU" << vcl_endl;
                        items.clear(); // control menu is not added to the list
                        return; 
                }
        }
}

vcl_string vidl2_v4l2_control_menu::description() const
{
  char cad[256];
  snprintf(cad,256,"Control \"%s\": menu (%d items, default: %d)",
                  (const char *) ctrl_.name, n_items(), default_value());
  return cad;
}

// ----------------- Control boolean ---------------

