 //-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation
#endif
//
// .NAME vgui_soview
// Author: Philip C. Pritchett, RRG, University of Oxford
// Created: 24 Mar 99
//
//-----------------------------------------------------------------------------

#include "vgui_soview.h"

#include <vcl_iostream.h>
#include <vcl_algorithm.h>
#include <vcl_map.h>

#include <vgui/vgui_gl.h>
#include <vgui/vgui_observer.h>
#include <vgui/vgui_style.h>
#include <vgui/vgui_style_factory.h>

#define VGUI_STATIC_OBJECT(T, var) \
static T& var () { \
  static T * t = 0; \
  if (t == 0) t = new T(); \
  return *t; \
}

#define AWF_USE_MAP 0

unsigned vgui_soview::current_id = 1;

#if AWF_USE_MAP
typedef vcl_map<unsigned, void*, vcl_less<unsigned> > Map_soview;
VGUI_STATIC_OBJECT(Map_soview, object_map);
#else
typedef vcl_vector<void* > Map_soview;
VGUI_STATIC_OBJECT(Map_soview, object_map);
#endif


unsigned vgui_soview::create_id() {
  unsigned nid = current_id;
  current_id++;
  return nid;
}

void vgui_soview::add_id() {
  id = create_id();
#if AWF_USE_MAP
  object_map().insert(Map_soview::value_type(id, this));
#else
  object_map().resize(id * 2);
  object_map()[id] = this;
#endif
}

vgui_soview* vgui_soview::id_to_object(unsigned id) {

#if AWF_USE_MAP
  Map_soview::iterator i = object_map().find(id);
  if (i != object_map().end()) {
    return static_cast<vgui_soview*>((*i).second);
  }
#else
  if (id < object_map().size()) {
    return static_cast<vgui_soview*>(object_map()[id]);
  }
#endif

  return 0;
}


vgui_soview::vgui_soview() :  style(0) {
  add_id();
}

vcl_ostream& vgui_soview::print(vcl_ostream& s) const
{
  return s << "id " << id;
}

vcl_string vgui_soview::type_name() const  {
  // this should never be called. derived classes should implement type_name().
  static bool warned=false;
  if (!warned) {
    vcl_cerr << __FILE__ " : WARNING : vgui_soview::type_name() called" << vcl_endl;
    warned=true;
  }
  return "vgui_soview";
}

void vgui_soview::load_name() {
  glLoadName(id);
}


void vgui_soview::set_style(vgui_style* newstyle) {
  // inform style factory that this vgui_soview now has style s
  vgui_style_factory::instance()->change_style(this, newstyle, style);

  style = newstyle;
}

void vgui_soview::set_colour(float r, float g, float b) {

  vgui_style* newstyle =
    vgui_style_factory::get_style(r, g, b, style->point_size, style->line_width);

  vgui_style_factory::change_style(this, newstyle, style);

  style = newstyle;
}

void vgui_soview::set_point_size(float s) {

  vgui_style* newstyle =
    vgui_style_factory::get_style(style->rgba[0], style->rgba[1], style->rgba[2],
                                  s, style->line_width);

  vgui_style_factory::change_style(this, newstyle, style);

  style = newstyle;
}

void vgui_soview::set_line_width(float w) {

  vgui_style* newstyle =
    vgui_style_factory::get_style(style->rgba[0], style->rgba[1], style->rgba[2],
                                  style->point_size, w);

  vgui_style_factory::change_style(this, newstyle, style);

  style = newstyle;
}


vgui_style* vgui_soview::get_style() {
  return style;
}

//
const void * const vgui_soview::msg_select="x";
const void * const vgui_soview::msg_deselect="x";



//--------------------------------------------------------------------------------

// Observers. Rather than storing a vcl_list/vcl_vector/whatever of observers on each
// soview, we maintain a static multimap from soviews to observers. This makes
// the soviews smaller and optimizes the common case of soviews with no
// observers.
// fsm: I have not tested this code yet -- where is it used?
#include <vcl_map.h>

// vc++ static data members have some peculiarities, so
// we use this traditional work-around instead :
typedef vcl_multimap<void *, void *, vcl_less<void *> > mmap_Pv_Pv;
static mmap_Pv_Pv &the_map() {
  static mmap_Pv_Pv *ptr = 0;
  if (!ptr)
    ptr = new mmap_Pv_Pv;
  return *ptr;
}

void vgui_soview::attach(vgui_observer* o) {
  the_map().insert(mmap_Pv_Pv::value_type(this, o));
}

void vgui_soview::detach(vgui_observer* o) {
  mmap_Pv_Pv::iterator lo = the_map().lower_bound(this);
  mmap_Pv_Pv::iterator hi = the_map().upper_bound(this);
  for (mmap_Pv_Pv::iterator i=lo; i!=hi; ++i)
    if (
        (*i).second == o
        ) {
      the_map().erase(i);
      return;
    }

  // not found :
  vcl_cerr << __FILE__ " : no such observer on this soview" << vcl_endl;
}

void vgui_soview::get_observers(vcl_vector<vgui_observer*>& vobs) const {
  mmap_Pv_Pv::const_iterator lo = the_map().lower_bound( const_cast<vgui_soview*>(this) );
  mmap_Pv_Pv::const_iterator hi = the_map().upper_bound( const_cast<vgui_soview*>(this) );
  for (mmap_Pv_Pv::const_iterator i=lo; i!=hi; ++i)
    vobs.push_back( static_cast<vgui_observer*>((*i).second) );
}

// These two method could be optimized a bit.
void vgui_soview::notify() const {
  vcl_vector<vgui_observer*> vobs;
  get_observers(vobs);
  for (unsigned i=0; i<vobs.size(); ++i)
    vobs[i]->update();
}

void vgui_soview::notify(vgui_message const &msg) const {
  vcl_vector<vgui_observer*> vobs;
  get_observers(vobs);
  for (unsigned i=0; i<vobs.size(); ++i)
    vobs[i]->update(msg);
}
