//-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation
#endif

#include "vgui_tableau.h"

#include <vcl_cassert.h>
#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <vcl_algorithm.h>

#include <vgui/vgui_macro.h>
#include <vgui/vgui_event.h>
#include <vgui/vgui_slot.h>
#include <vgui/vgui_menu.h>
#include <vgui/vgui_command.h>
#include <vgui/vgui_popup_params.h>

// change 'true' to 'false' to turn on debugging.
#define debug if (true) { } else vcl_cerr

// static data
// every tableau is on this array.
// must be a ptr as must live longer than any tableau_ref
static vgui_DLLDATA vcl_vector<vgui_tableau*>* all = 0;

vgui_tableau::vgui_tableau() 
  : references(0)
{
  debug << "vgui_tableau constructor: this = " << (void*)this << vcl_endl;

  // register :
  if (all == 0) {
    //vcl_cerr << __FILE__ ": CREATING tableau cache\n";
    all = new vcl_vector<vgui_tableau*>;
  }
  all->push_back(this);
}

vgui_tableau::~vgui_tableau()
{
  debug << "vgui_tableau destructor : this = " << (void*)this << vcl_endl;

  if (references != 0)
    vgui_macro_warning << "there are still " << references
		       << " references. this=" << (void*)this << vcl_endl;
  
  // deregister :
  vcl_vector<vgui_tableau*>::iterator i=vcl_find(all->begin(), all->end(), this);
  assert(i != all->end());
  all->erase(i);
  if (all->size() == 0) {
    //vcl_cerr << __FILE__ ": DELETING tableau cache\n";
    delete all;
    all = 0;
  }
}

void vgui_tableau::ref() const
{
  ++ const_cast<int &>(references);
}

void vgui_tableau::unref() const
{
  assert(references > 0); // fatal if not
  
  if (-- const_cast<int &>(references) == 0) {
    //vcl_cerr << "about to delete this=" << (void*)this << ", " << type_name() << vcl_endl;
    delete const_cast<vgui_tableau*>(this);
  }
}

//--------------------------------------------------------------------------------

// -- Override in subclasses to give the tableau some appearance and behaviour.
bool vgui_tableau::handle(vgui_event const &event) {
  vgui_macro_report_errors;
  
  switch (event.type) {
  case vgui_DRAW:
    return draw();

  case vgui_BUTTON_DOWN:
    return mouse_down (event.wx, event.wy, event.button, event.modifier);
  
  case vgui_MOTION:
    return motion     (event.wx, event.wy);

  case vgui_BUTTON_UP:
    return mouse_up   (event.wx, event.wy, event.button, event.modifier);

  case vgui_KEY_PRESS:
    if (event.key == '?') 
      return help();
    else
      return key_press(event.wx, event.wy, event.key, event.modifier);
    
  default:
    return false;
  }
}

bool vgui_tableau::mouse_down(int, int, vgui_button, vgui_modifier) { 
  debug << "vgui_tableau::mouse_down" << vcl_endl;
  return false;
}

bool vgui_tableau::mouse_up(int, int, vgui_button, vgui_modifier) {
  debug << "vgui_tableau::mouse_up" << vcl_endl;
  return false; 
}

bool vgui_tableau::motion(int, int) {
  debug << "vgui_tableau::motion" << vcl_endl;
  return false; 
}

bool vgui_tableau::key_press(int, int, vgui_key, vgui_modifier) {
  debug << "vgui_tableau::key_press" << vcl_endl;
  return false;
}

bool vgui_tableau::help() {
  debug << "vgui_tableau::help" << vcl_endl;
  return false;
}

bool vgui_tableau::draw() {
  debug << "vgui_tableau::draw" << vcl_endl;
  return false;
}



// -- Return the bounding box of this tableau.
// If infinite in extent, or nothing is drawn, or you can't be bothered to 
// implement it, return false.
bool vgui_tableau::get_bounding_box(float /*low*/[3], float /*high*/[3]) const {
  return false;
}

//--------------------------------------------------------------------------------

/*
void vgui_tableau::post(const vgui_message &m) {
  //  // send message to all observers :
  //  notify(m);
  
  // send message to all parents :
  vcl_vector<vgui_tableau*> parents;
  this->get_parents(parents);
  for (unsigned i=0;i<parents.size();i++)
    parents[i]->post(m);
}
*/

void vgui_tableau::post_message(char const *msg, void const *data) {
  vcl_vector<vgui_tableau_ref> ps;
  get_parents(&ps);
  for (unsigned i=0; i<ps.size(); ++i)
    ps[i]->post_message(msg, data);
}

void vgui_tableau::post_redraw() {
  vcl_vector<vgui_tableau_ref> ps;
  get_parents(&ps);
  for (unsigned i=0; i<ps.size(); ++i)
    ps[i]->post_redraw();
}

void vgui_tableau::post_overlay_redraw() {
  vcl_vector<vgui_tableau_ref> ps;
  get_parents(&ps);
  for (unsigned i=0; i<ps.size(); ++i)
    ps[i]->post_overlay_redraw();
}

//--------------------------------------------------------------------------------

// -- virtual. returns some sort of name.
vcl_string vgui_tableau::name() const {
  return file_name();
}

// -- virtual. returns the name of a file associated with some tableau below.
vcl_string vgui_tableau::file_name() const {
  return "(none)"; 
}

// -- virtual. used to provide an informative name for printouts, debugging
// etc.  Often it's type_name() + some representation of the essential state.
vcl_string vgui_tableau::pretty_name() const {
  return type_name();
}

// -- virtual. Return the name of the most derived (tableau) class. This
// ought never to be called as derived classes should implement type_name().
vcl_string vgui_tableau::type_name() const {
  static bool warned=false;
  if (!warned) {
    vgui_macro_warning << "WARNING: vgui_tableau::type_name() called" << vcl_endl;
    warned=true;
  }
  return "vgui_tableau"; 
}

//--------------------------------------------------------------------------------

// push parents onto the given vcl_vector.
void vgui_tableau::get_parents(vcl_vector<vgui_tableau_ref> *v) const {
  vgui_slot::get_parents_of(const_cast<vgui_tableau*>(this),v);
}

// push children onto the given vcl_vector.
void vgui_tableau::get_children(vcl_vector<vgui_tableau_ref> *v) const {
  vgui_slot::get_children_of(const_cast<vgui_tableau*>(this),v);
}

// get the ith child, or return 0. supplied by special request from geoff.
vgui_tableau_ref vgui_tableau::get_child(unsigned i) const {
  vcl_vector<vgui_tableau_ref> children;
  get_children(&children);
  return i<children.size() ? children[i] : vgui_tableau_ref();
}

// virtual overridden by consenting parents :
bool vgui_tableau::add_child(vgui_tableau_ref const &) {
  return false;
}

//
bool vgui_tableau::remove_child(vgui_tableau_ref const&) {
  return false;
}

// -- This method is called when some part of the program (typically the slot
// mechanism) is about to forcibly replace a child of this tableau.
// The canonical reason to override this is in order to invalidate caches.
bool vgui_tableau::notify_replaced_child(vgui_tableau_ref const& /*old_child*/,
					 vgui_tableau_ref const& /*new_child*/)
{
  return true;
}

//--------------------------------------------------------------------------------
// @{ MENUS @}

// -- This method is for tableaux to implement if they want to _add_ some items to
// the popup menu. They can assign to or clear 'menu', but that is not recommended
// as it would remove what other tableaux put there.
// The recommended usage is to .add() items or to .include() another menu.
// ** this is an interface method. it abstracts a behaviour. **
void vgui_tableau::add_popup(vgui_menu &/*menu*/) {
  // do nothing by default.
}

// -- Gets popup menu for this tableau. If recurse is, true, recursively add the
// popup menus for children and children's children etc.
// ** this is a mixin method. it does some work for you. **
void vgui_tableau::get_popup(vgui_popup_params const &params, vgui_menu &menu) {
  // extract this tableau's popup menu into 'submenu'.
  vgui_menu submenu;
  add_popup(submenu);

  if (params.nested) { // nested menu style.
    // get list of children of this tableau.
    vcl_vector<vgui_tableau_ref> children;
    get_children(&children);

    if (params.defaults && !children.empty())
      submenu.separator();
    
    for (unsigned i=0; i<children.size(); ++i)
      if (children[i])
        children[i]->get_popup(params, submenu);

    menu.add(type_name(), submenu);
  }
  else {
    // not nested.
    if (submenu.size() > 0) // do not add empty submenus.
      menu.add(type_name(), submenu);
    
    if (params.recurse) {
      vcl_vector<vgui_tableau_ref> children;
      get_children(&children);
      
      for (unsigned i=0; i<children.size(); ++i)
	if (children[i]) 
          children[i]->get_popup(params, menu);
    }
  }
  
}

void vgui_tableau::adopt (vgui_tableau_ref const &) const
{
}

void vgui_tableau::disown(vgui_tableau_ref const &) const
{
}

//--------------------------------------------------------------------------------

// prints pretty name and address of tableau.
// eg : pig.jpg[vgui_composite:0xeffff728]
vcl_ostream &operator<<(vcl_ostream &os, vgui_tableau_ref const &t)
{
  if (t)
    return os << t->pretty_name() << '[' << t->type_name() << ':' << static_cast<const void*>(t.operator->()) << ']';
  else
    return os << "(empty vgui_tableau_ref)" << vcl_flush;
}

//--------------------------------------------------------------------------------

// push all tableaux onto v :
void vgui_tableau::get_all(vcl_vector<vgui_tableau_ref> *v) {
  //v->insert(v->begin(), all->begin(), all->end());
  for (unsigned i=0; i<all->size(); ++i)
    v->push_back((*all)[i]);
}

// returns true if the given address points to a valid tableau.
bool vgui_tableau::exists(vgui_tableau_ref const& ptr) {
  return vcl_find(all->begin(), all->end(), ptr.operator->()) != all->end();
}
