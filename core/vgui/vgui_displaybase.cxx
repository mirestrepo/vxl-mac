#ifdef __GNUC__
#pragma implementation
#endif
//-*- c++ -*-------------------------------------------------------------------
//
// .NAME vgui_displaybase
// Author: Philip C. Pritchett, RRG, University of Oxford
// Created: 14 Sep 99
//
//-----------------------------------------------------------------------------

#include "vgui_displaybase.h"

#include <vcl_vector.h>
#include <vcl_algorithm.h>

#include <vul/vul_sprintf.h>
#include <vbl/vbl_bool_ostream.h>

#include <vgui/vgui_gl.h>
#include <vgui/vgui_glu.h>
#include <vgui/vgui_event.h>
#include <vgui/vgui_message.h>
#include <vgui/vgui_style.h>
#include <vgui/vgui_drag_mixin.h>
#include <vgui/vgui_projection_inspector.h>
#include <vgui/vgui_image_tableau.h>
#include <vgui/vgui_soview.h>
#include <vgui/vgui_style_factory.h>

bool vgui_displaybase_selection_callback::select(unsigned)
{
  return false;
}

bool vgui_displaybase_selection_callback::deselect(unsigned)
{
  return false;
}

bool vgui_displaybase_selection_callback::deselect_all()
{
  return false;
}

static bool debug=false;

vgui_displaybase::vgui_displaybase()
{
  id = vgui_soview::create_id();

  gl_mode = GL_RENDER;
  highlighted = 0;
  gl_display_list = GL_INVALID_VALUE;

  cb_ = 0;
}


vgui_displaybase::~vgui_displaybase()
{
}

void vgui_displaybase::set_selection_callback(vgui_displaybase_selection_callback* cb)
{
  cb_ = cb;
}


void vgui_displaybase::add(vgui_soview* object)
{

  vcl_vector<vgui_soview*>::iterator i = vcl_find(objects.begin(), objects.end(), object);
  if (i == objects.end())
  {
    objects.push_back(object);
  }
}

void vgui_displaybase::remove(vgui_soview* object)
{
  if (object->get_id() == highlighted)
  {
    // if the point to be removed is the currently highlighted
    // one, zero out the 'highlighted' field. otherwise the
    // point will still be rendered when the pointer moves.
    highlighted = 0;
  }

  // must first unselect the object, if it's selected
  deselect( object->get_id());

  vcl_vector<vgui_soview*>::iterator i = vcl_find(objects.begin(), objects.end(), object);
  if (i != objects.end())
  {
    objects.erase(i);
  }
}


void vgui_displaybase::clear()
{
  highlighted = 0;
  deselect_all();
  objects.clear();
}

void vgui_displaybase::draw_soviews_render()
{
  if (debug)
    vcl_cerr << "vgui_style_factory::use_factory : "
             << vbl_bool_ostream::true_false(vgui_style_factory::use_factory)
             << vcl_endl;

  if (vgui_style_factory::use_factory)
  {
    vcl_vector<vgui_style*> styles_copy;

    vgui_style_factory::get_styles(styles_copy);
    if (debug) vcl_cerr << "found " << styles_copy.size() << " styles " << vcl_endl;

    // get all the styles held by the style factory
    for (vcl_vector<vgui_style*>::iterator s_iter = styles_copy.begin();
         s_iter != styles_copy.end(); ++s_iter)
    {
      vgui_style* style = *s_iter;

      glColor3f(style->rgba[0], style->rgba[1], style->rgba[2]);
      glPointSize(style->point_size);
      glLineWidth(style->line_width);

      vcl_vector<vgui_soview*> soviews;
      vgui_style_factory::get_soviews(*s_iter, soviews);

      if (debug) vcl_cerr << "found " << soviews.size() << " soviews with this style" << vcl_endl;

      // for each soview with this style
      for (vcl_vector<vgui_soview*>::iterator so_iter = soviews.begin();
           so_iter != soviews.end(); ++so_iter)
      {
        vgui_soview *so = *so_iter;

        // only draw the soview if it is displayed by THIS vcl_list
        vcl_vector<vgui_soview*>::iterator i = vcl_find(objects.begin(), objects.end(), so);
        if (i != objects.end())
        {
          if (!is_selected(so->get_id()))
            so->draw();
        }
      }
    }

    if (debug) vcl_cerr << "setting color" << vcl_endl;
    if (debug) vcl_cerr << "drawing " << selections.size() << " selected soviews" << vcl_endl;

    for (vcl_vector<unsigned>::iterator id_iter = selections.begin();
         id_iter != selections.end(); ++id_iter )
    {
      vgui_soview* so = vgui_soview::id_to_object(*id_iter);

      vgui_style* style = so->get_style();
      glPointSize(style->point_size);
      glLineWidth(style->line_width);
      glColor3f(1,0,0);

      so->draw();
    }
  }
  else // vgui_style_factory::use_factory == false
  {
    for (vcl_vector<vgui_soview*>::iterator so_iter=objects.begin();
         so_iter != objects.end(); ++so_iter)
    {
      vgui_soview *so = *so_iter;
      vgui_style* style = so->get_style();
      glColor3f(style->rgba[0], style->rgba[1], style->rgba[2]);
      glPointSize(style->point_size);
      glLineWidth(style->line_width);

      if (is_selected(so->get_id()))
        glColor3f(1, 0, 0);

      so->draw();
    }//  for all soviews
  }
}


void vgui_displaybase::draw_soviews_select()
{
  // push the name of this displaylist onto the name stack
  glPushName(id);
  for (vcl_vector<vgui_soview*>::iterator so_iter=objects.begin();
       so_iter != objects.end(); ++so_iter)
  {
    // only highlight if the so is selectable
    vgui_soview* so = *so_iter;
    if( so->get_selectable())
      {
        vgui_soview *so = *so_iter;
        glPushName(so->get_id());
        so->draw();
        glPopName();
      }
  }//  for all soviews

  // remove the name of the displaylist from the name stack
  glPopName();
}


bool vgui_displaybase::handle(const vgui_event& e)
{
  if (e.type == vgui_DRAW)
  {
    if (gl_mode == GL_SELECT)
      draw_soviews_select();
    else
      draw_soviews_render();

#if 0
    else if (gl_display_list == GL_INVALID_VALUE) {
      gl_display_list = glGenLists(1);
      glNewList(gl_display_list, GL_COMPILE_AND_EXECUTE);
      draw_soviews();
      glEndList();
    }// gl_display_list == GL_INVALID_VALUE
    else {
      glCallList(gl_display_list);
    }
#endif

    return true;
  }
  else
    return vgui_tableau::handle(e);
}

bool vgui_displaybase::is_selected(unsigned id)
{
  vcl_vector<unsigned>::iterator result = vcl_find(selections.begin(), selections.end(), id);
  return (result != selections.end());
}

vcl_vector<unsigned> const & vgui_displaybase::get_selected() const
{
  return selections;
}

vcl_vector<vgui_soview*> vgui_displaybase::get_selected_soviews() const
{
  vcl_vector<vgui_soview*> svs;
#if 0
  for (vcl_vector<unsigned>::const_iterator s_iter = selections.begin();
       s_iter != selections.end(); ++s_iter )
  {
    unsigned id = *s_iter;
    vgui_soview* so = vgui_soview::id_to_object(id);
    svs.push_back(so);
  }
#else
  for (unsigned i=0; i<selections.size(); ++i)
  {
    svs.push_back(vgui_soview::id_to_object(selections[i]));
  }
#endif
  return svs;
}

vcl_vector<vgui_soview*> const & vgui_displaybase::get_all() const
{
  return objects;
}

vcl_vector<unsigned> const vgui_displaybase::get_all_ids() const
{
  vcl_vector<unsigned> ids;
  for(unsigned int i=0; i< objects.size(); i++)
    ids.push_back( objects[i]->get_id());

  return ids;
}

bool vgui_displaybase::select(unsigned id)
{
  vcl_vector<unsigned>::iterator result = vcl_find(selections.begin(), selections.end(), id);
  if (result == selections.end())
  {
    // add selection to vcl_list
    selections.push_back(id);

    // notify so's observers
    vgui_soview* so = vgui_soview::id_to_object(id);

    if( so->get_selectable())
      {
        vgui_message msg;
        //msg.text = "soview select";
        msg.user = (void*) &vgui_soview::msg_select;
        so->notify(msg);

        if (cb_) cb_->select(id);
      }
  }

  return true;
}

bool vgui_displaybase::deselect(unsigned id)
{
  vcl_vector<unsigned>::iterator result = vcl_find(selections.begin(), selections.end(), id);
  if (result != selections.end())
  {
    // remove selection from vcl_list
    selections.erase(result);

    // notify so's observers
    vgui_soview* so = vgui_soview::id_to_object(id);
    vgui_message msg;
    //msg.text = "soview deselect";
    msg.user = (void*) &vgui_soview::msg_deselect;
    so->notify(msg);

    if (cb_) cb_->deselect(id);
  }

  return true;
}

bool vgui_displaybase::deselect_all()
{
  // this is a bit inelegant but you have to make a copy
  // of the selections vcl_list as sending the deselect message
  // may actually change the selections

  vcl_vector<unsigned> oldselections = selections;

  for (vcl_vector<unsigned>::iterator s_iter = oldselections.begin();
       s_iter != oldselections.end(); ++s_iter )
  {
    unsigned id = *s_iter;

    // notify so's observers
    vgui_soview* so = vgui_soview::id_to_object(id);
    vgui_message msg;
    msg.user = (void*)&vgui_soview::msg_deselect;
    so->notify(msg);

    if (cb_) cb_->deselect(id);
  }

  selections.clear();
  this->post_redraw();

  return true;
}


bool vgui_displaybase::is_highlighted(unsigned id)
{
  return (id == highlighted);
}

unsigned vgui_displaybase::get_highlighted()
{
  return highlighted;
}

vgui_soview* vgui_displaybase::get_highlighted_soview()
{
    return vgui_soview::id_to_object(highlighted);
}

bool vgui_displaybase::highlight(unsigned id)
{
  highlighted = id;
  return true;
}

vgui_soview* vgui_displaybase::contains_hit(vcl_vector<unsigned> names)
{
#if 0
  vcl_cerr << "vgui_displaybase::contains_hit names ";
  copy(names.begin(), names.end(), ostream_iterator<unsigned>(vcl_cerr, " "));
  vcl_cerr << vcl_endl;
#endif

  for (vcl_vector<vgui_soview*>::iterator i = objects.begin() ;
       i != objects.end() ; ++i)
  {
    // get id of soview
    unsigned soview_id = (*i)->get_id();
    //vcl_cerr << "vgui_displaybase::contains_hit soview_id" << soview_id << vcl_endl;

    vcl_vector<unsigned>::iterator ni = vcl_find(names.begin(), names.end(), soview_id);
    if (ni != names.end())
      return *i;
  }

  return 0;
}

