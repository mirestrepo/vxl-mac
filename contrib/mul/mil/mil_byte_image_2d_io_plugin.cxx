#ifdef __GNUC__
#pragma implementation
#endif
//:
// \file
// \brief Interface for loading new image formats 
// This class provides an interface for loading images in new formats
// \author      Franck Bettinger
// \date        Sun Mar 17 22:57:00 2002
//
// \verbatim
// Modifications
// \endverbatim

#include "mil_byte_image_2d_io_plugin.h"
#include <vcl_vector.h>

//=======================================================================

static vcl_vector<mil_byte_image_2d_io_plugin*>
    *mil_byte_image_2d_of_plugins_list_ = 0;

//=======================================================================

mil_byte_image_2d_io_plugin::mil_byte_image_2d_io_plugin()
{
}

//=======================================================================

mil_byte_image_2d_io_plugin::~mil_byte_image_2d_io_plugin()
{
}

//=======================================================================

vcl_string mil_byte_image_2d_io_plugin::is_a() const
{
  return vcl_string("mil_byte_image_2d_io_plugin");
}

//=======================================================================

bool mil_byte_image_2d_io_plugin::loadTheImage (
    mil_image_2d_of<vil_byte>& image,
    const vcl_string & path, const vcl_string & filetype,
    const vcl_string & colour)
{
  if (mil_byte_image_2d_of_plugins_list_==0 ||
      is_a()!=vcl_string("mil_byte_image_2d_io_plugin")) 
  {
    return false;
  }

  for (unsigned int i=0;i<mil_byte_image_2d_of_plugins_list_->size();i++)
  {
    if (mil_byte_image_2d_of_plugins_list_->operator[](i)->loadTheImage(
        image,path,filetype,colour))
    {
      return true;
    }
  }

  return false;
}

//=======================================================================

void mil_byte_image_2d_io_plugin::register_plugin(
    mil_byte_image_2d_io_plugin* plugin)
{
  if (plugin==0 || plugin->is_a()==vcl_string("mil_byte_image_2d_io_plugin")) 
  {
    return;
  }

  if (mil_byte_image_2d_of_plugins_list_==0)
  {
    mil_byte_image_2d_of_plugins_list_ =
      new vcl_vector<mil_byte_image_2d_io_plugin*>(); 
  }

  mil_byte_image_2d_of_plugins_list_->push_back(plugin);
}

//=======================================================================

void mil_byte_image_2d_io_plugin::delete_all_plugins()
{
  if (mil_byte_image_2d_of_plugins_list_==0) return;
  unsigned int n = mil_byte_image_2d_of_plugins_list_->size();
  for (unsigned int i=0;i<n;++i)
    delete mil_byte_image_2d_of_plugins_list_->operator[](i);
  mil_byte_image_2d_of_plugins_list_->resize(0);

  // Clean up the list itself
  delete mil_byte_image_2d_of_plugins_list_;
  mil_byte_image_2d_of_plugins_list_=0;
}

//=======================================================================

bool mil_byte_image_2d_io_plugin::can_be_loaded(const vcl_string& filename)
{
  if (mil_byte_image_2d_of_plugins_list_==0 ||
      is_a()!=vcl_string("mil_byte_image_2d_io_plugin")) 
  {
    return false;
  }

  for (unsigned int i=0;i<mil_byte_image_2d_of_plugins_list_->size();i++)
  {
    if (mil_byte_image_2d_of_plugins_list_->operator[](i)->can_be_loaded(
        filename))
    {
      return true;
    }
  }
  return false;
}
