#include "rgrl_view.h"
//:
// \file
// \brief  Represent a "view" of the registration problem.
// \author Chuck Stewart
// \date   25 Nov 2002

#include "rgrl_estimator.h"

rgrl_view::
rgrl_view( rgrl_mask_box           const& from_image_roi,
           rgrl_mask_box           const& to_image_roi,
           rgrl_mask_box           const& region,
           rgrl_mask_box           const& global_region,
           rgrl_estimator_sptr       xform_estimator,
           rgrl_transformation_sptr  xform_estimate,
           unsigned                  resolution,
           rgrl_transformation_sptr  reverse_estimate )
  : from_image_roi_( from_image_roi ),
    to_image_roi_( to_image_roi ),
    current_region_( region ),
    global_region_( global_region ),
    xform_estimator_( xform_estimator ),
    xform_estimate_( xform_estimate ),
    reverse_estimate_( reverse_estimate ),
    current_resolution_( resolution )
{
}


rgrl_mask_box const&
rgrl_view::
from_image_roi() const
{
  return from_image_roi_;
}


rgrl_mask_box const&
rgrl_view::
to_image_roi() const
{
  return to_image_roi_;
}


rgrl_mask_box const&
rgrl_view::
region() const
{
  return current_region_;
}

rgrl_mask_box const&
rgrl_view::
global_region() const
{
  return global_region_;
}

rgrl_estimator_sptr
rgrl_view::
xform_estimator() const
{
  return xform_estimator_;
}

rgrl_transformation_sptr
rgrl_view::
xform_estimate() const
{
  return xform_estimate_;
}

rgrl_transformation_sptr
rgrl_view::
reverse_xform_estimate() const
{
  return reverse_estimate_;
}

unsigned
rgrl_view::
resolution() const
{
  return current_resolution_;
}


bool
rgrl_view::
is_at_finest_resolution() const
{
  return current_resolution_ == 0;
}

bool
rgrl_view::
current_region_converged() const
{
  return current_region_ == global_region_;
}

bool
rgrl_view::
regions_converged_to(const rgrl_view& other) const
{
  // Check if the x0 and x1 of the current region diff from other region more
  // than one pixel in either x or y direction
  //
  bool current_region_changed =
    ( (this->current_region_.x0() - other.current_region_.x0()).inf_norm() > 1 ||
      (this->current_region_.x1() - other.current_region_.x1()).inf_norm() > 1);
  // Check if the x0 and x1 of the current global region diff from other
  // global region more than one pixel in either x or y direction
  //
  bool current_global_region_changed =
    ( (this->global_region_.x0() - other.global_region_.x0()).inf_norm() > 1 ||
      (this->global_region_.x1() - other.global_region_.x1()).inf_norm() > 1);

  return this->from_image_roi_.x0() == other.from_image_roi_.x0() &&
         this->from_image_roi_.x1() == other.from_image_roi_.x1() &&
         this->to_image_roi_.x0() == other.to_image_roi_.x0() &&
         this->to_image_roi_.x1() == other.to_image_roi_.x1() &&
         !current_region_changed &&
         !current_global_region_changed &&
         this->xform_estimator_->transformation_type() == other.xform_estimator_->transformation_type() &&
         this->current_resolution_ == other.current_resolution_;
}

bool
rgrl_view::is_valid() const
{
  return xform_estimator_ && xform_estimate_;
}

#if 0
bool
rgrl_view::
operator==( const rgrl_view& other ) const
{
  return this->from_image_roi_.x0() == other.from_image_roi_.x0() &&
         this->from_image_roi_.x1() == other.from_image_roi_.x1() &&
         this->to_image_roi_.x0() == other.to_image_roi_.x0() &&
         this->to_image_roi_.x1() == other.to_image_roi_.x1() &&
         this->current_region_.x0() == other.current_region_.x0() &&
         this->current_region_.x1() == other.current_region_.x1() &&
         this->global_region_.x0() == other.global_region_.x0() &&
         this->global_region_.x1() == other.global_region_.x1() &&
         this->xform_estimator_ == other.xform_estimator_ &&
//       this->xform_estimate_ == other.xform_estimate_ &&
         this->current_resolution_ == other.current_resolution_;
}

bool
rgrl_view::
operator!=( const rgrl_view& other ) const
{
  return !( *this == other );
}
#endif // 0
