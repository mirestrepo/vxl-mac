#ifndef vidl_frame_h
#define vidl_frame_h

//:
// \file 
// \author Nicolas Dano, march 1999
//
// \verbatim
// Modifications
// Julien Esteve, May 2000 Ported from TargetJr
// 10/4/2001 Ian Scott (Manchester) Converted perceps header to doxygen
// \endverbatim


#include <vbl/vbl_ref_count.h>
#include <vil/vil_image.h>
#include <vidl/vidl_frame_sptr.h>
#include <vidl/vidl_codec.h>

//: A single frame of a Video Sequence.
class vidl_frame : public vbl_ref_count
{
   // PUBLIC INTERFACE
   public:

        // Constructors/Initializers/Destructors
        vidl_frame(int position, vidl_codec_sptr coder);
        ~vidl_frame();

        vil_image get_image();
        virtual bool get_section(
                void* ib, int x0, int y0, int width, int heigth) const;

        vidl_codec_sptr get_codec() {return coder_;}

        inline char get_image_class()const { return(coder_->get_image_class()); }
        inline char get_format() const     { return(coder_->get_format()); }
        inline int  width() const     { return(coder_->width()); }
        inline int  height() const     { return(coder_->height()); }
        inline int  get_bits_pixel() const { return(coder_->get_bits_pixel()); }
        inline int  get_bytes_pixel()const { return(coder_->get_bytes_pixel()); }

        //: Return the index of this frame in the movie.
        int get_real_frame_index() const { return position_; }

   protected:

        // Data Members
        const int position_;
        vidl_codec_sptr coder_;
        vil_image image_;
};

#endif // vidl_frame_h
