#ifndef vidl_codec_h
#define vidl_codec_h
//-----------------------------------------------------------------------------
//
// .NAME vidl_codec - Base for video coder/decoder
// .LIBRARY vidl
// .HEADER vxl package
// .INCLUDE vidl/vidl_codec.h
//
// .SECTION Description
//   A vidl_codec is a pure virtual class defining the codecs of video
//
// .SECTION See also
//   vidl_io
//   vidl_image_list_codec
//   vidl_avicodec
//
// .SECTION Author
//   Nicolas Dano, September 1999
//
// .SECTION Modifications
//   Julien Esteve, May 2000
//   Ported from TargetJr
//
//-----------------------------------------------------------------------------


#include <vcl_string.h>
#include <vidl/vidl_codec_sptr.h>
#include <vbl/vbl_ref_count.h>

class vidl_movie;
class vidl_image_list_codec;


class vidl_codec :  public vbl_ref_count
{
public:

  // Constructors/Initializers/Destructors-------------------------------------
  vidl_codec() {clear_strings();}
  ~vidl_codec() {}

  //=====================================================
  // Casting methods -- lets use a standard form for these, namely 
  // CastToXXX, where XXX is the subclass
  virtual vidl_image_list_codec* castto_vidl_image_list_codec() { return NULL; }

  // Data Control--------------------------------------------------------------

  inline void set_number_frames(int n = 0) { numberframes = n;}
  inline void set_name(const char* n = "") {  }
  inline void set_description(const char* d = "") { }
  inline void set_format(char f = '\0') { format = f; }
  inline void set_image_class(char t = 'M') { Class = t; }
  inline void set_bits_pixel(int d = 0) { B = d; }
  inline void set_width(int x = 0) { X = x; }
  inline void set_height(int y = 0) { Y = y; }
  inline void set_size_z(int z = 1) { Z = z; }
  inline void set_size_t(int t = 1) { T = t; }

  // Data Access---------------------------------------------------------------

  inline int length() const {return numberframes; }
  inline const char* get_name() const  { return(name)?name:""; }
  inline const char* get_description() const 
	{ 	
	return(description)?description:""; 
	}

  inline char get_image_class()const { return(Class); } 
  inline char get_format() const     { return(format); }
  inline int  width() const          { return(X); }
  inline int  height() const     { return(Y); }
  inline int  get_bits_pixel() const { return(B); }
  inline int  get_bytes_pixel()const { return((B+7)/8); }

  virtual bool get_section(
	int position, 
	void* ib, 
	int x0, 
	int y0, 
	int width, 
	int heigth) const = 0;

  virtual int put_section(
	int position, 
	void* ib, 
	int x0, 
	int y0, 
	int xs, 
	int ys) = 0;

  virtual const char* type() = 0;

  // IO
  virtual vidl_codec_sptr load(const char* fname, char mode = 'r' ) = 0;
  virtual bool save(vidl_movie* movie, const char* fname) = 0;
  virtual bool probe(const char* fname) = 0;
  
private:

  inline void clear_strings() { name = description = date_time = NULL; }

  char*      name;             // Video Name
  char*      description;      // Video Descriptor
  char*      date_time;        // Date/Time Stamp
  char       format;           // Video format
  char       Class;            // Video class
  int        B;                // Pixel Precision
  int        X,Y,Z,T;          // Frame Size (width,height,up,time)
  int        numberframes;     // Length of the sequence

};
#endif // vidl_codec_h




