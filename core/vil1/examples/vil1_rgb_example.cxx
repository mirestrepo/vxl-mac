// Example use of the vil1_rgb and vil1_rgb_cell classes.
//
// Author: Peter Vanroose, KULeuven/ESAT, December 1999

#include <vcl_iostream.h>
#include <vil1/vil1_rgb.h>
typedef vil1_rgb<unsigned char> vil1_rgb_cell;

static char* as_hex(vil1_rgb_cell const&);

int main(int /*argc*/, char** /*argv*/)
{
  // The "normal" vil1_rgb class, with ubyte cells, is called vil1_rgb_cell;
  // its constructor takes the R, G and B values to be set:
  vil1_rgb_cell orange(255,140,0);
  // A ubyte (= grey pixel) value is automatically converted to an vil1_rgb_cell:
  vil1_rgb_cell black = (unsigned char)0;
  vil1_rgb_cell white = (unsigned char)255;
  vil1_rgb_cell grey = (unsigned char)190;

  // Conversely, an vil1_rgb_cell can be converted to its ubyte grey equivalent:
  unsigned char orange_grey = orange.grey();

  // Write to a vcl_ostream: the output as the format [R G B].
  vcl_cout << "/* XPM */\n"
           << "/* " << orange << black << white << grey
           << (int)orange_grey << " */\n";

  // And now an .xpm file using these vil1_rgb_cells, and the function as_hex:
  vcl_cout << "static char* example_rgb[] = {\n\"20 8 4 1\",\n"
           << "\" \tc " << as_hex(white) << "\",\n"
           << "\"*\tc " << as_hex(black) << "\",\n"
           << "\".\tc " << as_hex(orange)<< "\",\n"
           << "\"/\tc " << as_hex(grey)  << "\",\n"
           << "\"/ /.*** / /.* /.* / \",\n"
           << "\" /.* /.* / .*/.* / /\",\n"
           << "\"/.* / /.* /.*.* / / \",\n"
           << "\" .*/ / .*/ .** / / /\",\n"
           << "\"/.* / /.* /.*.* / / \",\n"
           << "\" .*/ / .*/ .*/.* / /\",\n"
           << "\"/ .*/ .*/ /.* /.* / \",\n"
           << "\" / .***/.* .*/ /.* .\"\n};\n";
  return 0;
}

// This function writes, e.g., vil1_rgb_cell(177,49,97) as "#b13161":

char* as_hex(vil1_rgb_cell const& rgb) {
  // The data members r, g and b of an vil1_rgb_cell are public:
  unsigned char r = rgb.r;
  unsigned char g = rgb.g;
  unsigned char b = rgb.b;

  // And now some magic char manipulations, to obtain hex values:
  static char s[] = "#000000";
  s[1] = '0'+(r/16); if (s[1] > '9') s[1] += 'a'-'9'-1;
  s[2] = '0'+(r%16); if (s[2] > '9') s[2] += 'a'-'9'-1;
  s[3] = '0'+(g/16); if (s[3] > '9') s[3] += 'a'-'9'-1;
  s[4] = '0'+(g%16); if (s[4] > '9') s[4] += 'a'-'9'-1;
  s[5] = '0'+(b/16); if (s[5] > '9') s[5] += 'a'-'9'-1;
  s[6] = '0'+(b%16); if (s[6] > '9') s[6] += 'a'-'9'-1;
  return s;
}

