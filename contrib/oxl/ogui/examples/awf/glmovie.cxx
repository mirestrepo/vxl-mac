//:
// \file
// \author Andrew W. Fitzgibbon, Oxford RRG
// \date   21 May 99
//-----------------------------------------------------------------------------

#include <vcl_vector.h>
#include <vcl_cstdlib.h> // for vcl_abort
#include <vcl_algorithm.h> // for vcl_min

#include <vul/vul_file.h>
#include <vul/vul_printf.h>
#include <vul/vul_sprintf.h>
#include <vul/vul_arg.h>
#include <vul/vul_timer.h>

#include <vnl/vnl_math.h>

#include <vil1/vil1_memory_image_of.h>

#include <vgui/vgui_glut.h>

#include <oxp/ImageSequenceName.h>
#include <oxp/MovieFile.h>
#include <oxp/GXFileVisitor.h>

// extern void gl_draw_conic(double cx, double cy, double rx, double ry, double theta);

vul_arg<int>   a_start_frame("-s", "Start frame", 0);


/////////////////////////////////////////////////////////////////////////////

// Overlay mgmt
int nlayers = 1;
int layers[] = {
  GLUT_NORMAL,
  GLUT_OVERLAY,
};
int ovl_transparent = 0;
int ovl_opaque = 1;
int ovl_display_list = 1;
bool have_overlay = true;

void overlayDisplay()
{
  glClear(GL_COLOR_BUFFER_BIT);
  glCallList(ovl_display_list);

  glFlush();
}

/////////////////////////////////////////////////////////////////////////////

const int TEXTHEIGHT = 24;

MovieFile* moviefile = 0;
int frame = 1;
int dir = 1;
int num_frames = 0;
int window_width = 0;
int window_height = 0;
int mouse_x = 0; // Set by draw callbacks
int mouse_y = 0;
bool playing = true;
double pixel_zoom = 1.0;
double pixel_zoom_tx = 0;
double pixel_zoom_ty = TEXTHEIGHT;

vul_arg<bool> fast("-fast", "Go faster", false);
vul_arg<bool> gx_notext("-gx:notext", "No text", false);
vul_arg<double> a_gx_scale("-gscale", "GX file scale factor", 1.0);

int gx_list_base = 10; // Display lists for gx files start here
ImageSequenceName* gx_basename;
ImageSequenceName* output_basename;

void output(void* font, float x, float y, char const* s)
{
  glRasterPos2f(x, y);
  for (;*s;++s)
    glutBitmapCharacter(font, *s);
}

inline void output(float x, float y, char const* s)
{
  output(GLUT_BITMAP_HELVETICA_18, x, y, s);
}

void init()
{
  if (!fast()) {
    vcl_cerr << "High quality\n";
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    glEnable (GL_LINE_SMOOTH);
    glEnable (GL_BLEND);
    // glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendFunc (GL_SRC_ALPHA,  GL_ONE);
    glHint (GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
    glLineWidth (1.5);

    glShadeModel(GL_FLAT);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glPointSize(3);

  } else {
    vcl_cerr << "Fast\n";
    glDisable(GL_POINT_SMOOTH);
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_BLEND);

    glPointSize(3);
    glLineWidth(0.1);
  }
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glDepthRange(-0.1,1.0);
}

struct example_GXFileVisitor_OpenGL : public GXFileVisitor {
  virtual bool point(char const* type, float x, float y) {
    if (*type == 'p') {
      glBegin(GL_POINTS);
      glVertex2f(x,y);
      glEnd();
    } else if (*type == '+') {
      glBegin(GL_LINES);
      glVertex2f(x,y-point_radius);
      glVertex2f(x,y+point_radius);
      glVertex2f(x-point_radius,y);
      glVertex2f(x+point_radius,y);
      glEnd();
    }
        return true; //gsgs
  }

  virtual bool polyline(float const* x, float const* y, int n) {
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < n; ++i)
      glVertex2f(x[i], y[i]);
    glEnd();
        return true; //gsgs
  }

  virtual bool text(float x, float y, char const* text) {
    if (!gx_notext())
      output(GLUT_BITMAP_HELVETICA_10, x, y, text);
        return true; //gsgs
  }

  virtual bool set_color(float r, float g, float b) {
    glColor3f(r,g,b);
        return true; //gsgs
  }

  virtual bool set_point_radius(float r) {
    glPointSize(r);
        return true; //gsgs
  }

  virtual bool set_line_width(float w) {
    glLineWidth(w);
        return true; //gsgs
  }
};

bool drawframe(int frame)
{
  if (!moviefile->HasFrame(frame))
    return false;

  if (gx_basename)
    if (!vul_file::exists(gx_basename->name(frame).c_str())) {
      vcl_cerr << "glmovie: Graphics file " << gx_basename->name(frame) << " not there, bailing\n";
      return false;
    }

  vul_timer tic;

  glClear(GL_COLOR_BUFFER_BIT);

  // Draw image
  {
    int image_w = moviefile->GetSizeX(frame);
    int image_h = moviefile->GetSizeY(frame);
    glPixelZoom(pixel_zoom,-pixel_zoom);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, image_w);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
    glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);

    // negative rasterpos must be made +ive by skipping
    double rx = pixel_zoom_tx;
    double ry = pixel_zoom_ty;
    int rw = image_w;
    int rh = image_h;
    if (rx < 0) {
      int rasterpos_in_image_pixels = int(-rx / pixel_zoom + 1);
      rw = image_w - rasterpos_in_image_pixels;
      if (rw < 0) {
        rw = 0;
      } else {
        glPixelStorei(GL_UNPACK_SKIP_PIXELS, rasterpos_in_image_pixels);
        rx += rasterpos_in_image_pixels * pixel_zoom;
      }
    }

    if (ry < 0) {
      int rasterpos_in_image_pixels = int(-ry / pixel_zoom + 1);
      rh = image_h - rasterpos_in_image_pixels;
      if (rh < 0) {
        rh = 0;
      } else {
        glPixelStorei(GL_UNPACK_SKIP_ROWS, rasterpos_in_image_pixels);
        ry += rasterpos_in_image_pixels * pixel_zoom;
      }
    }

    glRasterPos2f(rx,ry);
    if (moviefile->GetBitsPixel() == 8) {
      vil1_memory_image_of<vxl_byte> img(image_w, image_h);
      moviefile->GetFrame(frame, img);
      glDrawPixels(rw, rh, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.get_buffer());
    } else {
      vil1_memory_image_of<vil1_rgb<vxl_byte> > img(image_w, image_h);
      moviefile->GetFrame(frame, img);
      glDrawPixels(rw, rh, GL_RGB,       GL_UNSIGNED_BYTE, img.get_buffer());
    }
  }

  // Parse GX file
  if (gx_basename) {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(pixel_zoom_tx, pixel_zoom_ty, 0); // Translate GL coords to raster pos
    double s = a_gx_scale() * pixel_zoom;
    glScalef(s,s,s);

    int display_list = gx_list_base + frame;
    if (glIsList(display_list)) {
      glCallList(display_list);
    } else {
      glNewList(display_list, GL_COMPILE_AND_EXECUTE);
      example_GXFileVisitor_OpenGL gx;
      gx.set_color(0,1,0);
      gx.visit(gx_basename->name(frame).c_str());
      glEndList();
    }
    glPopMatrix();
  }

  glFlush();
  // Draw text over image
  glScissor(0,window_height - TEXTHEIGHT, window_width, window_height);
  glColor3f(.5,0,1);
  {
    // Build msg
    char msg[1024];
    float fps = 1000.0 / tic.real();

    vul_sprintf(msg, "Frame[%d] FPS %4.1f ", moviefile->GetRealFrameIndex(frame), fps);
    vul_sprintf(msg + vcl_strlen(msg), "Zoom %g ", pixel_zoom);
    vul_sprintf(msg + vcl_strlen(msg), "(%7.2f, %7.2f)",
                (mouse_x - pixel_zoom_tx) / pixel_zoom,
                (mouse_y - pixel_zoom_ty) / pixel_zoom);

    output(2,20,msg);
  }

  glutSwapBuffers();

  return true;
}

void reshape(int w, int h)
{
  if (window_width > 0) {
    double scale = vcl_min(double(w) / window_width, double(h) / window_height);
    pixel_zoom_tx = pixel_zoom_tx * scale;
    pixel_zoom_ty = pixel_zoom_ty * scale;
    pixel_zoom = pixel_zoom * scale;
  }

  window_width = w;
  window_height = h;

  /* Reshape both layers. */
  for (int i = 0; i < nlayers; ++i) {
    glutUseLayer(GLenum(layers[i]));
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);  /* Start modifying the projection matrix. */
    glLoadIdentity();             /* Reset project matrix. */
    glOrtho(0, w, 0, h, -1, 1);   /* Map abstract coords directly to window coords. */
    glScalef(1, -1, 1);           /* Invert Y axis so increasing Y goes down. */
    glTranslatef(0, -h, 0);       /* Shift origin up to upper-left corner. */
  }
}

void display()
{
  bool ok = drawframe(frame);

  if (!ok) {
    vul_printf(vcl_cerr, "Ran out of frames at %d\n", frame);
    num_frames = frame;
    frame = a_start_frame();
    if (!drawframe(frame))
      vcl_abort();
  }
}

void idle()
{
  // Update frame counter
  frame += dir;
  if (dir > 0) {
    if (frame >= num_frames) frame = 0;
  } else {
    if (frame < 0) frame = num_frames - 1;
  }

  glutPostRedisplay();
}

void setidle(bool on) {
  if (on)
    glutIdleFunc(idle);
  else
    glutIdleFunc(0);
}

/////////////////////////////////////////////////////////////////////////////

struct example_CB {
  virtual ~example_CB () {}

  //: Return true if this event is interesting
  virtual bool want(int button, int modifiers, int x, int y) { return false; }
  virtual void down(int, int) { }
  virtual void motion(int, int) {}
  virtual void up(int, int) {}
};

vcl_vector<example_CB*> down_cbs;
example_CB* down_cb = 0;

#include <vcl_vector.txx>
VCL_VECTOR_INSTANTIATE(example_CB*);

int down_x, down_y;
int down_button;
bool down_modifiers;

//:
// Run through down_cbs until one that likes this mouse event is found,
// Then call it...
void mouse_callback(int button, int state, int x, int y)
{
  if (state == GLUT_DOWN) {
    down_x = x;
    down_y = y;
    down_button = button;
    down_modifiers = glutGetModifiers() != 0;

    int num_want = 0;
    for (vcl_vector<example_CB*>::const_iterator p = down_cbs.begin(); p != down_cbs.end(); ++p)
      if ((*p)->want(button, down_modifiers, x, y))
        ++num_want;
    if (num_want > 1)
      vul_printf(vcl_cerr, "WARNING: Everybody (well %d) wants a button %d, mods %x\n", num_want, button, down_modifiers);

    for (vcl_vector<example_CB*>::const_iterator p = down_cbs.begin(); p != down_cbs.end(); ++p)
      if ((*p)->want(button, down_modifiers, x, y)) {
        down_cb = *p;
        down_cb->down(x, y);
        break;
      }
  } else if (state == GLUT_UP) {
    if (down_cb)
      down_cb->up(x, y);
    down_cb = 0;
  } else
    vcl_abort();
}

void motion_callback(int x, int y)
{
  mouse_x = x;
  mouse_y = y;
  if (down_cb)
    down_cb->motion(x,y);
}

// -----------------------------------------------------------------------------
// Shuttle event handler

struct example_ShuttleCB : public example_CB {
  int down_frame;

  bool want(int button, int modifiers, int, int) {
    // Shuttle takes an unmodified left mouse button
    return modifiers == 0 && button == GLUT_LEFT_BUTTON;
  }

  void down(int, int) {
    down_frame = frame;

    setidle(false);
  }
  void up(int, int) {
    if (playing)
      setidle(true);
  }
  void motion(int x, int y) {
    double scale_factor = vcl_min(100, num_frames) / 800.0;
    frame = down_frame + int((x - down_x) * scale_factor);

    if (frame < 0) {
      frame = 0;
      down_x = x;
      down_frame = frame;
    }
    if (frame >= num_frames) {
      frame = num_frames-1;
      down_x = x;
      down_frame = frame;
    }
    glutPostRedisplay();
  }
};

// -----------------------------------------------------------------------------
// Zoom event handler
struct example_ZoomCB : public example_CB {
  double down_pixel_zoom;
  double down_pixel_zoom_tx;
  double down_pixel_zoom_ty;

  bool want(int button, int modifiers, int, int) {
    // Take any shifted mouse event
    return modifiers & GLUT_ACTIVE_SHIFT;
  }

  void down(int x, int y) {
    down_pixel_zoom = pixel_zoom;
    down_pixel_zoom_tx = pixel_zoom_tx;
    down_pixel_zoom_ty = pixel_zoom_ty;

    glNewList(ovl_display_list, GL_COMPILE);
    glIndexi(ovl_opaque);
    glBegin(GL_LINES);
    glVertex2f(x,y);
    glVertex2f(x,y);
    glEnd();
    glEndList();

    if (have_overlay)
      glutShowOverlay();
    glutSetCursor(GLUT_CURSOR_CROSSHAIR);
  }
  void motion(int x, int y) {
    if (down_button == GLUT_LEFT_BUTTON) {
      int width = 100;
      double factor = double(y - down_y) / width;
      factor = vcl_exp(factor);
      if (factor < 1e-2) factor = 1e-2;
      if (factor > 1e+2) factor = 1e+2;

      double down_img_x = (down_x - down_pixel_zoom_tx) / down_pixel_zoom;
      double down_img_y = (down_y - down_pixel_zoom_ty) / down_pixel_zoom;

      pixel_zoom = down_pixel_zoom * factor;
      pixel_zoom_tx = down_x - pixel_zoom * down_img_x;
      pixel_zoom_ty = down_y - pixel_zoom * down_img_y;
    }

    if (down_button == GLUT_MIDDLE_BUTTON) {
      pixel_zoom_tx = down_pixel_zoom_tx + (x - down_x);
      pixel_zoom_ty = down_pixel_zoom_ty + (y - down_y);
    }

    glutPostRedisplay();

    if (have_overlay) {
    // Swishy graphics
    int x0 = down_x;
    int y0 = down_y;
    glNewList(ovl_display_list, GL_COMPILE);
    glIndexi(ovl_opaque);
    glBegin(GL_LINES);
    glVertex2f(x0,y0);
    glVertex2f(x,y);
    glEnd();
    glBegin(GL_LINE_LOOP);
    double boxw = pixel_zoom / 2;
    glVertex2f(x0-boxw,y0-boxw);
    glVertex2f(x0+boxw,y0-boxw);
    glVertex2f(x0+boxw,y0+boxw);
    glVertex2f(x0-boxw,y0+boxw);
    glEnd();
    glEndList();

    glutPostOverlayRedisplay();
  }
  }
  void up(int, int) {
    glutSetCursor(GLUT_CURSOR_INHERIT);
    if (have_overlay) glutHideOverlay();
  }
};

// -----------------------------------------------------------------------------
// Draw cb
struct example_DrawCB : public example_CB {
  bool want(int button, int modifiers, int x, int y) {
    return modifiers == 0 && button == GLUT_MIDDLE_BUTTON;
  }

  enum {
    line,
    line2way,
    infline,
    circle,
    box,
    conic
  } mode;
  example_DrawCB() {
    mode = infline;
  }

  void down(int x, int y) {
    glutUseLayer(GLenum(GLUT_OVERLAY));
    glNewList(ovl_display_list, GL_COMPILE_AND_EXECUTE);
    glClear(GL_COLOR_BUFFER_BIT);
    glEndList();
    glutPostOverlayRedisplay();
    glutShowOverlay();
  }

  void motion(int x, int y) {
    glNewList(ovl_display_list, GL_COMPILE);
    glIndexi(ovl_opaque);
    switch (mode) {
    case line:
      glBegin(GL_LINES);
      glVertex2f(down_x, down_y);
      glVertex2f(x, y);
      glEnd();
      break;
    case line2way:
      glBegin(GL_LINES);
      glVertex2f(x, y);
      glVertex2f(2 * down_x - x, 2 * down_y - y);
      glEnd();
      break;
    case infline: {
      double dx = x - down_x;
      double dy = y - down_y;
      double dirx = -dy * 100;
      double diry = dx * 100;
      glBegin(GL_LINES);
      glVertex2f(x + dirx, y + diry);
      glVertex2f(x - dirx, y - diry);
      glEnd();
      break;
    }
    case conic: {
#if 0
      double dx = x - down_x;
      double dy = y - down_y;
      gl_draw_conic(down_x, down_y, dx, dy, 0);
#endif
      break;
    }
    case box:
      glBegin(GL_LINE_LOOP);
      glVertex2f(down_x, down_y);
      glVertex2f(x, down_y);
      glVertex2f(x, y);
      glVertex2f(down_x, y);
      glEnd();
      break;
    case circle: {
      double dx = x - down_x;
      double dy = y - down_y;
      double r= vcl_sqrt(dx*dx + dy*dy);
      int n = 20;
      glBegin(GL_LINE_LOOP);
      double dt = vnl_math::pi / n * 2;
      for (int i = 0; i < n; ++i) {
        double ox = r * vcl_cos(i * dt);
        double oy = r * vcl_sin(i * dt);
        glVertex2f(down_x + ox, down_y + oy);
      }
      glEnd();
      break;
    }
    default:
      break;
    }

    glEndList();

    glutPostOverlayRedisplay();
  }

  void up(int x, int y) {

    double s = 1.0 / pixel_zoom;
    double tx = -pixel_zoom_tx / pixel_zoom;
    double ty = -pixel_zoom_ty / pixel_zoom;
    vul_printf(vcl_cerr, "v = [%d %g %g %g %g];\n",
           moviefile->GetRealFrameIndex(frame),
           s*down_x+tx, s*down_y+ty,
           s*x+tx, s*y+ty);
    glutHideOverlay();
  }
};

void keyboard(unsigned char key, int x, int y)
{
  switch (key) {
  case ' ': {
    playing = !playing;
    setidle(playing);
    break;
  }
  case 'r': {
    glDeleteLists(0, num_frames);
    break;
  }
  case 'q': {
    vcl_exit(0);
  }
  }
  glutPostRedisplay();
}

void visible(int vis)
{
  if (playing)
    setidle(vis == GLUT_VISIBLE);
}

int main(int argc, char ** argv)
{
  // ios::sync_with_stdio(false);
  vul_arg<char*> filename(0, "Input file");
  vul_arg<int>   a_step("-d", "frame step", 1);
  vul_arg<int>   a_end_frame("-e", "End frame", 10000);
  vul_arg<char*> a_gx_file("-g", "GX file");
  vul_arg_parse(argc,argv);

  if (argc > 1)
    vul_arg_display_usage_and_exit("Too many arguments\n");

  // Register callbacks
  example_ShuttleCB shuttle_cb;
  down_cbs.push_back(&shuttle_cb);

  example_ZoomCB zoom_cb;
  down_cbs.push_back(&zoom_cb);

  example_DrawCB draw_cb;
 // wait till wee know about overlays down_cbs.push_back(&draw_cb);

  // Copy args to globals
  int step = a_step();
  num_frames = (a_end_frame() - a_start_frame()) / step;
  if (num_frames > 0) {
    dir = 1;
    num_frames = +num_frames + 1;
    frame = 0;
  } else {
    dir = -1;
    num_frames = -num_frames + 1;
    frame = num_frames - 1;
  }
  dir = dir*step;

  moviefile = new MovieFile(filename(), a_start_frame(), dir, a_end_frame());
  if (moviefile->GetSizeX() < 1) {
     vcl_cerr << "glmovie: Couldn't find any movie files. Stopping\n";
     return -1;
  }

  if (a_gx_file()) {
    gx_basename = new ImageSequenceName(a_gx_file(), a_start_frame(), dir, "r", ".gx");
    vcl_cerr << "glmovie: Getting gx from " << *gx_basename << '\n';
  } else {
    gx_basename = 0;
  }

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
  // glutInitDisplayString("rgba double");
  glutCreateWindow("GL Movie Player");
  vcl_cerr << " movie size " << moviefile->GetSizeX() << " x " << moviefile->GetSizeY() << vcl_endl;
  glutReshapeWindow(moviefile->GetSizeX(), moviefile->GetSizeY() + TEXTHEIGHT);
  init();
   if (glutLayerGet(GLenum(GLUT_OVERLAY_POSSIBLE)) == 0)  {
      vul_printf(vcl_cout, "glmovie: no overlays supported; ok.\n");
      nlayers = 1;
      have_overlay = false;
      //vcl_exit(1);
   } else {
     have_overlay = true;
     down_cbs.push_back(&draw_cb);
   }

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse_callback);
  glutMotionFunc(motion_callback);
  glutVisibilityFunc(visible);

  // Now set up overlay
  {
    glutInitDisplayMode(GLUT_INDEX | GLUT_SINGLE);
    if (have_overlay) {
      vul_printf(vcl_cerr, "glmovie: Got overlay.\n");
      glutEstablishOverlay();
      glutHideOverlay();
      glutOverlayDisplayFunc(overlayDisplay);

      /* Find transparent and opaque index. */
      ovl_transparent = glutLayerGet(GLenum(GLUT_TRANSPARENT_INDEX));
      ovl_opaque = (ovl_transparent + 1) % glutGet(GLenum(GLUT_WINDOW_COLORMAP_SIZE));

      glutSetColor(ovl_opaque, 1.0, 0.0, 0.0);

      /* Make sure overlay clears to transparent. */
      glClearIndex(ovl_transparent);
    }
    glDisable(GL_POINT_SMOOTH);
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_BLEND);
    glLineWidth(0.1);
  }

  glutMainLoop();
  return 0;
}
