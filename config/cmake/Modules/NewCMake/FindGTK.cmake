#
# try to find GTK (and glib) and GTKGLArea
#

# GTK_INCLUDE_DIR   - Directories to include to use GTK
# GTK_LIBRARIES     - Files to link against to use GTK
# GTK_FOUND         - If false, don't try to use GTK

# don't even bother under WIN32
IF (UNIX)

  FIND_PATH( GTK_gtk_INCLUDE_DIR gtk/gtk.h
    /usr/include
    /usr/local/include
    /usr/openwin/share/include
    /usr/openwin/include
    /usr/X11R6/include
    /usr/include/X11
    /usr/X11R6/include/gtk12
    /usr/include/gtk-1.2
    /usr/local/include/gtk-1.2
    /opt/gnome/include
    ${GTK_gtk_INCLUDE_PATH}
  )

  # Some Linux distributions (e.g. Red Hat) have glibconfig.h
  # and glib.h in different directories, so we need to look
  # for both.
  #  - Atanas Georgiev <atanas@cs.columbia.edu>

  FIND_PATH( GTK_glibconfig_INCLUDE_DIR glibconfig.h
    /usr/include
    /usr/local/include
    /usr/openwin/share/include
    /usr/local/include/glib12
    /usr/lib/glib/include
    /usr/local/lib/glib/include
    /opt/gnome/include
    /opt/gnome/lib/glib/include
    ${GTK_glibconfig_INCLUDE_PATH}
  )

  FIND_PATH( GTK_glib_INCLUDE_DIR glib.h
    /usr/include
    /usr/local/include
    /usr/openwin/share/include
    /usr/include/gtk-1.2
    /usr/local/include/glib12
    /usr/lib/glib/include
    /usr/include/glib-1.2
    /usr/local/include/glib-1.2
    /opt/gnome/include
    /opt/gnome/include/glib-1.2
    ${GTK_glib_INCLUDE_PATH}
  )

  FIND_PATH( GTK_gtkgl_INCLUDE_DIR gtkgl/gtkglarea.h
    /usr/include
    /usr/local/include
    /usr/openwin/share/include
    /opt/gnome/include
    ${GTK_gtkgl_INCLUDE_PATH}
  )

  FIND_LIBRARY( GTK_gtkgl_LIBRARY gtkgl
    /usr/lib
    /usr/local/lib
    /usr/openwin/lib
    /usr/X11R6/lib
    /opt/gnome/lib
  )

  #
  # The 12 suffix is thanks to the FreeBSD ports collection
  #

  FIND_LIBRARY( GTK_gtk_LIBRARY
    NAMES  gtk gtk12
    PATHS /usr/lib
          /usr/local/lib
          /usr/openwin/lib
          /usr/X11R6/lib
          /opt/gnome/lib
  )

  FIND_LIBRARY( GTK_gdk_LIBRARY
    NAMES  gdk gdk12
    PATHS  /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
  )

  FIND_LIBRARY( GTK_gmodule_LIBRARY
    NAMES  gmodule gmodule12
    PATHS  /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
  )

  FIND_LIBRARY( GTK_glib_LIBRARY
    NAMES  glib glib12
    PATHS  /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
  )

  IF(GTK_gtk_INCLUDE_DIR)
  IF(GTK_glibconfig_INCLUDE_DIR)
  IF(GTK_glib_INCLUDE_DIR)
  IF(GTK_gtkgl_INCLUDE_DIR)
  IF(GTK_gtk_LIBRARY)
  IF(GTK_glib_LIBRARY)
  IF(GTK_gtkgl_LIBRARY)

    # Assume that if gtk and glib were found, the other
    # supporting libraries have also been found.

    SET( GTK_FOUND "YES" )
    SET( GTK_INCLUDE_DIR  ${GTK_gtk_INCLUDE_DIR}
                           ${GTK_glibconfig_INCLUDE_DIR}
                           ${GTK_glib_INCLUDE_DIR}
                           ${GTK_gtkgl_INCLUDE_DIR} )
    SET( GTK_LIBRARIES  ${GTK_gtkgl_LIBRARY}
                        ${GTK_gtk_LIBRARY}
                        ${GTK_gdk_LIBRARY}
                        ${GTK_gmodule_LIBRARY}
                        ${GTK_glib_LIBRARY}    )

  ENDIF(GTK_gtkgl_LIBRARY)
  ENDIF(GTK_glib_LIBRARY)
  ENDIF(GTK_gtk_LIBRARY)
  ENDIF(GTK_gtkgl_INCLUDE_DIR)
  ENDIF(GTK_glib_INCLUDE_DIR)
  ENDIF(GTK_glibconfig_INCLUDE_DIR)
  ENDIF(GTK_gtk_INCLUDE_DIR)

  MARK_AS_ADVANCED(
    GTK_gtkgl_LIBRARY
    GTK_glib_LIBRARY
    GTK_gtk_LIBRARY
    GTK_gtkgl_INCLUDE_DIR
    GTK_glibconfig_INCLUDE_DIR
    GTK_glib_INCLUDE_DIR
    GTK_gtk_INCLUDE_DIR
  )

ENDIF (UNIX)

