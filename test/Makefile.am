noinst_PROGRAMS = macro

EXTRA_DIST = arcball.hpp

macro_SOURCES  = macro.cpp
macro_CPPFLAGS = $(LIBROAD_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
macro_LDFLAGS  = $(LDFLAGS)
macro_LDADD    = $(top_builddir)/libarz/libarz.la

if DO_VISUAL
noinst_PROGRAMS += macro-viewer

macro_viewer_SOURCES = macro-viewer.cpp
macro_viewer_CPPFLAGS = $(LIBROAD_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir)
macro_viewer_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS)
macro_viewer_LDADD    = $(top_builddir)/libarz/libarz.la $(LIBROAD_LIBS)/libroad_visual.la
endif
