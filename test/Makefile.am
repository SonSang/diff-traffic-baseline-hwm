noinst_PROGRAMS = hybrid

EXTRA_DIST = arcball.hpp

hybrid_SOURCES  = hybrid.cpp
hybrid_CPPFLAGS = $(LIBROAD_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
hybrid_LDFLAGS  = $(LDFLAGS)
hybrid_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

if DO_VISUAL
noinst_PROGRAMS += hybrid-viewer

hybrid_viewer_SOURCES = hybrid-viewer.cpp
hybrid_viewer_CPPFLAGS = $(LIBROAD_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir)
hybrid_viewer_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS)
hybrid_viewer_LDADD    = $(top_builddir)/libhybrid/libhybrid.la $(LIBROAD_LIBS)/libroad_visual.la
endif
