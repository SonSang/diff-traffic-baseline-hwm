noinst_PROGRAMS = hybrid ih-riemann-test pc-int-test

EXTRA_DIST = arcball.hpp

hybrid_SOURCES  = hybrid.cpp
hybrid_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
hybrid_LDFLAGS  = $(LDFLAGS)
hybrid_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

ih_riemann_test_SOURCES  = ih-riemann-test.cpp
ih_riemann_test_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
ih_riemann_test_LDFLAGS  = $(LDFLAGS)
ih_riemann_test_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

pc_int_test_SOURCES  = pc-int-test.cpp
pc_int_test_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
pc_int_test_LDFLAGS  = $(LDFLAGS)
pc_int_test_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

if DO_VISUAL
noinst_PROGRAMS += hybrid-viewer

hybrid_viewer_SOURCES = hybrid-viewer.cpp
hybrid_viewer_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir)
hybrid_viewer_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS)
hybrid_viewer_LDADD    = $(top_builddir)/libhybrid/libhybrid.la $(LIBROAD_LIBS)/libroad_visual.la
endif
