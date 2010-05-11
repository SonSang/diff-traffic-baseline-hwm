noinst_PROGRAMS = hybrid ih-riemann-test pc-int-test

EXTRA_DIST = arcball.hpp big-image-tile.hpp

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

if DO_IMAGE
noinst_PROGRAMS += cairo-traffic

cairo_traffic_SOURCES  = cairo-traffic.cpp
cairo_traffic_CPPFLAGS = $(LIBROAD_CFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CAIRO_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(MAGICKPP_CXXFLAGS) $(CAIRO_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
cairo_traffic_LDFLAGS  = $(BOOST_FILESYSTEM_LIBS) $(CAIRO_LIBS) $(MAGICKPP_LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(LDFLAGS)
cairo_traffic_LDADD    = $(BOOST_THREAD_LIBS) $(BOOST_REGEX_LIBS) $(LIBROAD_LIBS)/libroad.la $(LIBROAD_LIBS)/libroad_visual.la $(LIBROAD_LIBS)/libroad_image.la $(top_builddir)/libhybrid/libhybrid.la $(LIBROAD_LIBS)/libroad_image.la
endif
endif
