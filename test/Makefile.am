noinst_PROGRAMS = hybrid # ih-riemann-test pc-int-test dump-to-png image-average

EXTRA_DIST = arcball.hpp big-image-tile.hpp night-render.hpp gl-common.hpp car-animation.hpp

hybrid_SOURCES  = hybrid.cpp
hybrid_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(OPENMP_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir)
hybrid_LDFLAGS  = $(LDFLAGS) $(OPENMP_CXXFLAGS) $(BOOST_FILESYSTEM_LIBS) -lboost_system-mt
hybrid_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

# ih_riemann_test_SOURCES  = ih-riemann-test.cpp
# ih_riemann_test_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(OPENMP_CXXFLAGS)  $(CXXFLAGS) -I$(top_srcdir)
# ih_riemann_test_LDFLAGS  = $(LDFLAGS) $(OPENMP_CXXFLAGS)
# ih_riemann_test_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

# pc_int_test_SOURCES  = pc-int-test.cpp
# pc_int_test_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(OPENMP_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir)
# pc_int_test_LDFLAGS  = $(LDFLAGS) $(OPENMP_CXXFLAGS)
# pc_int_test_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

# dump_to_png_SOURCES  = dump-to-png.cpp
# dump_to_png_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(OPENMP_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir)
# dump_to_png_LDFLAGS  = $(LDFLAGS)
# dump_to_png_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

# image_average_SOURCES  = image-average.cpp
# image_average_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(OPENMP_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir)
# image_average_LDFLAGS  = $(LDFLAGS)
# image_average_LDADD    = $(top_builddir)/libhybrid/libhybrid.la

if DO_VISUAL
if DO_IMAGE
# noinst_PROGRAMS += hybrid-viewer

# hybrid_viewer_SOURCES = hybrid-viewer.cpp
# hybrid_viewer_CPPFLAGS = $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(OPENMP_CXXFLAGS) $(FLTK_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir)
# hybrid_viewer_LDFLAGS  = $(LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(OPENMP_CXXFLAGS)
# hybrid_viewer_LDADD    = $(LIBROAD_LIBS)/libroad.la $(LIBROAD_LIBS)/libroad_visual.la $(LIBROAD_LIBS)/libroad_image.la $(top_builddir)/libhybrid/libhybrid.la $(LIBROAD_LIBS)/libroad_image.la

noinst_PROGRAMS += traffic-2d

traffic_2d_SOURCES  = traffic-2d.cpp car-animation.cpp
traffic_2d_CPPFLAGS = $(LIBROAD_CFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CAIRO_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(MAGICKPP_CXXFLAGS) $(CAIRO_CFLAGS) $(OPENMP_CXXFLAGS) $(CXXFLAGS) -I$(top_srcdir)
traffic_2d_LDFLAGS  = $(BOOST_FILESYSTEM_LIBS) $(CAIRO_LIBS) $(MAGICKPP_LDFLAGS) $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(LDFLAGS) $(OPENMP_CXXFLAGS)
traffic_2d_LDADD    = $(BOOST_FILESYSTEM_LIBS) -lboost_system-mt -lboost_filesystem-mt $(BOOST_THREAD_LIBS) $(BOOST_REGEX_LIBS) $(top_builddir)/libhybrid/libhybrid.la $(BOOST_FILESYSTEM_LIBS) -lroad
endif
endif
