AM_CXXFLAGS = -I$(top_srcdir)/sim/include $(BOOST_CPPFLAGS) $(LIBXML2_CFLAGS)
ADD_LIBS = $(top_builddir)/sim/src/libhwm_net.la $(BOOST_LDFLAGS) $(LIBXML2_LIBS)

EXTRA_DIST = showsim.cpp showlane.cpp shownetwork.cpp plot.cpp plot.hpp plotwin.hpp xml-test.cpp car-data-test.cpp

noinst_PROGRAMS = interval-test inhomo-test lane-test xml-test car-data-test

interval_test_SOURCES = interval-test.cpp
interval_test_LDADD   = $(ADD_LIBS)

inhomo_test_SOURCES   = inhomo-test.cpp
inhomo_test_LDADD     = $(ADD_LIBS)

lane_test_SOURCES     = lane-test.cpp
lane_test_LDADD       = $(ADD_LIBS)

xml_test_SOURCES      = xml-test.cpp
xml_test_LDADD        = $(ADD_LIBS)

car_data_test_SOURCES = car-data-test.cpp
car_data_test_LDADD   = $(ADD_LIBS)

if DO_VISUAL
noinst_PROGRAMS += showlane shownetwork showsim

showlane_SOURCES = showlane.cpp \
		   plot.cpp     \
		   plot.hpp     \
		   plotwin.hpp
showlane_CXXFLAGS = $(AM_CXXFLAGS) $(CAIRO_CFLAGS) $(FLTK_CXXFLAGS)
showlane_LDADD    = $(ADD_LIBS) $(CAIRO_LIBS) $(FLTK_LDFLAGS)

shownetwork_SOURCES  = shownetwork.cpp arcball.hpp
shownetwork_CXXFLAGS = $(AM_CXXFLAGS) $(FLTK_GL_CXXFLAGS)
shownetwork_LDADD    = $(ADD_LIBS) $(FLTK_GL_LDFLAGS)

showsim_SOURCES = showsim.cpp \
		  plot.cpp    \
		  plot.hpp    \
		  plotwin.hpp

showsim_CXXFLAGS = $(AM_CXXFLAGS) $(CAIRO_CFLAGS) $(FLTK_CXXFLAGS)
showsim_LDADD    = $(ADD_LIBS) $(CAIRO_LIBS) $(FLTK_LDFLAGS)
endif
