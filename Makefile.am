ACLOCAL_AMFLAGS = -I m4
AM_CXXFLAGS = $(WARN_FLAGS) $(OPTIMIZE_FLAGS) $(DEBUG_FLAGS) $(ARCH_FLAGS) $(DEBUG_FLAGS) $(LIBXML2_CFLAGS) $(BOOST_CPPFLAGS)
AM_LDFLAGS = $(LIBXML2_LIBS)

bin_PROGRAMS = simtest lane-test interval-test xml-test
lib_LIBRARIES = libhwm.a

libhwm_a_SOURCES = \
	timer.cpp \
	network.cpp \
	road.cpp \
	lane.cpp \
	intersection.cpp \
	timer.hpp \
	network.hpp \
	road.hpp \
	lane.hpp \
	intersection.hpp \
	intervals.hpp

if DO_VISUAL
bin_PROGRAMS += showtest showsim shownetwork

showtest_SOURCES = \
	showlane.cpp \
	plotwin.hpp \
	plot.cpp \
	plot.hpp

showtest_CXXFLAGS = $(CAIRO_CFLAGS) $(FLTK_CXXFLAGS) $(AM_CXXFLAGS)
showtest_LDFLAGS = $(FLTK_LDFLAGS) $(CAIRO_LIBS) $(AM_LDFLAGS)

showsim_SOURCES = \
	showsim.cpp \
	plotwin.hpp \
	plot.cpp \
	plot.hpp \
	arz.hpp

showsim_CXXFLAGS = $(CAIRO_CFLAGS) $(FLTK_CXXFLAGS) $(AM_CXXFLAGS)
showsim_LDFLAGS = $(FLTK_LDFLAGS) $(CAIRO_LIBS) $(AM_LDFLAGS)

shownetwork_SOURCES = \
	shownetwork.cpp \
	timer.hpp \
	network.hpp \
	road.hpp \
	lane.hpp \
	intersection.hpp \
	intervals.hpp

shownetwork_CXXFLAGS = $(FLTK_GL_CXXFLAGS) $(AM_CXXFLAGS)
shownetwork_LDFLAGS = $(FLTK_GL_LDFLAGS) $(AM_LDFLAGS)
shownetwork_LDADD = libhwm.a
endif

xml_test_SOURCES = \
	xml-test.cpp \
	timer.hpp \
	network.hpp \
	road.hpp \
	lane.hpp \
	intersection.hpp \
	intervals.hpp

xml_test_CXXFLAGS = $(AM_CXXFLAGS)
xml_test_LDFLAGS = $(AM_LDFLAGS)
xml_test_LDADD = libhwm.a

simtest_SOURCES = \
	test_solve.cpp \
	arz.hpp

simtest_CXXFLAGS = $(TS_CXXFLAGS) $(AM_CXXFLAGS)
simtest_LDADD = $(TS_LDFLAGS) $(AM_LDFLAGS)

lane_test_SOURCES = \
	lane-test.cpp \
	road.cpp

lane_test_CXXFLAGS = $(AM_CXXFLAGS)
lane_test_LDFLAGS = $(AM_LDFLAGS)

interval_test_SOURCES = \
	interval-test.cpp

interval_test_CXXFLAGS = $(AM_CXXFLAGS)
interval_test_LDFLAGS = $(AM_LDFLAGS)
