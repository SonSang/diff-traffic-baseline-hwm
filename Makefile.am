AM_CXXFLAGS = $(WARN_FLAGS) $(OPTIMIZE_FLAGS) $(ARCH_FLAGS) $(DEBUG_FLAGS)

bin_PROGRAMS = simtest lane-test interval-test

if DO_VISUAL
bin_PROGRAMS += showtest showsim

showtest_SOURCES = \
	showlane.cpp \
	plotwin.hpp \
	plot.cpp \
	plot.hpp

showtest_CXXFLAGS = $(CAIRO_CFLAGS) $(FLTK_CXXFLAGS) $(AM_CXXFLAGS)
showtest_LDFLAGS = $(FLTK_LDFLAGS) $(CAIRO_LIBS)

showsim_SOURCES = \
	showsim.cpp \
	plotwin.hpp \
	plot.cpp \
	plot.hpp \
	arz.hpp

showsim_CXXFLAGS = $(CAIRO_CFLAGS) $(FLTK_CXXFLAGS) $(AM_CXXFLAGS)
showsim_LDFLAGS = $(FLTK_LDFLAGS) $(CAIRO_LIBS)
endif

if DO_XML
bin_PROGRAMS += xml-test

xml_test_SOURCES = \
	xml-test.cpp

xml_test_CXXFLAGS = $(LIBXML2_CFLAGS) $(AM_CXXFLAGS)
xml_test_LDFLAGS = $(LIBXML2_LIBS)
endif

simtest_SOURCES = \
	test_solve.cpp \
	arz.hpp

simtest_CXXFLAGS = $(TS_CXXFLAGS) $(AM_CXXFLAGS)
simtest_LDADD = $(TS_LDFLAGS)

lane_test_SOURCES = \
	lane-test.cpp \
	road.cpp

lane_test_CXXFLAGS = $(AM_CXXFLAGS)

interval_test_SOURCES = \
	interval-test.cpp

interval_test_CXXFLAGS = $(AM_CXXFLAGS)
