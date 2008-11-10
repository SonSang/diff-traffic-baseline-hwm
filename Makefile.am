AM_CXXFLAGS = $(WARN_FLAGS) $(OPTIMIZE_FLAGS) $(ARCH_FLAGS) $(DEBUG_FLAGS)

bin_PROGRAMS = simtest

if DO_VISUAL
bin_PROGRAMS += showtest

showtest_SOURCES = \
	showlane.cpp \
	plot.cpp \
	plot.hpp

showtest_CXXFLAGS = $(CAIRO_CFLAGS) $(FLTK_CXXFLAGS) $(AM_CXXFLAGS)
showtest_LDFLAGS = $(FLTK_LDFLAGS) $(CAIRO_LIBS)
endif

simtest_SOURCES = \
	test_solve.cpp \
	awz.hpp

simtest_CXXFLAGS = $(TS_CXXFLAGS) $(AM_CXXFLAGS)
simtest_LDADD = $(TS_LDFLAGS)
