AM_CXXFLAGS = $(WARN_FLAGS) $(OPTIMIZE_FLAGS) $(ARCH_FLAGS) $(DEBUG_FLAGS)

bin_PROGRAMS = simtest

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
	awz.hpp

showsim_CXXFLAGS = $(CAIRO_CFLAGS) $(FLTK_CXXFLAGS) $(AM_CXXFLAGS)
showsim_LDFLAGS = $(FLTK_LDFLAGS) $(CAIRO_LIBS)
endif

simtest_SOURCES = \
	test_solve.cpp \
	awz.hpp

simtest_CXXFLAGS = $(TS_CXXFLAGS) $(AM_CXXFLAGS)
simtest_LDADD = $(TS_LDFLAGS)
