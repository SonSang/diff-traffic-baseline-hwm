lib_LTLIBRARIES = libarz.la

libarz_la_SOURCES  = macro-sim.cpp

pkginclude_HEADERS  = arz.hpp \
		      arz-eq.hpp \
		      arz-impl.hpp \
		      macro-sim.hpp

libarz_la_CPPFLAGS =  $(LIBROAD_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS)

libarz_la_LDFLAGS  = -static $(LDFLAGS)
libarz_la_LIBADD   =  $(LIBROAD_LIBS)/libroad.la
