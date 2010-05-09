lib_LTLIBRARIES = libhybrid.la

libhybrid_la_SOURCES  = hybrid-sim.cpp \
	                hybrid-macro.cpp \
			hybrid-micro.cpp \
		        hybrid-draw.cpp \
			timer.cpp \
	                libhybrid-common.cpp

pkginclude_HEADERS  = arz.hpp \
		      arz-eq.hpp \
		      arz-impl.hpp \
	              hybrid-sim.hpp \
		      pc-integrate.hpp \
		      pc-poisson.hpp \
		      timer.hpp \
		      libhybrid-common.hpp

libhybrid_la_CPPFLAGS =  $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS)

libhybrid_la_LDFLAGS  = -static $(LDFLAGS)
libhybrid_la_LIBADD   =  $(LIBROAD_LIBS)/libroad.la
