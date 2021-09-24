lib_LTLIBRARIES = libhybrid.la

libhybrid_la_SOURCES  = hybrid-sim.cpp \
			hybrid-sim-omp.cpp \
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
		      libhybrid-common.hpp \
                      allocate.hpp

libhybrid_la_CPPFLAGS =  $(LIBROAD_CFLAGS) $(CAIRO_CFLAGS) $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CXXFLAGS) $(OPENMP_CXXFLAGS) -I$(top_srcdir)

libhybrid_la_LDFLAGS  = -static $(LDFLAGS) $(OPENMP_CXXFLAGS)
libhybrid_la_LIBADD   =  $(OPENMP_CXXFLAGS)
