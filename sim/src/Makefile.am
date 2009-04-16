lib_LTLIBRARIES = libhwm_net.la

libhwm_net_la_SOURCES = intersection.cpp \
	                network.cpp \
	 	        lane.cpp \
                        road.cpp \
                        timer.cpp

libhwm_net_la_CPPFLAGS = -I$(srcdir)/../include $(BOOST_CPPFLAGS) $(LIBXML2_CFLAGS)
libhwm_net_la_LDFLAGS  = -static