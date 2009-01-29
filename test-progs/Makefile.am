AM_CXXFLAGS = -I$(top_srcdir)/sim/include $(BOOST_CPPFLAGS) $(LIBXML2_CFLAGS)
ADD_LIBS = $(top_builddir)/sim/src/libhwm_net.a $(BOOST_LDFLAGS) $(LIBXML2_LIBS)

noinst_PROGRAMS = interval-test inhomo-test lane-test xml-test

interval_test_SOURCES = interval-test.cpp
interval_test_LDADD   = $(ADD_LIBS)

inhomo_test_SOURCES   = inhomo-test.cpp
inhomo_test_LDADD     = $(ADD_LIBS)

lane_test_SOURCES     = lane-test.cpp
lane_test_LDADD       = $(ADD_LIBS)

xml_test_SOURCES      = xml-test.cpp
xml_test_LDADD        = $(ADD_LIBS)
