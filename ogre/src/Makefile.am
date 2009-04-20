EXTRA_DIST = hwm-viewer.cpp

if DO_OGRE
noinst_PROGRAMS = hwm-viewer

hwm_viewer_SOURCES = hwm-viewer.cpp hwm-viewer.hpp

hwm_viewer_CPPFLAGS = $(CAELUM_CFLAGS) $(OGRE_CFLAGS) $(OIS_CFLAGS) $(BOOST_CPPFLAGS) $(LIBXML2_CFLAGS) -Wno-deprecated -I@top_srcdir@/ogre/include -I@top_srcdir@/sim/include
hwm_viewer_LDADD = $(CAELUM_LIBS) $(OGRE_LIBS) $(OIS_LIBS) $(BOOST_LDFLAGS) $(LIBXML2_LIBS) @top_builddir@/sim/src/libhwm_net.la
endif