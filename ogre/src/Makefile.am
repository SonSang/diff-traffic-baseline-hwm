EXTRA_DIST = hwm-viewer.cpp

if DO_OGRE
noinst_PROGRAMS = hwm-viewer

hwm_viewer_SOURCES = hwm-viewer.cpp

hwm_viewer_CPPFLAGS = $(OGRE_CFLAGS) $(OIS_CFLAGS) -Wno-deprecated
hwm_viewer_LDADD = $(OGRE_LIBS) $(OIS_LIBS)
endif