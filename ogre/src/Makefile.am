EXTRA_DIST = SampleApp.cpp

if DO_OGRE
noinst_PROGRAMS = SampleApp

SampleApp_SOURCES = SampleApp.cpp

SampleApp_CPPFLAGS = -I$(srcdir)/../include $(OGRE_CFLAGS) $(OIS_CFLAGS) -Wno-deprecated
SampleApp_LDADD = $(OGRE_LIBS) $(OIS_LIBS)
endif