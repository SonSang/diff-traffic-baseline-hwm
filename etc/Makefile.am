AM_CXXFLAGS = $(GSL_CFLAGS)
ADD_LIBS = $(GSL_LIBS)

EXTRA_DIST = ss_kinematics.cpp lc_kinematics.cpp kinematic-table.py

if DO_KINEMATIC
noinst_PROGRAMS = ss_kinematic lc_kinematic

ss_kinematic_SOURCES = ss_kinematics.cpp
ss_kinematic_LDADD   = $(ADD_LIBS)

lc_kinematic_SOURCES = lc_kinematics.cpp
lc_kinematic_LDADD   = $(ADD_LIBS)
endif