## Process this file with automake to produce Makefile.in
ACLOCAL_AMFLAGS = -I m4
AUTOMAKE_OPTIONS = foreign

SUBDIRS = sim ogre doc

man:
	cd doc && $(MAKE) $(AM_MAKEFLAGS) man && cd ..

