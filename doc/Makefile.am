#
#  NASPRO - The NASPRO Architecture for Sound Processing
#  Core library
#
#  Copyright (C) 2008 Stefano D'Angelo <zanga.mail@gmail.com>
#
#  See the COPYING file for license conditions.
#

# TODO: uninstall for html/GENERATE_DOCSET

html:
if BUILD_DOC
	rm -fr api/html/*
	@$(SED) 's/GENERATE_LATEX[[:space:]]*=[[:space:]]*YES/GENERATE_LATEX=NO/g' Doxyfile \
	| $(SED) 's/GENERATE_RTF[[:space:]]*=[[:space:]]*YES/GENERATE_RTF=NO/g' \
	| $(SED) 's/GENERATE_MAN[[:space:]]*=[[:space:]]*YES/GENERATE_MAN=NO/g' \
	| $(SED) 's/GENERATE_XML[[:space:]]*=[[:space:]]*YES/GENERATE_XML=NO/g' \
	| $(DOXYGEN) -
	@if test "@GENERATE_DOCSET@" = YES; then \
	  cd $(builddir)/api/html && $(MAKE) && cd ../..; \
	fi
else
	@echo "The package was not configured for rebuilding documentation."
endif

pdf:
if BUILD_DOC
	@if test "@GENERATE_LATEX@" = YES; then \
	  rm -fr api/latex/*; \
	  $(MAKE) $(AM_MAKEFLAGS) api/latex/refman.pdf; \
	else \
	  echo "The package was not configured for rebuilding LaTeX output."; \
	fi
else
	@echo "The package was not configured for rebuilding documentation."
endif

api/latex/refman.pdf: api/latex/refman.tex
if BUILD_DOC
	@cd api/latex && \
	$(PDFLATEX) refman.tex && \
	$(MAKEINDEX) refman.idx && \
	$(PDFLATEX) refman.tex && \
	latex_count=5 && \
	while $(EGREP) -s 'Rerun (LaTeX|to get cross-references right)' refman.pdf.log && [ $$latex_count -gt 0 ]; do \
	  echo "Rerunning latex..."; \
	  $(PDFLATEX) refman.tex; \
	  latex_count=`expr $$latex_count - 1`; \
	done && \
	cd ../..
endif

api/latex/refman.tex:
if BUILD_DOC
	@$(SED) 's/GENERATE_HTML[[:space:]]*=[[:space:]]*YES/GENERATE_HTML=NO/g' Doxyfile \
	| $(SED) 's/GENERATE_QHP[[:space:]]*=[[:space:]]*YES/GENERATE_QHP=NO/g' \
	| $(SED) 's/QHG_LOCATION[[:space:]]*=[[:space:]]*.*/QHG_LOCATION=/g' \
	| $(SED) 's/QCH_FILE[[:space:]]*=[[:space:]]*.*/QCH_FILE=/g' \
	| $(SED) 's/GENERATE_HTMLHELP[[:space:]]*=[[:space:]]*YES/GENERATE_HTMLHELP=NO/g' \
	| $(SED) 's/GENERATE_RTF[[:space:]]*=[[:space:]]*YES/GENERATE_RTF=NO/g' \
	| $(SED) 's/GENERATE_MAN[[:space:]]*=[[:space:]]*YES/GENERATE_MAN=NO/g' \
	| $(SED) 's/GENERATE_XML[[:space:]]*=[[:space:]]*YES/GENERATE_XML=NO/g' \
	| $(DOXYGEN) -
endif

man:
if BUILD_DOC
	rm -fr api/man/man3/*
	@$(SED) 's/GENERATE_HTML[[:space:]]*=[[:space:]]*YES/GENERATE_HTML=NO/g' Doxyfile \
	| $(SED) 's/GENERATE_QHP[[:space:]]*=[[:space:]]*YES/GENERATE_QHP=NO/g' \
	| $(SED) 's/QHG_LOCATION[[:space:]]*=[[:space:]]*.*/QHG_LOCATION=/g' \
	| $(SED) 's/QCH_FILE[[:space:]]*=[[:space:]]*.*/QCH_FILE=/g' \
	| $(SED) 's/GENERATE_HTMLHELP[[:space:]]*=[[:space:]]*YES/GENERATE_HTMLHELP=NO/g' \
	| $(SED) 's/GENERATE_LATEX[[:space:]]*=[[:space:]]*YES/GENERATE_LATEX=NO/g' \
	| $(SED) 's/GENERATE_RTF[[:space:]]*=[[:space:]]*YES/GENERATE_RTF=NO/g' \
	| $(SED) 's/GENERATE_XML[[:space:]]*=[[:space:]]*YES/GENERATE_XML=NO/g' \
	| $(DOXYGEN) -
	@links='api/man/man3/NACORE*.3 api/man/man3/nacore*.3'; \
	for p in $$links; do \
	  $(SED) 's/[a-z]*\.h\.3/@NAMESPACE@&/' $$p > $$p.new; \
	  mv $$p.new $$p; \
	done
	@headers='$(top_srcdir)/include/NASPRO/core/*.h'; \
	for p in $$headers; do \
	  p=`basename $$p`; \
	  mv "api/man/man3/$$p.3" "api/man/man3/@NAMESPACE@$$p.3"; \
	done
else
	@echo "The package was not configured for rebuilding documentation."
endif

html_DATA = api/html/*.html api/html/*.css api/html/*.png api/html/*.gif \
	    api/html/*.chm api/html/*.qch
pdf_DATA = api/latex/refman.pdf

# Wihtout the following targets, nothing get installed...
# Automake bug or am I missing something???
install-html:
	@$(NORMAL_INSTALL)
	test -z "$(htmldir)" || $(MKDIR_P) "$(DESTDIR)$(htmldir)"
	@list='$(html_DATA)'; \
	for p in $$list; do \
	  if test -f "$$p"; then d=; else d="$(srcdir)/"; fi; \
	  f=$(am__strip_dir) \
	  echo " $(INSTALL_DATA) '$$d$$p' '$(DESTDIR)$(htmldir)/$$f'"; \
	  $(INSTALL_DATA) "$$d$$p" "$(DESTDIR)$(htmldir)/$$f"; \
	done
	@if test "@GENERATE_DOCSET@" = YES; then \
	  cd api/html && $(MAKE) install && cd ../..
	fi

install-pdf:
	@$(NORMAL_INSTALL)
	test -z "$(pdfdir)" || $(MKDIR_P) "$(DESTDIR)$(pdfdir)"
	@list='$(pdf_DATA)'; \
	for p in $$list; do \
	  if test -f "$$p"; then d=; else d="$(srcdir)/"; fi; \
	  f=$(am__strip_dir) \
	  echo " $(INSTALL_DATA) '$$d$$p' '$(DESTDIR)$(pdfdir)/$$f'"; \
	  $(INSTALL_DATA) "$$d$$p" "$(DESTDIR)$(pdfdir)/$$f"; \
	done

man3_MANS = api/man/man3/NACORE* api/man/man3/nacore*

EXTRA_DIST = $(html_DATA) $(pdf_DATA) $(man3_MANS)

$(EXTRA_DIST):

clean-local:
	@list='api/html/* api/latex/* api/man/man3/*'; \
	for p in $$list; do \
	  remove=yes; \
	  for f in $(EXTRA_DIST); do \
	    if test "$$p" = "$$f"; then \
	      remove=no; \
	    fi; \
	  done; \
	  if test $$remove = yes; then \
	    echo " rm -f '$$p'"; \
	    rm -f "$$p"; \
	  fi; \
	done
