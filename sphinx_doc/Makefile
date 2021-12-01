# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = python3 -m sphinx
SPHINXINTL    = sphinx-intl
SOURCEDIR     = .
BUILDDIR      = _build
BUILDTYPE     = html
LANGPATH      = locale
LANGUAGE      = en
LANGS	      = $(notdir $(shell find $(LANGPATH)/ -maxdepth 1 -type d))

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

all: update $(LANGS)

update: gettext
	@$(SPHINXINTL) update -p "$(BUILDDIR)/gettext"

gettext:
	@$(SPHINXBUILD) -M gettext "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS)

en: LANGUAGE=en
en: 
	@$(SPHINXBUILD) -D language="$(LANGUAGE)" -b "$(BUILDTYPE)" "$(SOURCEDIR)" "$(BUILDDIR)/$(LANGUAGE)"

fr: LANGUAGE=fr
fr:
	@$(SPHINXBUILD) -D language="$(LANGUAGE)" -b "$(BUILDTYPE)" "$(SOURCEDIR)" "$(BUILDDIR)/$(LANGUAGE)"

# add any lang here
# es: LANGUAGE=es
# es: 
#	@$(SPHINXBUILD) -D language="$(LANGUAGE)" -b "$(BUILDTYPE)" "$(SOURCEDIR)" "$(BUILDDIR)/$(LANGUAGE)"

lang: gettext	 
	 @$(SPHINXINTL) update -p "$(BUILDDIR)/gettext" -l $(LANGUAGE)

clean:
	rm -rf "$(BUILDDIR)"

