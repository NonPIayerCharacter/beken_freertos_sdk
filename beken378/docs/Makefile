# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD         = sphinx-build
SOURCEDIR_zh_CN     = zh_CN
BUILDDIR_zh_CN      = build/zh_CN

SOURCEDIR_EN     = en
BUILDDIR_EN      = build/en

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

clean:
	rm -Rf ./build

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR_zh_CN)" "$(BUILDDIR_zh_CN)" $(SPHINXOPTS) $(O)
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR_EN)" "$(BUILDDIR_EN)" $(SPHINXOPTS) $(O)
	cp ./hw_doc -R ./build/
	mv ./build/zh_CN/html build/zh_CN/latest
	mv ./build/en/html ./build/en/latest


