# use `make VESC_TOOL=path/to/your/vesc_tool` to specify custom vesc_tool path
VESC_TOOL ?= vesc_tool
# use `make MINIFY_QML=0` to skip qml minification and pack the qml verbatim
MINIFY_QML ?= 1

all: refloat.vescpkg

refloat.vescpkg: src package.lisp README-pkg.md ui.qml
	$(VESC_TOOL) --buildPkg "refloat.vescpkg:package.lisp:ui.qml:0:README-pkg.md:Refloat"

src:
	$(MAKE) -C $@

VERSION=`cat version`

ifeq ($(strip $(MINIFY_QML)),1)
    MINIFY_CMD="./rjsmin.py"
else
    MINIFY_CMD="cat"
endif

README-pkg.md: README.md version
	cp $< $@
	echo "- Version: ${VERSION}" >> $@
	echo "- Build Date: `date --rfc-3339=seconds`" >> $@
	echo "- Git Commit: #`git rev-parse --short HEAD`" >> $@

ui.qml: ui.qml.in version
	cat $< | sed "s/{{VERSION}}/${VERSION}/g" | ${MINIFY_CMD} > $@

clean:
	rm -f refloat.vescpkg README-pkg.md ui.qml
	$(MAKE) -C src clean

.PHONY: all clean src
