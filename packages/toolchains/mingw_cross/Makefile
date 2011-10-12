#include $(shell rospack find eros_build)/mk/custom_installer.mk

PREFIX=/opt/mingw
TUPLE=i686-pc-mingw32
NAME=mingw-cross-env-2.21
URL=http://hg.savannah.nongnu.org/hgweb/mingw-cross-env

##############################################################################
# Primary targets
##############################################################################

.PHONY: clean

all: compile

install: links
	
uninstall:
	sudo rm -rf ${PREFIX}
	sudo rm -f /usr/local/bin/$(TUPLE)-*

clean:
	rm -rf build

##############################################################################
# Helper targets
##############################################################################

help:
	@echo "all       : clones to /opt/mingw and compiles gcc, qt, boost and apache libs."
	@echo "links     : installs links to binaries in /opt/mingw/usr/bin."
	@echo "install   : same as above."
	@echo "uninstall : uninstalls links and deletes /opt/mingw."
	@echo "clone     : just clone the stable mingw cross repo."

links:
	@if [ -d $(PREFIX)/usr/bin ]; then \
		for i in $(PREFIX)/usr/bin/*; do \
			sudo ln -sf $$i /usr/local/bin/`basename $$i`; \
		done; \
	else \
		echo "Not installed yet, run make first."; \
	fi
	sudo ln -sf ${PREFIX}/usr/${TUPLE}/lib/libboost_thread_win32-mt.a ${PREFIX}/usr/${TUPLE}/lib/libboost_thread-mt.a
	sudo ln -sf ${PREFIX}/usr/${TUPLE}/lib/libboost_thread_win32-mt-d.a ${PREFIX}/usr/${TUPLE}/lib/libboost_thread-mt-d.a



compile: clone
	@echo "Compiling mingw gcc, log4cxx, boost and qt."
	@if [ ! -f $(PREFIX)/usr/installed/gcc ]; then \
		cd $(PREFIX); make gcc log4cxx boost qt; \
	    echo; \
	    echo "Mingw cross environment successfully installed in /opt/mingw."; \
	    echo " - run 'make links' or 'make install' to install symbolic links in"; \
	    echo "   /usr/local/bin from binaries in /opt/mingw/usr/bin"; \
	    echo " - run 'make uninstall' to delete links and /opt/mingw."; \
	    echo; \
	    echo "The cross environment has many build scripts that can be used to"; \
	    echo "install packages into your toolchain. Refer to"; \
	    echo; \
	    echo "  http://mingw-cross-env.nongnu.org/ for a full list."; \
	    echo; \
	    echo "Example (installation of gdb and wxwidgets):"; \
	    echo; \
	    echo "  cd /opt/mingw"; \
	    echo "  make gdb wxwidgets"; \
	    echo; \
	    echo "If the download links should ever happen to be broken, try"; \
	    echo; \
	    echo "  cd /opt/mingw/src"; \
	    echo "  hg pull -u"; \
	    echo; \
	    echo "that will sometimes work. If it fails, then contact help as something is";  \
	    echo "probably really broken."; \ 
	    echo; \
	else \
		echo "There is already an installed gcc in $(PREFIX)."; \
	fi

clone: 
	@echo "Cloning stable branch to $(PREFIX)"
	sudo mkdir -p $(PREFIX)
	sudo chown -R $(USER):$(USER) $(PREFIX)
	@if [ ! -f $(PREFIX)/Makefile ]; then \
		hg clone -b stable $(URL) $(PREFIX); \
	else \
		echo "Mingw cross has already been cloned at $(PREFIX)."; \
	fi
	@cp patches/*.mk /opt/mingw/src
	@cp patches/*.c /opt/mingw/src
	@cp patches/*.patch /opt/mingw/src
