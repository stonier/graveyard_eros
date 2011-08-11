#include $(shell rospack find eros_build)/mk/custom_installer.mk

PREFIX=/opt/mingw
TUPLE=i686-pc-mingw32
NAME=mingw-cross-env-2.21
TARBALL=${NAME}.tar.gz
URL=https://bitbucket.org/vog/mingw-cross-env/downloads/$(TARBALL)

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
	@echo "all       : downloads, extracts to /opt/mingw and compiles gcc."
	@echo "links     : installs links to binaries in /opt/mingw/usr/bin."
	@echo "install   : same as above."
	@echo "uninstall : uninstalls links and deletes /opt/mingw."
	@echo "download  : downloads the mingw cross tarball."

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



compile: extract
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
	else \
		echo "There is already an installed gcc in $(PREFIX)."; \
	fi

extract: download
	@echo "Extracting to $(PREFIX)"
	sudo mkdir -p $(PREFIX)
	sudo chown -R $(USER):$(USER) $(PREFIX)
	@if [ ! -f $(PREFIX)/Makefile ]; then \
		rm -rf build/$(NAME); \
		tar -xvzf build/$(TARBALL) -C build; \
		mv build/$(NAME)/* $(PREFIX); \
	else \
		echo "$(TARBALL) already extracted to $(PREFIX)."; \
	fi
	@cp patches/*.mk /opt/mingw/src
	@cp patches/*.patch /opt/mingw/src

download:
	@mkdir -p build
	@if [ -f build/$(TARBALL) ]; then \
		echo "$(TARBALL) already downloaded."; \
	else \
		cd build; wget $(URL); \
	fi
	
