#include $(shell rospack find eros_build)/mk/custom_installer.mk

PREFIX=/opt/mingw
TUPLE=i686-pc-mingw32
NAME=mingw-cross-env-2.16
TARBALL=mingw-cross-env-2.16.tar.gz
URL=http://download.savannah.gnu.org/releases-redirect/mingw-cross-env/$(TARBALL)

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
	@echo "all       : downloads, extracts to /opt/mingw and compiles gcc, qt."
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

compile: extract
	@if [ ! -f $(PREFIX)/usr/installed/qt ]; then \
		cd $(PREFIX); make gcc; \
	else \
		echo "There is already an installed gcc in $(PREFIX)."; \
	fi

extract: download
	@sudo mkdir -p $(PREFIX)
	sudo chown -R $(USER):$(USER) $(PREFIX)
	@if [ ! -f $(PREFIX)/Makefile ]; then \
		rm -rf build/$(NAME); \
		tar -xvzf build/$(TARBALL) -C build; \
		mv build/$(NAME)/* $(PREFIX); \
	else \
		echo "$(TARBALL) already extracted to $(PREFIX)."; \
	fi

download:
	@mkdir -p build
	@if [ -f build/$(TARBALL) ]; then \
		echo "$(TARBALL) already downloaded."; \
	else \
		cd build; wget $(URL); \
	fi
	
