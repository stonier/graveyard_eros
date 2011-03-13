# This file is part of mingw-cross-env.
# See doc/index.html for further information.

# libpng
PKG             := libpng
$(PKG)_IGNORE   :=
#$(PKG)_VERSION  := 1.4.4
$(PKG)_VERSION  := 1.4.5
#$(PKG)_CHECKSUM := 638c5e184b569167148b0cb0f0f2a799660ea44f
$(PKG)_CHECKSUM := 58005bee4a3452f2a3406b07910912f1266cde59
$(PKG)_SUBDIR   := libpng-$($(PKG)_VERSION)
$(PKG)_FILE     := libpng-$($(PKG)_VERSION).tar.bz2
$(PKG)_WEBSITE  := http://www.libpng.org/
$(PKG)_URL      := http://$(SOURCEFORGE_MIRROR)/project/$(PKG)/$(PKG)$(subst .,,$(call SHORT_PKG_VERSION,$(PKG)))/$($(PKG)_VERSION)/$($(PKG)_FILE)
$(PKG)_URL_2    := ftp://ftp.simplesystems.org/pub/$(PKG)/png/src/$($(PKG)_FILE)
$(PKG)_DEPS     := gcc zlib

define $(PKG)_UPDATE
    wget -q -O- 'http://libpng.git.sourceforge.net/git/gitweb.cgi?p=libpng/libpng;a=tags' | \
    grep '<a class="list name"' | \
    $(SED) -n 's,.*<a[^>]*>v\([0-9][^<]*\)<.*,\1,p' | \
    grep -v alpha | \
    grep -v beta | \
    grep -v rc | \
    grep -v '^1\.2\.' | \
    head -1
endef

define $(PKG)_BUILD
    cd '$(1)' && ./configure \
        --host='$(TARGET)' \
        --disable-shared \
        --prefix='$(PREFIX)/$(TARGET)'
    $(MAKE) -C '$(1)' -j '$(JOBS)' install bin_PROGRAMS= sbin_PROGRAMS= noinst_PROGRAMS=
endef
