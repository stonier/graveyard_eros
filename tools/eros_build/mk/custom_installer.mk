include $(shell rospack find mk)/cmake.mk

install:
		@if [ -d build ]; then \
			cd build; \
			sudo $(MAKE) custom_install; \
		else \
			$(MAKE) all; \
			cd build; \
			sudo $(MAKE) custom_install; \
		fi 

uninstall:
		@if [ -d build ]; then \
			cd build; \
			sudo $(MAKE) custom_uninstall; \
		else \
			echo No build directory; \
		fi
