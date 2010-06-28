all: installed

SVN_DIR = $(shell pwd)/build/people_detector
SVN_URL = https://svnradig.informatik.tu-muenchen.de/svn/people_detector
SVN_CMDLINE = svn --username reader --password reader
SOURCE_DIR = $(SVN_DIR)
INSTALL_PREFIX = $(shell pwd)/people_detector

#Authorized access
#SVN_URL = svn+ssh://svn@svnradig.informatik.tu-muenchen.de/people_detector

include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) Makefile
	cd $(SOURCE_DIR) && INSTALL_PREFIX=$(INSTALL_PREFIX) make
#	cd $(SOURCE_DIR) && INSTALL_PREFIX=$(INSTALL_PREFIX) make install
	touch installed

clean:
	cd $(SOURCE_DIR) && make clean
	rm -rf installed

wipe: clean
	rm -rf build omnidrive rospack_nosubdirs

