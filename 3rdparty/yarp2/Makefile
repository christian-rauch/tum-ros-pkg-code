
all: installed

PKG_DIR=$(shell rospack find yarp2)/
INSTALL_DIR=$(PKG_DIR)/yarp2
BUILD_DIR=$(PKG_DIR)/build
SOURCE_DIR=$(BUILD_DIR)/yarp2

CMAKE=cmake
CMAKE_ARGS = -D CREATE_SHARED_LIBRARY=ON \
             -D CMAKE_INSTALL_PREFIX=$(INSTALL_DIR) \
             -D CMAKE_BUILD_TYPE=RELEASE \
             -D CREATE_LIB_MATH=ON \
             -D CREATE_GUIS=ON \
             -D CREATE_NEW_YARPVIEW=ON \
             -D CREATE_YARPSERVER3=ON \
             -D ENABLE_FORCE_RPATH=ON

CMAKE_BINDINGS_ARGS = -D CMAKE_INSTALL_PREFIX=$(INSTALL_DIR) \
                      -D CMAKE_BUILD_TYPE=RELEASE \
                      -D CREATE_PYTHON:BOOL=ON \
                      -D CREATE_CSHARP:BOOL=OFF \
                      -D YARP_DIR:PATH=../../build \
	                  -D PYTHON_INSTDIR:PATH=$(INSTALL_DIR)/lib/python/site-packages

checkedout:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && svn checkout -r 8126 https://yarp0.svn.sourceforge.net/svnroot/yarp0/trunk/yarp2
	touch checkedout

patched: checkedout
	cd $(SOURCE_DIR) && patch -p0 < $(PKG_DIR)/total.patch
	touch patched

built: wiped checkedout patched
	mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SOURCE_DIR)/build && make
	mkdir -p $(SOURCE_DIR)/example/swig/build
	cd $(SOURCE_DIR)/example/swig/build && $(CMAKE) $(CMAKE_BINDINGS_ARGS) ..
	cd $(SOURCE_DIR)/example/swig/build && make
	touch built

installed: built
	cd $(SOURCE_DIR)/build && make install
	cd $(SOURCE_DIR)/example/swig/build && make install
	touch installed

wiped: Makefile
	rm -rf $(INSTALL_DIR) $(BUILD_DIR) installed built checkedout patched
	touch wiped

clean:
	rm -rf $(BUILD_DIR) checkedout built patched

