
all: installed

BUILD_DIR=$(shell pwd)/build
INCLUDE_DIR=$(shell pwd)/include
INCLUDE_LIBSVM_DIR=$(shell pwd)/include/libsvm
LIB_DIR=$(shell pwd)/lib

VERSION=3.0

checkedout:
	mkdir -p $(BUILD_DIR) $(INCLUDE_DIR) $(INCLUDE_LIBSVM_DIR) $(LIB_DIR)
	cd $(BUILD_DIR) &&\
	wget -O libsvm-$(VERSION).tar.gz http://www.csie.ntu.edu.tw/~cjlin/cgi-bin/libsvm.cgi?+http://www.csie.ntu.edu.tw/~cjlin/libsvm+tar.gz && tar xf libsvm-$(VERSION).tar.gz
	touch checkedout

installed: wiped checkedout
	cd $(BUILD_DIR)/libsvm-$(VERSION) && make && make lib
	cd $(BUILD_DIR)/libsvm-$(VERSION) && cp svm.h $(INCLUDE_LIBSVM_DIR)
	cd $(BUILD_DIR)/libsvm-$(VERSION) && cp libsvm*  $(LIB_DIR)
	cd  $(LIB_DIR) && ln -s libsvm.so.2 libsvm.so
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf $(BUILD_DIR)  $(INCLUDE_DIR) $(LIB_DIR)
	rm -f checkedout wiped installed

wipe: clean

