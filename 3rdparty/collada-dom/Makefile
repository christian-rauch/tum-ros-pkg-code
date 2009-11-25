all: installed

INSTALL_DIR=$(shell rospack find collada-dom)/collada-dom
BUILD_DIR=$(shell rospack find collada-dom)/build
SOURCE_DIR=$(BUILD_DIR)/collada-dom/dom
CUSTOM_MAKE_ADDS=customSettings.mk
COLLADA_PATCH=$(shell rospack find collada-dom)/collada-dom.patch

checkedout:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && wget http://downloads.sourceforge.net/project/collada-dom/Collada%20DOM/Collada%20DOM%202.2/collada-dom-2.2.zip?use_mirror=surfnet
	cd $(BUILD_DIR) && unzip collada-dom-2.2.zip
	touch $(shell rospack find collada-dom)/checkedout

installed: checkedout patched
	cd $(SOURCE_DIR) && mkdir -p build/linux-1.4 && make 
	touch $(shell rospack find collada-dom)/installed

patched: collada-dom.patch checkedout
	cd $(BUILD_DIR) && patch -p0 < $(COLLADA_PATCH)
	touch $(shell rospack find collada-dom)/patched

clean:
	rm -rf $(BUILD_DIR)
	rm -f checkedout wiped installed patched

wipe: clean

