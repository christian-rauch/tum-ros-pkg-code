# include $(shell rospack find mk)/cmake.mk

BUILD_DIR=build
CMAKE=cmake

all:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(BUILD_DIR) && make

clean:
	rm -rf $(BUILD_DIR)
   