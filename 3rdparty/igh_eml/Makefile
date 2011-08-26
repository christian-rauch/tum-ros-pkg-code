
SOURCE_DIR=build/igh_eml

all: installed

downloaded: Makefile
	rm -rf build
	mkdir -p build
	hg clone http://etherlabmaster.hg.sourceforge.net:8000/hgroot/etherlabmaster/etherlabmaster $(SOURCE_DIR)
	#hg clone $(HOME)/work/base/ethercat-default $(SOURCE_DIR)
	cd $(SOURCE_DIR); hg checkout -r e9f722488fcd
	touch $(SOURCE_DIR)/config.h
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p include lib
	make libethercat
	ln -sf ../$(SOURCE_DIR)/include include/igh_eml

	touch installed

clean:
	rm -rf build
	rm -f downloaded rospack_nosubdirs installed


# direct compile rules. We only need the library, so we can get rid of the kernel-header dependency

LIB_OBJ=$(patsubst %.c,%.o,$(wildcard $(SOURCE_DIR)/lib/*.c))

libethercat: $(LIB_OBJ)
	echo $(LIB_OBJ)
	gcc -shared -Wl,-soname -Wl,libethercat.so -o lib/libethercat.so $(LIB_OBJ)

.c.o:
	gcc -I $(SOURCE_DIR) -I $(SOURCE_DIR)/lib -fPIC -o $@ -c $<
