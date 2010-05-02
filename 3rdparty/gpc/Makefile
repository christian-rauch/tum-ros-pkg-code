all: installed

ROOT_DIR=$(shell pwd)
TARGET_DIR=$(shell pwd)/gpc
ZIP_FILE=gpc232-release.zip

installed: checkedout
	mkdir include && mkdir lib
	cd $(TARGET_DIR) && gcc -c -fPIC gpc.c -o gpc.o && gcc -shared -Wl,-soname,libgpc.so -o libgpc.so  gpc.o
	cd include && ln -s ../gpc/gpc.h  gpc.h
	cd lib && ln -s ../gpc/libgpc.so  libgpc.so
	touch installed

checkedout:
	mkdir $(TARGET_DIR)
	cd $(TARGET_DIR) && wget ftp://ftp.cs.man.ac.uk/pub/toby/gpc/gpc232-release.zip && unzip $(ZIP_FILE)
	touch checkedout

polytest:	
	gcc test/test.c -o test/test -I./include -L./lib -lgpc

clean:
	rm -rf $(TARGET_DIR) include lib checkedout installed test/test *~

