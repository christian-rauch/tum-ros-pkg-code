
PACKAGE_DIR=$(shell rospack find gsll)

all: installed

downloaded:
	git clone git://repo.or.cz/gsll.git
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p asdf
	ln -s ${PACKAGE_DIR}/gsll/gsll.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf gsll asdf
	rm -f downloaded rospack_nosubdirs installed patched

