
CFFI_ARCHIVE=cffi_0.10.5.tar.gz

all: installed

$(CFFI_ARCHIVE):
	wget http://common-lisp.net/project/cffi/releases/$(CFFI_ARCHIVE)

installed: $(CFFI_ARCHIVE)
	tar xvzf $(CFFI_ARCHIVE)
	touch rospack_nosubdirs
	rosrun lisp_asdf_manager register_asdf cffi cffi_0.10.5/cffi.asd
	rosrun lisp_asdf_manager register_asdf cffi cffi_0.10.5/cffi-uffi-compat.asd
	touch installed

clean:
	rosrun lisp_asdf_manager unregister_asdf cffi.asd
	rosrun lisp_asdf_manager unregister_asdf cffi-uffi-compat.asd
	rm -rf cffi_0.10.5
	rm -f $(CFFI_ARCHIVE)
	rm -f rospack_nosubdirs
	rm -f installed

