
PACKAGE_DIR=$(shell rospack find cram_emacs_repl)

all: .slime .swank

.slime: Makefile
	make clean
	git clone git://sbcl.boinkor.net/slime.git
	touch slime/rospack_nosubdirs
	touch .slime

.swank: 
	[ -f $(HOME)/.swank.lisp ] || cp $(PACKAGE_DIR)/swank.lisp $(HOME)/.swank.lisp
	touch .swank
	
clean:
	rm -rf slime .slime .swank

