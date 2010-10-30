
all: ias_karlstrasse_20100730.pgm ics_karlstrasse_20100825.pgm ias_karlstrasse_20100830.pgm edeka20100928.pgm ias_karlstrasse_20101007.pgm ias_karlstrasse_20101008.pgm ccrl_20101029.pgm

clean:
	rm *.pgm

%.pgm: %.png
	convert $< $@
