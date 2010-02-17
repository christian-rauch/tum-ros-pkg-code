
ORO_ONTOLOGY_PATH=$(shell rospack find oro_ontology)

all: downloaded config/oro.conf

downloaded:
	git clone http://softs.laas.fr/git/robots/oro.git
	touch downloaded

config/oro.conf: config/oro.conf.in
	sed 's,@ORO_ONTOLOGY_PATH@,${ORO_ONTOLOGY_PATH},g' $< > $@

clean:
	rm -rf oro downloaded
	rm config/oro.conf

