include $(shell rospack find mk)/cmake.mk

download:
	mkdir -p data
	cd data && wget http://ias.cs.tum.edu/~pangerci/ros_tutorial_data.zip && unzip ros_tutorial_data.zip

wipe:	
	rm -rf data