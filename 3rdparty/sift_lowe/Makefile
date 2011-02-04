
all: installed

BUILD_DIR=build
RESULTS=results
ARCHIVE=siftDemoV4.zip
ARCHIVE_URL=http://www.cs.ubc.ca/~lowe/keypoints/siftDemoV4.zip


checkedout:
	mkdir -p $(BUILD_DIR) $(RESULTS)
	cd $(BUILD_DIR) && \
	wget ${ARCHIVE_URL} && unzip ${ARCHIVE} && touch checkedout

installed: checkedout
	cd  $(BUILD_DIR)/siftDemoV4 && make
	touch installed

clean:
	rm -rf $(BUILD_DIR) $(RESULTS)
	rm -f checkedout installed

