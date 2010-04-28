all: installed

INSTALL_DIR=`rospack find people_detector`
SVN_DIR = people_detector
SVN_URL = https://svnradig.informatik.tu-muenchen.de/svn/people_detector
#Authorized access
#SVN_URL = svn+ssh://svn@svnradig.informatik.tu-muenchen.de/people_detector
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR)
	cd $(SVN_DIR) && make && touch installed
	#rm patched rospack_nosubdirs

#delete ROS node only
clean:
	cd $(INSTALL_DIR) && rm -rf $(SVN_DIR)

#delete everything
wipe:	clean
	cd $(INSTALL_DIR) && rm -rf $(SVN_DIR) patched rospack_nosubdirs

include $(shell rospack find mk)/cmake.mk