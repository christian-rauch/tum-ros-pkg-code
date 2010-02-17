#
# ROS Package for oro-server
#
# Copyright (c) 2010 TUM-IAS/LAAS-CNRS
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any purpose
# with or without   fee is hereby granted, provided   that the above  copyright
# notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS  SOFTWARE INCLUDING ALL  IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS. IN NO EVENT SHALL THE AUTHOR  BE LIABLE FOR ANY SPECIAL, DIRECT,
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR  ANY DAMAGES WHATSOEVER RESULTING  FROM
# LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
# OTHER TORTIOUS ACTION,   ARISING OUT OF OR IN    CONNECTION WITH THE USE   OR
# PERFORMANCE OF THIS SOFTWARE.
#
#                                          Séverin Lemaignan on Mon 15 Feb 2010
#                                   (no way to put the date in the wrong order)

PKGNAME=	oro-server
PKGVERSION=	0.7.3
DISTNAME=	$(PKGNAME)-${PKGVERSION}

OPENROBOTS_PUBLIC_FTP=	ftp://softs.laas.fr/pub/openrobots

PREFIX=	$(shell rospack find oro_server)

INSTALL_DIR=$(PREFIX)
BUILD_DIR=$(PREFIX)/work

UNTAR=tar -xf

CLASSPATH=$(shell rospack export --lang=java --attrib=classpath oro_server | sed 's/ /:/g')

all: installed

checkedout:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && wget $(OPENROBOTS_PUBLIC_FTP)/$(PKGNAME)/$(DISTNAME).tar.gz
	cd $(BUILD_DIR) && $(UNTAR) $(DISTNAME).tar.gz
	touch checkedout

installed: checkedout
	mkdir -p $(INSTALL_DIR)
	mkdir -p $(INSTALL_DIR)/bin
	cd $(BUILD_DIR)/$(DISTNAME) && make CLASSPATH=$(CLASSPATH) PREFIX=$(INSTALL_DIR) install
	touch installed

clean:
	rm -rf $(BUILD_DIR)
	rm -f checkedout wiped installed patched

wipe: clean
	rm -rf $(INSTALL_DIR)/etc
	rm -rf $(INSTALL_DIR)/bin
	rm -rf $(INSTALL_DIR)/java
	rmdir --ignore-fail-on-non-empty $(INSTALL_DIR)

