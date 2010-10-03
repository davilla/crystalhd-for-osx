#
#  Broadcom "BCM970012/BCM970015" Crystal HD device interface library.
#
#
BCLIB_MINOR=6
BCLIB_MAJOR=3
BCLIB_NAME=libcrystalhd.dylib
BCLIB_SL=$(BCLIB_NAME).$(BCLIB_MAJOR)
BCLIB=$(BCLIB_NAME).$(BCLIB_MAJOR).$(BCLIB_MINOR)
LIBDIR=/usr/lib

AT   = @
ECHO = ${AT} echo
CXX = /usr/bin/g++-4.0
SDK = -isysroot /Developer/SDKs/MacOSX10.4u.sdk -mmacosx-version-min=10.4

ROOTDIR = ../..

INCLUDES = -I$(ROOTDIR)/include
INCLUDES += -I$(ROOTDIR)/include/link
INCLUDES += -I$(ROOTDIR)/include/flea


CXXFLAGS = -D__LINUX_USER__
# -DLDIL_PRINTS_ON
# -D_USE_SHMEM_

CXXFLAGS += ${INCLUDES}
CXXFLAGS += -O2 -Wall -fPIC -arch i386 -shared -fstrict-aliasing -msse2 $(SDK)
LDFLAGS = -dynamiclib -twolevel_namespace -compatibility_version $(BCLIB_MAJOR).5.0 -current_version $(BCLIB_MAJOR).$(BCLIB_MINOR).0

SRCFILES = 	libcrystalhd_if.cpp \
		libcrystalhd_int_if.cpp \
		libcrystalhd_fwcmds.cpp \
		libcrystalhd_priv.cpp \
		libcrystalhd_fwdiag_if.cpp \
		libcrystalhd_fwload_if.cpp \
		libcrystalhd_parser.cpp \
		linux_compatible.cpp

OBJFILES = ${SRCFILES:.cpp=.o}

all:help $(OBJFILES)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $(BCLIB) ${OBJFILES} 
	ln -sf $(BCLIB) $(BCLIB_NAME)
	ln -sf $(BCLIB) $(BCLIB_SL)

help:
	${ECHO} OBJFILES = ${OBJFILES}
	${ECHO} SRCFILES = ${SRCFILES}
	${ECHO} LNM = ${BCLIB} ${BCLIB_SL}

clean:
	rm -f  ${BCLIB} ${BCLIB_SL} ${BCLIB_NAME} *.o
	rm -f  ${OBJFILES}

install:
	# this is messy yes, xbmc needs the headers in /opt/local/include
	if [ -d /opt/local/include ] ; then mkdir -p /opt/local/include/libcrystalhd ; fi
	if [ -d /opt/local/include ] ; then cp libcrystalhd_if.h /opt/local/include/libcrystalhd/; fi
	if [ -d /opt/local/include ] ; then cp $(ROOTDIR)/include/bc_dts_defs.h /opt/local/include/libcrystalhd/ ; fi
	if [ -d /opt/local/include ] ; then cp $(ROOTDIR)/include/bc_dts_types.h /opt/local/include/libcrystalhd/ ; fi
	if [ -d /opt/local/include ] ; then cp $(ROOTDIR)/include/libcrystalhd_version.h /opt/local/include/libcrystalhd/ ; fi
	mkdir -p /usr/include/libcrystalhd
	cp libcrystalhd_if.h /usr/include/libcrystalhd/
	cp $(ROOTDIR)/include/bc_dts_defs.h /usr/include/libcrystalhd/
	cp $(ROOTDIR)/include/bc_dts_types.h /usr/include/libcrystalhd/
	cp $(ROOTDIR)/include/libcrystalhd_version.h /usr/include/libcrystalhd/
	cp $(ROOTDIR)/firmware/fwbin/70012/bcm70012fw.bin $(LIBDIR)
	cp $(ROOTDIR)/firmware/fwbin/70015/bcm70015fw.bin $(LIBDIR)
	cp $(BCLIB) $(LIBDIR)
	install -m 755 $(BCLIB) $(LIBDIR)
	(cd $(DESTDIR)$(LIBDIR); ln -sf $(BCLIB) $(BCLIB_NAME))
	(cd $(DESTDIR)$(LIBDIR); ln -sf $(BCLIB) $(BCLIB_SL))

