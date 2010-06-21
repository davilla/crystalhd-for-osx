#
#  Broadcom "BCM970012/BCM970015" Crystal HD device interface library.
#
#
BCLIB=libcrystalhd.dylib
LIBDIR=/usr/lib

AT   = @
ECHO = ${AT} echo
BCGCC = g++

ROOTDIR = ../..

INCLUDES = -I./ -I/usr/include -I$(ROOTDIR)/include
INCLUDES += -I$(ROOTDIR)/include/link


CPPFLAGS = -D__LINUX_USER__
# -DLDIL_PRINTS_ON
# -D_USE_SHMEM_

CPPFLAGS += ${INCLUDES}
CPPFLAGS += -g -O0 -Wall -fPIC -arch i386 -shared -isysroot /Developer/SDKs/MacOSX10.4u.sdk -mmacosx-version-min=10.4
LDFLAGS = -dynamiclib -twolevel_namespace 

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
	$(BCGCC) $(CPPFLAGS) $(LDFLAGS) -o $(BCLIB) ${OBJFILES} 

help:
	${ECHO} OBJFILES = ${OBJFILES}
	${ECHO} SRCFILES = ${SRCFILES}
	${ECHO} LNM = ${BCLIB} ${BCLIB_SL}

clean:
	rm -f  ${BCLIB} *.o
	rm -f  ${OBJFILES}

install:
	# this is messy yes, xbmc needs the headers in /opt/local/include
	if [ -d /opt/local/include ] ; then mkdir -p /opt/local/include/libcrystalhd ; fi
	if [ -d /opt/local/include ] ; then cp libcrystalhd_if.h /opt/local/include/libcrystalhd/; fi
	if [ -d /opt/local/include ] ; then cp $(ROOTDIR)/include/bc_dts_defs.h /opt/local/include/libcrystalhd/ ; fi
	if [ -d /opt/local/include ] ; then cp $(ROOTDIR)/include/bc_dts_types.h /opt/local/include/libcrystalhd/ ; fi
	mkdir -p /usr/include/libcrystalhd
	cp libcrystalhd_if.h /usr/include/libcrystalhd/
	cp $(ROOTDIR)/include/bc_dts_defs.h /usr/include/libcrystalhd/
	cp $(ROOTDIR)/include/bc_dts_types.h /usr/include/libcrystalhd/
	cp $(ROOTDIR)/firmware/fwbin/70012/bcm70012fw.bin $(LIBDIR)
	cp $(ROOTDIR)/firmware/fwbin/70015/bcm70015fw.bin $(LIBDIR)
	cp $(BCLIB) $(LIBDIR)
	install -m 755 $(BCLIB) $(LIBDIR)

