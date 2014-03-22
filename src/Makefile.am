## Process this file with automake to produce Makefile.in
ACLOCAL_AMFLAGS=-I m4

AM_CPPFLAGS= 

AM_CFLAGS = 

bin_PROGRAMS = \
	boblightd \
	boblight-enigma2

boblightd_SOURCES = \
	util/condition.cpp \
	util/log.cpp \
	util/misc.cpp \
	util/mutex.cpp \
	util/thread.cpp \
	util/timer.cpp \
	util/serialport.cpp \
	util/timeutils.cpp \
	main.cpp \
	util/tcpsocket.cpp \
	util/messagequeue.cpp \
	client.cpp \
	light.cpp \
	configuration.cpp \
	device/device.cpp \
	device/devicepopen.cpp \
	device/devicers232.cpp \
	device/deviceltbl.cpp \
	device/devicedioder.cpp \
	device/deviceambioder.cpp \
	util/signaltimer.cpp \
	util/daemonize.cpp \
	util/modp_numtoa.c

boblightd_LDFLAGS =

if HAVE_LIBUSB
boblightd_SOURCES += device/deviceibelight.cpp \
	device/devicelightpack.cpp
boblightd_LDFLAGS += -lusb-1.0
endif

boblightd_LDADD = 

boblight_enigma2_SOURCES = \
	clients/boblight-enigma2/boblight-enigma2.cpp \
	clients/boblight-enigma2/flagmanager-enigma2.cpp \
	clients/boblight-enigma2/grabber-enigma2.cpp \
	clients/flagmanager.cpp \
	util/threading.cpp \
	util/timer.cpp \
	util/mutex.cpp \
	util/condition.cpp \
	util/timeutils.cpp \
	util/log.cpp \
	util/misc.cpp \
	util/daemonize.cpp \
	util/modp_numtoa.c

boblight_enigma2_LDFLAGS =

lib_LTLIBRARIES = \
	libboblight.la

libboblight_la_SOURCES = \
	lib/boblight_client.cpp \
	lib/boblight.cpp \
	util/messagequeue.cpp \
	util/misc.cpp \
	util/tcpsocket.cpp \
	util/modp_numtoa.c

libboblight_la_LDFLAGS =
   
libboblight_la_CPPFLAGS = $(AM_CPPFLAGS)
