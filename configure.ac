dnl Process this file with autoconf to produce a configure script.
dnl Created by Anjuta application wizard.

AC_INIT([Boblight For Enigma2],[0.8r7])

AM_INIT_AUTOMAKE
AC_CONFIG_HEADERS([config.h])
AM_MAINTAINER_MODE

AC_ISC_POSIX
AC_PROG_CXX
AC_PROG_CC

AC_HEADER_STDC

AM_PROG_LIBTOOL

dnl all use this
AC_CHECK_LIB(pthread,[main])
AC_CHECK_LIB(rt,[main])
AC_CHECK_LIB(m,[main])
AC_CHECK_LIB(dl,[main])


AC_ARG_WITH([libusb], [AS_HELP_STRING([--without-libusb], [Disable libusb support (no support for iBeLight)])], [],
            [with_libusb=check])

AS_IF([test "x$with_libusb" = xcheck], [
AC_CHECK_LIB([usb-1.0],[main],AC_DEFINE([HAVE_LIBUSB], [1], [ ]))
AC_CHECK_LIB([usb-1.0],[libusb_error_name],AC_DEFINE([HAVE_LIBUSB_ERROR_NAME], [1], [ ]))
AC_CHECK_HEADERS([libusb-1.0/libusb.h],[],[AC_MSG_ERROR([Missing a required header, please install it or use --without-libusb])])
])

AM_CONDITIONAL(HAVE_LIBUSB, [test "x$with_libusb" = xcheck])
AC_CHECK_DECL([clock_gettime], AC_DEFINE([HAVE_CLOCK_GETTIME], [1], [ ]), [], [[#include <time.h>]])

AC_OUTPUT([
Makefile
src/Makefile
])

AS_IF([test "x$with_libusb" != xcheck], [AC_MSG_WARN([--without-libusb passed, no support for iBelight and Lightpack])])

