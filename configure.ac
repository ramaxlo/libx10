AC_INIT([libx10], [0.1], [ramaxlo@gmail.com])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

AC_PROG_CC
AC_PROG_RANLIB
AC_ARG_ENABLE(debug, AC_HELP_STRING([--enable-debug], [Enable debugging macro]),
	[AC_DEFINE([DEBUG], [1], [Define if debugging is enabled])])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_FILES([Makefile src/Makefile test/Makefile include/Makefile])
AC_OUTPUT
