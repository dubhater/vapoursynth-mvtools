AC_INIT([MVTools], [1], [https://github.com/dubhater/vapoursynth-mvtools/issues], [MVTools], [https://github.com/dubhater/vapoursynth-mvtools/])

: ${CXXFLAGS="-std=c++11 -O2"}

AM_INIT_AUTOMAKE([foreign no-dist-gzip dist-xz subdir-objects no-define])
AM_SILENT_RULES([yes])

LT_INIT([disable-static win32-dll])

AC_PROG_CXX

AC_CANONICAL_HOST

AS_CASE(
   [$host_cpu], [i?86],
   [AS_CASE(
      [$host_os], [cygwin*|mingw*],
      [LDFLAGS="-Wl,--kill-at"]
   )]
)

PKG_CHECK_MODULES([VapourSynth], [vapoursynth])

PKG_CHECK_MODULES([FFTW3F], [fftw3f])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
