AC_INIT([MVTools], [1], [https://github.com/dubhater/vapoursynth-mvtools/issues], [MVTools], [https://github.com/dubhater/vapoursynth-mvtools/])

: ${CXXFLAGS="-std=c++11 -O2"}

AM_INIT_AUTOMAKE([foreign no-dist-gzip dist-xz subdir-objects no-define])
AM_SILENT_RULES([yes])

LT_INIT([disable-static win32-dll])

AC_PROG_CXX

AC_CANONICAL_HOST




AC_CHECK_PROGS([YASM], [yasm])
AS_IF(
   [test "x$YASM" = "x"],
   [AC_MSG_ERROR([yasm required but not found])],
   [AS="$YASM"]
)

ASFLAGS="$ASFLAGS -Dprivate_prefix=mvtools -DHIGH_BIT_DEPTH=0 -DBIT_DEPTH=8"

AS_CASE(
  [$host_cpu],
  [i?86],   [BITS="32" ASFLAGS="$ASFLAGS -DARCH_X86_64=0"],
  [x86_64], [BITS="64" ASFLAGS="$ASFLAGS -DARCH_X86_64=1 -DPIC -m amd64"],
  [AC_MSG_ERROR([Currently MVTools can only be compiled for Intel CPUs.])]
)

AS_CASE(
   [$host_os],
   [darwin*],
   [
      ASFLAGS="$ASFLAGS -f macho$BITS -DPREFIX"
   ],
   [*linux*],
   [
      ASFLAGS="$ASFLAGS -f elf"
   ],
   [gnu*],
   [
      ASFLAGS="$ASFLAGS -f elf"
   ],
   [cygwin*],
   [
      ASFLAGS="$ASFLAGS -f win32"
      AS_IF(
         [test "x$BITS" = "x32"],
         [ASFLAGS="$ASFLAGS -DPREFIX" LDFLAGS="-Wl,--kill-at"]
      )
   ],
   [mingw*],
   [
      ASFLAGS="$ASFLAGS -f win32"
      AS_IF(
         [test "x$BITS" = "x32"],
         [ASFLAGS="$ASFLAGS -DPREFIX" LDFLAGS="-Wl,--kill-at"]
      )
   ],
   [AC_MSG_ERROR([Unknown host OS])]
)
AC_SUBST([ASFLAGS])



PKG_CHECK_MODULES([VapourSynth], [vapoursynth])

PKG_CHECK_MODULES([FFTW3F], [fftw3f])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
