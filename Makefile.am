AM_CXXFLAGS = -std=c++11 -Wall -Wextra -Wno-unused-parameter

AM_CPPFLAGS = $(VapourSynth_CFLAGS) $(FFTW3F_CFLAGS)

lib_LTLIBRARIES = libmvtools.la

BUILT_SOURCES = src/asm/const-a.o \
				src/asm/cpu-a.o \
				src/asm/pixel-a.o \
				src/asm/sad-a.o
CLEANFILES = $(BUILT_SOURCES)

.asm.o:
	$(AS) $(ASFLAGS) -o $@ $<

libmvtools_la_SOURCES = src/asm-placeholder.cpp \
						src/CopyCode.cpp \
						src/CPU.cpp \
						src/DCTFFTW.cpp \
						src/EntryPoint.cpp \
						src/FakeBlockData.cpp \
						src/FakeGroupOfPlanes.cpp \
						src/FakePlaneOfBlocks.cpp \
						src/GroupOfPlanes.cpp \
						src/Interpolation.cpp \
						src/MaskFun.cpp \
						src/MVAnalyse.cpp \
						src/MVClip.cpp \
						src/MVCompensate.cpp \
						src/MVDegrain1.cpp \
						src/MVDegrain2.cpp \
						src/MVDegrain3.cpp \
						src/MVFilter.cpp \
						src/MVFrame.cpp \
						src/MVRecalculate.cpp \
						src/MVSuper.cpp \
						src/Overlap.cpp \
						src/Padding.cpp \
						src/PlaneOfBlocks.cpp \
						src/asm/const-a.asm \
						src/asm/cpu-a.asm \
						src/asm/pixel-a.asm \
						src/asm/sad-a.asm

libmvtools_la_LDFLAGS = -no-undefined -avoid-version \
						-Xlinker src/asm/const-a.o \
						-Xlinker src/asm/cpu-a.o \
						-Xlinker src/asm/pixel-a.o \
						-Xlinker src/asm/sad-a.o

libmvtools_la_LIBADD = $(FFTW3F_LIBS)

libmvtools_la_DEPENDENCIES = $(BUILT_SOURCES)
