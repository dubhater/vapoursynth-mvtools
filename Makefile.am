AM_CXXFLAGS = -std=c++11 -msse2 -Wall -Wextra -Wno-unused-parameter

AM_CPPFLAGS = $(VapourSynth_CFLAGS)

lib_LTLIBRARIES = libmvtools.la

BUILT_SOURCES = src/asm/const-a.o \
				src/asm/cpu-a.o \
				src/asm/pixel-a.o \
				src/asm/sad-a.o \
				src/asm/CopyCode.o \
				src/asm/Interpolation.o \
				src/asm/MVDegrains.o \
				src/asm/Overlap.o \
				src/asm/Variance.o
CLEANFILES = $(BUILT_SOURCES)

.asm.o:
	$(AS) $(ASFLAGS) -o $@ $<

libmvtools_la_SOURCES = src/asm-placeholder.cpp \
						src/CopyCode.cpp \
						src/CPU.cpp \
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
						src/SimpleResize.cpp \
						src/asm/const-a.asm \
						src/asm/cpu-a.asm \
						src/asm/pixel-a.asm \
						src/asm/sad-a.asm \
						src/asm/CopyCode.asm \
						src/asm/Interpolation.asm \
						src/asm/MVDegrains.asm \
						src/asm/Overlap.asm \
						src/asm/Variance.asm

libmvtools_la_LDFLAGS = -no-undefined -avoid-version \
						-Xlinker src/asm/const-a.o \
						-Xlinker src/asm/cpu-a.o \
						-Xlinker src/asm/pixel-a.o \
						-Xlinker src/asm/sad-a.o \
						-Xlinker src/asm/CopyCode.o \
						-Xlinker src/asm/Interpolation.o \
						-Xlinker src/asm/MVDegrains.o \
						-Xlinker src/asm/Overlap.o \
						-Xlinker src/asm/Variance.o

libmvtools_la_DEPENDENCIES = $(BUILT_SOURCES)
