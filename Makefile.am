warningflags = -Wall -Wextra -Wno-unused-parameter -Wshadow
commonflags = -O2 -msse2 -mfpmath=sse $(STACKREALIGN) $(warningflags)
AM_CFLAGS = -std=c99 $(commonflags)
AM_CXXFLAGS = -std=c++11 $(commonflags)

AM_CPPFLAGS = $(VapourSynth_CFLAGS) $(FFTW3F_CFLAGS)

lib_LTLIBRARIES = libmvtools.la

AM_V_YASM = $(am__v_YASM_$(V))
am__v_YASM_ = $(am__v_YASM_$(AM_DEFAULT_VERBOSITY))
am__v_YASM_0 = @echo "  YASM    " $@;
am__v_YASM_1 =

.asm.lo:
	$(AM_V_YASM)$(LIBTOOL) $(AM_V_lt) --mode=compile --tag=CC $(AS) $(ASFLAGS) -o $@ $< -prefer-non-pic

libmvtools_la_SOURCES =	src/CPU.c \
						src/DCT.cpp \
						src/DCTFFTW.cpp \
						src/EntryPoint.c \
						src/FakeBlockData.cpp \
						src/FakeGroupOfPlanes.cpp \
						src/FakePlaneOfBlocks.cpp \
						src/GroupOfPlanes.cpp \
						src/MaskFun.cpp \
						src/MVAnalyse.cpp \
						src/MVBlockFPS.cpp \
						src/MVClip.cpp \
						src/MVCompensate.cpp \
						src/MVDegrains.cpp \
						src/MVFilter.cpp \
						src/MVFinest.cpp \
						src/MVFlowBlur.cpp \
						src/MVFlowFPS.cpp \
						src/MVFlowInter.cpp \
						src/MVFrame.cpp \
						src/MVMask.cpp \
						src/MVRecalculate.cpp \
						src/MVSCDetection.cpp \
						src/MVSuper.cpp \
						src/Overlap.cpp \
						src/PlaneOfBlocks.cpp \
						src/SimpleResize.cpp \
						src/asm/const-a.asm \
						src/asm/cpu-a.asm \
						src/asm/pixel-a.asm \
						src/asm/sad-a.asm \
						src/asm/Interpolation.asm \
						src/asm/MVDegrains.asm \
						src/asm/Overlap.asm \
						src/asm/SAD.asm \
						src/asm/Variance.asm

libmvtools_la_LDFLAGS = -no-undefined -avoid-version $(PLUGINLDFLAGS)

libmvtools_la_LIBADD = $(FFTW3F_LIBS)
