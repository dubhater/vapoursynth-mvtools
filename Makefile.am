warningflags = -Wall -Wextra -Wshadow
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

libmvtools_la_SOURCES =	src/CopyCode.c \
						src/CPU.c \
						src/DCTFFTW.cpp \
						src/EntryPoint.c \
						src/Fakery.c \
						src/GroupOfPlanes.c \
						src/Luma.c \
						src/MaskFun.c \
						src/MVAnalyse.c \
						src/MVAnalysisData.c \
						src/MVBlockFPS.c \
						src/MVCompensate.c \
						src/MVDegrains.cpp \
						src/MVFinest.c \
						src/MVFlowBlur.c \
						src/MVFlowFPS.c \
						src/MVFlowFPSHelper.c \
						src/MVFlowInter.c \
						src/MVFrame.c \
						src/MVMask.c \
						src/MVRecalculate.c \
						src/MVSCDetection.c \
						src/MVSuper.c \
						src/Overlap.c \
						src/PlaneOfBlocks.c \
						src/SADFunctions.cpp \
						src/SimpleResize.c \
						src/asm/const-a.asm \
						src/asm/cpu-a.asm \
						src/asm/pixel-a.asm \
						src/asm/sad-a.asm \
						src/asm/Interpolation.asm \
						src/asm/Luma.asm \
						src/asm/MVDegrains.asm \
						src/asm/Overlap.asm \
						src/asm/SAD.asm

libmvtools_la_LDFLAGS = -no-undefined -avoid-version $(PLUGINLDFLAGS)

libmvtools_la_LIBADD = $(FFTW3F_LIBS)
