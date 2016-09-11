warningflags = -Wall -Wextra -Wshadow
commonflags = -O2 $(MFLAGS) $(STACKREALIGN) $(warningflags)
AM_CFLAGS = -std=c99 $(commonflags)
AM_CXXFLAGS = -std=c++11 $(commonflags)

AM_CPPFLAGS = $(VapourSynth_CFLAGS) $(FFTW3F_CFLAGS)

lib_LTLIBRARIES = libmvtools.la

AM_V_YASM = $(am__v_YASM_$(V))
am__v_YASM_ = $(am__v_YASM_$(AM_DEFAULT_VERBOSITY))
am__v_YASM_0 = @echo "  YASM    " $@;
am__v_YASM_1 =

.asm.lo:
	$(AM_V_YASM)$(LIBTOOL) $(AM_V_lt) --mode=compile --tag=CC $(YASM) $(YASMFLAGS) -o $@ $< -prefer-non-pic

libmvtools_la_SOURCES =	src/Bullshit.h \
						src/CommonFunctions.h \
						src/CopyCode.cpp \
						src/CopyCode.h \
						src/CPU.c \
						src/CPU.h \
						src/DCTFFTW.cpp \
						src/DCTFFTW.h \
						src/EntryPoint.c \
						src/Fakery.c \
						src/Fakery.h \
						src/GroupOfPlanes.c \
						src/GroupOfPlanes.h \
						src/Interpolation.h \
						src/Luma.cpp \
						src/Luma.h \
						src/MaskFun.cpp \
						src/MaskFun.h \
						src/MVAnalyse.c \
						src/MVAnalysisData.c \
						src/MVAnalysisData.h \
						src/MVBlockFPS.c \
						src/MVCompensate.c \
						src/MVDegrains.cpp \
						src/MVDegrains.h \
						src/MVDepan.cpp \
						src/MVFinest.c \
						src/MVFlow.cpp \
						src/MVFlowBlur.c \
						src/MVFlowFPS.c \
						src/MVFlowFPSHelper.c \
						src/MVFlowFPSHelper.h \
						src/MVFlowInter.c \
						src/MVFrame.c \
						src/MVFrame.h \
						src/MVMask.c \
						src/MVRecalculate.c \
						src/MVSCDetection.c \
						src/MVSuper.c \
						src/Overlap.cpp \
						src/Overlap.h \
						src/PlaneOfBlocks.c \
						src/PlaneOfBlocks.h \
						src/SADFunctions.cpp \
						src/SADFunctions.h \
						src/SimpleResize.cpp \
						src/SimpleResize.h

libmvtools_la_LDFLAGS = -no-undefined -avoid-version $(PLUGINLDFLAGS)

libmvtools_la_LIBADD = $(FFTW3F_LIBS)

if MVTOOLS_X86
libmvtools_la_SOURCES += src/asm/const-a.asm \
						 src/asm/cpu-a.asm \
						 src/asm/pixel-a.asm \
						 src/asm/sad-a.asm \
						 src/asm/Interpolation.asm \
						 src/asm/MVDegrains.asm

noinst_LTLIBRARIES = libavx2.la

libavx2_la_SOURCES = src/MaskFun_AVX2.cpp \
					 src/SimpleResize_AVX2.cpp

libavx2_la_CXXFLAGS = $(AM_CXXFLAGS) -mavx2

libmvtools_la_LIBADD += libavx2.la
endif
