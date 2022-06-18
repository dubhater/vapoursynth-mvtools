warningflags = -Wall -Wextra -Wshadow
commonflags = -O2 $(MFLAGS) $(STACKREALIGN) $(warningflags) -fvisibility=hidden
AM_CFLAGS = -std=c99 $(commonflags)
AM_CXXFLAGS = -std=c++11 $(commonflags)

AM_CPPFLAGS = $(VapourSynth_CFLAGS) $(FFTW3F_CFLAGS)

lib_LTLIBRARIES = libmvtools.la

AM_V_NASM = $(am__v_NASM_$(V))
am__v_NASM_ = $(am__v_NASM_$(AM_DEFAULT_VERBOSITY))
am__v_NASM_0 = @echo "  NASM    " $@;
am__v_NASM_1 =

.asm.lo:
	$(AM_V_NASM)$(LIBTOOL) $(AM_V_lt) --mode=compile --tag=CC $(NASM) $(NASMFLAGS) -o $@ $< -prefer-non-pic

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
						src/MVFrame.cpp \
						src/MVFrame.h \
						src/MVMask.c \
						src/MVRecalculate.c \
						src/MVSCDetection.c \
						src/MVSuper.c \
						src/Overlap.cpp \
						src/Overlap.h \
						src/PlaneOfBlocks.cpp \
						src/PlaneOfBlocks.h \
						src/SADFunctions.cpp \
						src/SADFunctions.h \
						src/SimpleResize.cpp \
						src/SimpleResize.h

libmvtools_la_LDFLAGS = -no-undefined -avoid-version $(PLUGINLDFLAGS)

libmvtools_la_LIBADD = $(FFTW3F_LIBS)

if MVTOOLS_ARM
libmvtools_la_SOURCES += src/asm/aarch64-pixel-a.S
endif

if MVTOOLS_X86
libmvtools_la_SOURCES += src/asm/const-a.asm \
						 src/asm/cpu-a.asm \
						 src/asm/pixel-a.asm \
						 src/asm/sad-a.asm

if MVTOOLS_I686
libmvtools_la_SOURCES += src/asm/pixel-32.asm
endif

noinst_LTLIBRARIES = libavx2.la

libavx2_la_SOURCES = src/MaskFun_AVX2.cpp \
					 src/MVDegrains_AVX2.cpp \
					 src/MVFrame_AVX2.cpp \
					 src/Overlap_AVX2.cpp \
					 src/SADFunctions_AVX2.cpp \
					 src/SimpleResize_AVX2.cpp

libavx2_la_CXXFLAGS = $(AM_CXXFLAGS) -mavx2 -mtune=haswell

libmvtools_la_LIBADD += libavx2.la
endif
