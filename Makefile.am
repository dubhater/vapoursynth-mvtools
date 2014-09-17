AM_CXXFLAGS = -Wall -Wextra -Wno-unused-parameter

AM_CPPFLAGS = $(VapourSynth_CFLAGS) $(FFTW3F_CFLAGS)

lib_LTLIBRARIES = libmvtools.la

libmvtools_la_SOURCES = src/asm-placeholder.cpp \
						src/CopyCode.cpp \
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
						src/MVSuper.cpp \
						src/Overlap.cpp \
						src/Padding.cpp \
						src/PlaneOfBlocks.cpp

libmvtools_la_LDFLAGS = -no-undefined -avoid-version

libmvtools_la_LIBADD = $(FFTW3F_LIBS)
