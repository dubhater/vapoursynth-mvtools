Description
===========

MVTools is a set of filters for motion estimation and compensation.

This is a port of version 2.5.11.20 of the Avisynth plugin.

Some changes from version 2.5.11.9 of the SVP fork have been incorporated as well (http://www.svp-team.com/wiki/Download).

The filter DepanEstimate was ported from the Avisynth plugin DepanEstimate, version 1.10.

The filters DepanCompensate and DepanStabilise were ported from the Avisynth plugin Depan, version 1.13.1.


Differences
===========

* All:
    * Free multithreading, courtesy of VapourSynth.

    * Parameters are all lowercase now.

    * YUY2 is not supported.

    * Grayscale, 4:2:0, 4:2:2, 4:4:0, and 4:4:4 are supported, except for DepanCompensate and DepanStabilise, which don't support 4:4:0.

    * Up to 16 bits per sample are supported.

    * The audio is definitely not killed.

    * No "planar" parameter.

    * "isse" parameter renamed to "opt".

* Analyse:
    * No "temporal" parameter, as it's sort of incompatible with multithreading.

    * No "outfile" parameter.

    * No "sadx264" parameter. If opt is True, the best functions imported from x264 will be selected automatically. Otherwise, only C functions will be used.

    * New parameters "fields" and "tff".

    * The optimised SAD, SATD, and SSD functions from x264 have been updated to the latest versions (as of September 2014).

    * Block sizes of 64x32, 64x64, 128x64, and 128x128 are supported.

    * The "dct" parameter can be 5..10 even with blocks larger than 16x16.

* Recalculate:
    * Same as Analyse.

* Compensate:
    * No "recursion" parameter. It was dodgy.

    * New parameter "tff".

* Flow
    * New parameter "tff".

* SCDetection:
    * No "ysc" parameter. The input frames are returned unchanged, with the ``_SceneChangePrev`` or ``_SceneChangeNext`` property attached.

    * No "isse" parameter. It wasn't used.

* DepanAnalyse:
    * Formerly "MDepan".

    * New parameters "fields" and "tff".

    * No "log", "range", "isse" parameters.

* DepanEstimate:
    * New parameters "fields" and "tff".

    * No "range", "log", "debug", "extlog" parameters.

* DepanCompensate:
    * Formerly "DePan".

    * No "inputlog" parameter.

* DepanStabilise:
    * Formerly "DePanStabilize".

    * No "inputlog" parameter.

    * Methods -1 and 2 unavailable.


Usage
=====
::

    mv.Super(clip clip[, int hpad=16, int vpad=16, int pel=2, int levels=0, bint chroma=True, int sharp=2, int rfilter=2, clip pelclip=None, bint opt=True])

    mv.Analyse(clip super[, int blksize=8, int blksizev=blksize, int levels=0, int search=4, int searchparam=2, int pelsearch=0, bint isb=False, int lambda, bint chroma=True, int delta=1, bint truemotion=True, int lsad, int plevel, int global, int pnew, int pzero=pnew, int pglobal=0, int overlap=0, int overlapv=overlap, bint divide=False, int badsad=10000, int badrange=24, bint opt=True, bint meander=True, bint trymany=False, bint fields=False, bint tff, int search_coarse=3, int dct=0])

    mv.Recalculate(clip super, clip vectors[, int blksize=8, int blksizev=blksize, int search=4, int searchparam=2, int lambda, bint chroma=True, bint truemotion=True, int pnew, int overlap=0, int overlapv=overlap, bint divide=False, bint opt=True, bint meander=True, bint fields=False, bint tff, int dct=0])

    mv.Compensate(clip clip, clip super, clip vectors[, int scbehavior=1, int thsad=10000, bint fields=False, float time=100.0, int thscd1=400, int thscd2=130, bint opt=True, bint tff])

    mv.Degrain1(clip clip, clip super, clip mvbw, clip mvfw[, int thsad=400, int thsadc=thsad, int plane=4, int limit=255, int limitc=limit, int thscd1=400, int thscd2=130, bint opt=True])

    mv.Degrain2(clip clip, clip super, clip mvbw, clip mvfw, clip mvbw2, clip mvfw2[, int thsad=400, int thsadc=thsad, int plane=4, int limit=255, int limitc=limit, int thscd1=400, int thscd2=130, bint opt=True])

    mv.Degrain3(clip clip, clip super, clip mvbw, clip mvfw, clip mvbw2, clip mvfw2, clip mvbw3, clip mvfw3[, int thsad=400, int thsadc=thsad, int plane=4, int limit=255, int limitc=limit, int thscd1=400, int thscd2=130, bint opt=True])

    mv.Mask(clip clip, clip vectors[, float ml=100.0, float gamma=1.0, int kind=0, float time=100.0, int ysc=0, int thscd1=400, int thscd2=130, bint opt=True])

    mv.Finest(clip super[, bint opt=True])

    mv.Flow(clip clip, clip super, clip vectors[, float time=100.0, int mode=0, bint fields=False, int thscd1=400, int thscd2=130, bint opt=True, bint tff])

    mv.FlowBlur(clip clip, clip super, clip mvbw, clip mvfw[, float blur=50.0, int prec=1, int thscd1=400, int thscd2=130, bint opt=True])

    mv.FlowInter(clip clip, clip super, clip mvbw, clip mvfw[, float time=50.0, float ml=100.0, bint blend=True, int thscd1=400, int thscd2=130, bint opt=True])

    mv.FlowFPS(clip clip, clip super, clip mvbw, clip mvfw[, int num=25, int den=1, int mask=2, float ml=100.0, bint blend=True, int thscd1=400, int thscd2=130, bint opt=True])

    mv.BlockFPS(clip clip, clip super, clip mvbw, clip mvfw[, int num=25, int den=1, int mode=3, float ml=100.0, bint blend=True, int thscd1=400, int thscd2=130, bint opt=True])

    mv.SCDetection(clip clip, clip vectors[, int thscd1=400, int thscd2=130])

    mv.DepanAnalyse(clip clip, clip vectors[, clip mask, bint zoom=True, bint rot=True, float pixaspect=1.0, float error=15.0, bint info=False, float wrong=10.0, float zerow=0.05, int thscd1=400, int thscd2=130, bint fields=False, bint tff])

    mv.DepanEstimate(clip clip[, float trust=4.0, int winx=0, int winy=0, int wleft=-1, int wtop=-1, int dxmax=-1, int dymax=-1, float zoommax=1.0, float stab=1.0, float pixaspect=1.0, bint info=False, bint show=False, bint fields=False, bint tff])

    mv.DepanCompensate(clip clip, clip data[, float offset=0.0, int subpixel=2, float pixaspect=1.0, bint matchfields=True, int mirror=0, int blur=0, bint info=False, bint fields=False, bint tff])

    mv.DepanStabilise(clip clip, clip data[, float cutoff=1.0, float damping=0.9, float initzoom=1.0, bint addzoom=False, int prev=0, int next=0, int mirror=0, int blur=0, float dxmax=60.0, float dymax=30.0, float zoommax=1.05, float rotmax=1.0, int subpixel=2, float pixaspect=1.0, int fitlast=0, float tzoom=3.0, bint info=False, int method=0, bint fields=False])


If *fields* is True, it is assumed that the clip named *clip* first went through std.SeparateFields.

For information about the other parameters, consult the Avisynth plugins' documentation at http://avisynth.org.ru/mvtools/mvtools2.html or http://www.avisynth.nl/users/fizick/depan/depan.html. This will not be necessary in the future.


Compilation
===========

FFTW3 configured for 32 bit floats is required ("fftw3f").

::

   mkdir build; cd build
   meson ../
   ninja

Or

::

   ./autogen.sh
   ./configure
   make

Meson runs faster than autogen.sh and configure.


License
=======

GPL 2, like the Avisynth plugins.
