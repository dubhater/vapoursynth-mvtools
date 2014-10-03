Description
===========

MVTools is a set of filters for motion estimation and compensation.

This is a port of version 2.5.11.3 of the Avisynth plugin (the latest from http://avisynth.org.ru/mvtools/mvtools2.html).


Differences
===========

* All:
    * Free multithreading, courtesy of VapourSynth.

    * Parameters are all lowercase now.

    * YUY2 is not supported, only YV12.

    * The audio is definitely not killed.

    * No "planar" parameter.

* Analyse:
    * No "temporal" parameter, as it's sort of incompatible with multithreading.

    * No "outfile" parameter.

    * No "dct" parameter, because fftw3 likes to crash, and the special code for 8x8 blocks is a pile of inline and standalone asm that may take a while to decipher.

    * No "sadx264" parameter. If isse is True, the best functions imported from x264 will be selected automatically. Otherwise, only C functions will be used.

    * New parameters "fields" and "tff".

    * The optimised SAD, SATD, and SSD functions from x264 have been updated to the latest versions (as of September 2014).

* Recalculate:
    * Same as Analyse.

* Compensate:
    * No "recursion" parameter. It was dodgy.

    * New parameter "tff".

* Mask:
    * No "isse" parameter, because there is no asm in Mask anymore.


Usage
=====
::

    mv.Super(clip clip[, int hpad=8, int vpad=8, int pel=2, int levels=0, bint chroma=True, int sharp=2, int rfilter=2, clip pelclip=None, bint isse=True])

    mv.Analyse(clip super[, int blksize=8, int blksizev=blksize, int levels=0, int search=4, int searchparam=2, int pelsearch=0, bint isb=False, int lambda, bint chroma=True, int delta=1, bint truemotion=True, int lsad, int plevel, int global, int pnew, int pzero=pnew, int pglobal=0, int overlap=0, int overlapv=overlap, bint divide=False, int badsad=10000, int badrange=24, bint isse=True, bint meander=True, bint trymany=False, bint fields=False, bint tff])

    mv.Recalculate(clip super, clip vectors[, int blksize=8, int blksizev=blksize, int search=4, int searchparam=2, int lambda, bint chroma=True, bint truemotion=True, int pnew, int overlap=0, int overlapv=overlap, bint divide=False, bint isse=True, bint meander=True, bint fields=False, bint tff])

    mv.Compensate(clip clip, clip super, clip vectors[, int scbehavior=1, int thsad=10000, bint fields=False, int thscd1=400, int thscd2=130, bint isse=True, bint tff])

    mv.Degrain1(clip clip, clip super, clip mvbw, clip mvfw[, int thsad=400, int thsadc=thsad, int plane=4, int limit=255, int limitc=limit, int thscd1=400, int thscd2=130, bint isse=True])

    mv.Degrain2(clip clip, clip super, clip mvbw, clip mvfw, clip mvbw2, clip mvfw2[, int thsad=400, int thsadc=thsad, int plane=4, int limit=255, int limitc=limit, int thscd1=400, int thscd2=130, bint isse=True])

    mv.Degrain3(clip clip, clip super, clip mvbw, clip mvfw, clip mvbw2, clip mvfw2, clip mvbw3, clip mvfw3[, int thsad=400, int thsadc=thsad, int plane=4, int limit=255, int limitc=limit, int thscd1=400, int thscd2=130, bint isse=True])

    mv.Mask(clip clip, clip vectors[, float ml=100.0, float gamma=1.0, int kind=0, int ysc=0, int thscd1=400, int thscd2=130])

    mv.Finest(clip super[, bint isse=True])

    mv.FlowBlur(clip clip, clip super, clip mvbw, clip mvfw[, float blur=50.0, int prec=1, int thscd1=400, int thscd2=130, bint isse=True])

    mv.FlowInter(clip clip, clip super, clip mvbw, clip mvfw[, float time=50.0, float ml=100.0, bint blend=True, int thscd1=400, int thscd2=130, bint isse=True])


If *fields* is True, it is assumed that the clip named *clip* first went through std.SeparateFields.

For information about the other parameters, consult the Avisynth plugin's documentation at http://avisynth.org.ru/mvtools/mvtools2.html. This will not be necessary in the future.


Things that may happen soon™
============================

  * Support for any subsampling

  * Support for up to 16 bits per sample

  * Possibly lower memory usage


Compilation
===========

::

   ./autogen.sh
   ./configure
   make


License
=======

GPL 2, like the Avisynth plugin.
