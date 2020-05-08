project('MVTools', 'c', 'cpp',
        version: '23',
        default_options: ['c_std=c99', 'cpp_std=c++11', 'buildtype=release'],
        meson_version: '>=0.46')


warnings = [
  '-Wall',
  '-Wextra',
  '-Wshadow',
]

cflags = [
  warnings,
  '-fvisibility=hidden',
  '-DPACKAGE_VERSION="@0@"'.format(meson.project_version()),
]

ldflags = [
]

nasm_flags = [
  '-I@0@'.format(join_paths(meson.current_source_dir(), 'src/asm/include/')),
  '-w',
  '-Worphan-labels',
  '-Wunrecognized-char',
  '-Dprivate_prefix=mvtools',
  '-DHIGH_BIT_DEPTH=0',
  '-DBIT_DEPTH=8',
]


vapoursynth_dep = dependency('vapoursynth').partial_dependency(includes: true, compile_args: true)


sources = [
  'src/CopyCode.cpp',
  'src/CPU.c',
  'src/DCTFFTW.cpp',
  'src/EntryPoint.c',
  'src/Fakery.c',
  'src/GroupOfPlanes.c',
  'src/Luma.cpp',
  'src/MaskFun.cpp',
  'src/MVAnalyse.c',
  'src/MVAnalysisData.c',
  'src/MVBlockFPS.c',
  'src/MVCompensate.c',
  'src/MVDegrains.cpp',
  'src/MVDepan.cpp',
  'src/MVFinest.c',
  'src/MVFlow.cpp',
  'src/MVFlowBlur.c',
  'src/MVFlowFPS.c',
  'src/MVFlowFPSHelper.c',
  'src/MVFlowInter.c',
  'src/MVFrame.cpp',
  'src/MVMask.c',
  'src/MVRecalculate.c',
  'src/MVSCDetection.c',
  'src/MVSuper.c',
  'src/Overlap.cpp',
  'src/PlaneOfBlocks.cpp',
  'src/SADFunctions.cpp',
  'src/SimpleResize.cpp',
]


debug_build = get_option('buildtype').startswith('debug')


host_cpu_family = host_machine.cpu_family()

if host_cpu_family == 'x86'
  host_x86_bits = 32
elif host_cpu_family == 'x86_64'
  host_x86_bits = 64
endif


host_system = host_machine.system()

if host_system == 'windows' or host_system == 'cygwin'
  if host_cpu_family == 'x86'
    cflags += '-mstackrealign'
    ldflags += '-Wl,--kill-at'
    nasm_flags += '-DPREFIX'
  endif

  nasm_flags += ['-f', 'win@0@'.format(host_x86_bits)]
elif host_system == 'linux' or host_system == 'bsd' # The BSDs are close enough, right?
  if debug_build
    nasm_flags += '-gdwarf'
  endif

  nasm_flags += ['-f', 'elf@0@'.format(host_x86_bits)]
elif host_system == 'darwin'
  if debug_build
    nasm_flags += '-gdwarf'
  endif

  nasm_flags += ['-DPREFIX', '-f', 'macho@0@'.format(host_x86_bits)]
else
  error('Unknown host system "@0@".'.format(host_system))
endif


helper_libs = []


if host_cpu_family.startswith('x86')
  cflags += ['-mfpmath=sse', '-msse2', '-DMVTOOLS_X86=1']


  nasm_sources = [
    'src/asm/const-a.asm',
    'src/asm/cpu-a.asm',
    'src/asm/pixel-a.asm',
    'src/asm/sad-a.asm',
  ]


  if host_cpu_family == 'x86'
    nasm_flags += '-DARCH_X86_64=0'

    nasm_sources += [
      'src/asm/pixel-32.asm',
    ]
  else
    nasm_flags += ['-DARCH_X86_64=1', '-DPIC']
  endif

  nasm = find_program(get_option('with_nasm'))

  outputname = '@BASENAME@.o'
  if host_system == 'windows'
    outputname = '@BASENAME@.obj'
  endif

  nasm_gen = generator(nasm,
                       output: outputname,
                       arguments: nasm_flags + ['@INPUT@', '-o', '@OUTPUT@'])

  sources += nasm_gen.process(nasm_sources)


  libavx2_sources = [
    'src/MaskFun_AVX2.cpp',
    'src/MVDegrains_AVX2.cpp',
    'src/MVFrame_AVX2.cpp',
    'src/Overlap_AVX2.cpp',
    'src/SADFunctions_AVX2.cpp',
    'src/SimpleResize_AVX2.cpp',
  ]

  helper_libs += static_library('avx2',
                                libavx2_sources,
                                dependencies: vapoursynth_dep,
                                cpp_args: [cflags, '-mavx2', '-mtune=haswell'],
                                install: false)
endif


cxx = meson.get_compiler('cpp')


deps = [
  vapoursynth_dep,
  dependency('fftw3f'),
  cxx.find_library('m', required: false),
]

shared_module('mvtools',
              sources,
              dependencies: deps,
              link_args: ldflags,
              c_args: cflags,
              cpp_args: cflags,
              link_with: helper_libs,
              install: true)
