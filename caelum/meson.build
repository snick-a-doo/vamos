caelum_sources = [
  'caelum.cc',
  'sky.cc',
]

caelum_app = executable('caelum',
                        caelum_sources,
                        dependencies: [gl_dep, glu_dep, sdl_dep],
                        include_directories: vamos_includes,
                        link_with: media_lib)
