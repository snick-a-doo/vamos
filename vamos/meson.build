vamos_sources = [
  'options.cc',
  'vamos.cc',
]

vamos_libs = [
  body_lib,
  geometry_lib,
  media_lib,
  track_lib,
  world_lib
]

vamos_app = executable('vamos',
                       vamos_sources,
                       include_directories: vamos_includes,
                       link_with: vamos_libs)
