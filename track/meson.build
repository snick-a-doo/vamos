track_sources = [
  'gl-road-segment.cc',
  'road-segment.cc',
  'strip-track.cc',
  'strip-track-reader.cc',
]

track_lib = library('vamos-track',
                    track_sources,
                    dependencies: [al_dep, gl_dep, xml_dep],
                    include_directories: vamos_includes,
                    link_with: [geometry_lib, media_lib],
                    install: true)
