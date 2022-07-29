media_sources = [
  'model.cc',
  'sample.cc',
  'texture-image.cc',
  'two-d.cc',
  'xml-parser.cc',
]

media_lib = library('vamos-media',
                    media_sources,
                    dependencies: [al_dep, alut_dep, gl_dep,
                                   glu_dep, glut_dep, png_dep],
                    include_directories: vamos_includes,
                    link_with: geometry_lib,
                    install: true)
