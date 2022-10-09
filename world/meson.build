world_sources = [
  'controls.cc',
  'gl-world.cc',
  'interactive-driver.cc',
  'pid.cc',
  'robot-driver.cc',
  'sounds.cc',
  'timing-info.cc',
  'world.cc',
]

inc = include_directories('..')

world_lib = library('vamos-world',
                    world_sources,
                    dependencies: [al_dep, alut_dep, gl_dep, glu_dep, glut_dep, sdl_dep, xml_dep],
                    include_directories: vamos_includes,
                    link_with: [body_lib, geometry_lib, media_lib, track_lib],
                    install: true)
