project('vamos', 'cpp',
        version: '1.0.0',
        default_options: ['cpp_std=c++20', 'warning_level=3'],
        license: 'GPL3')

al_dep = dependency('openal', required: true)
alut_dep = dependency('freealut', required: true)
assimp_dep = dependency('assimp', required: true)
gl_dep = dependency('opengl', required: true)
glu_dep = dependency('glu', required: true)
glut_dep = dependency('glut', required: true)
png_dep = dependency('libpng', required: true)
sdl_dep = dependency('sdl2', required: true)
xml_dep = dependency('pugixml', required: true)

# A subdir may include headers from another with <subdir/header.hpp>.  So .. should be
# an include directory.
vamos_includes = include_directories('..')

# Entries must come after libraries that they link with.
subdir('geometry')
subdir('media')
subdir('body')
subdir('track')
subdir('world')

subdir('test')
subdir('caelum')
subdir('vamos')

install_subdir('data', install_dir: get_option('datadir') / 'vamos')
