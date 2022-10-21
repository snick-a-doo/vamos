test_sources = [
  'test.cc',
  'test_aerodynamics.cc',
  'test_car.cc',
  'test_circular-buffer.cc',
  'test_controls.cc',
  'test_frame.cc',
  'test_linear-interpolator.cc',
  'test_numeric.cc',
  'test_particle.cc',
  'test_pid.cc',
  'test_rectangle.cc',
  'test_rigid-body.cc',
  'test_road-segment.cc',
  'test_spline.cc',
  'test_strip-track.cc',
  'test_three-matrix.cc',
  'test_three-vector.cc',
  'test_timing-info.cc',
  'test_two-vector.cc',
]

vamos_test = executable('vamos-test',
                        test_sources,
                        include_directories: vamos_includes,
                        link_with: [body_lib, geometry_lib, media_lib, track_lib, world_lib])
test('vamos test', vamos_test)
