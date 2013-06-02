## \file Common track-generation code.

import random
import math
import sys

header = '''<?xml version="1.0"?>
<track name="Random">

  <!-- Sky Box -->

  <sky>
    <sides>textures/sky_sides.png</sides>
	<top>textures/sky_top.png</top>
	<bottom>textures/sky_bottom.png</bottom>
	<smooth/>
  </sky>

  <material name="track" type="asphalt">
	<friction>1.0</friction>
	<restitution>0.1</restitution>
	<rolling>1.0</rolling>
	<drag>0.0</drag>
	<bump-amplitude>0.01</bump-amplitude>
	<bump-wavelength>100.0</bump-wavelength>
	<texture>
	  <file>textures/track2.png</file>
	  <length>200.0</length>
	  <smooth/>
	  <mipmap/>
	</texture>
  </material>

  <material name="grass" type="grass">
	<friction>0.7</friction>
	<restitution>0.1</restitution>
	<rolling>5.0</rolling>
	<drag>20.0</drag>	
	<bump-amplitude>0.03</bump-amplitude>
	<bump-wavelength>2.0</bump-wavelength>
	<texture>
	  <file>textures/grass.png</file>
	  <width>10.0</width>
	  <length>12.0</length>
	  <smooth/>
	  <mipmap/>
	</texture>
  </material>

  <material name="gravel" type="gravel">
	<friction>0.8</friction>
	<restitution>0.0</restitution>
	<rolling>40.0</rolling>
	<drag>200.0</drag>	
	<bump-amplitude>0.05</bump-amplitude>
	<bump-wavelength>2.0</bump-wavelength>
	<texture>
	  <file>textures/gravel3.png</file>
	  <width>10.0</width>
	  <length>10.0</length>
	  <smooth/>
	  <mipmap/>
	</texture>
  </material>

  <material name="tires" type="rubber">
	<friction>1.0</friction>
	<restitution>0.8</restitution>
	<rolling>1.0</rolling>
	<drag>5.0</drag>
	<bump-amplitude>0.0</bump-amplitude>
	<bump-wavelength>1.0</bump-wavelength>
    <texture>
	  <file>textures/red-tires.png</file>
	  <width>0.33</width>
	  <length>3.0</length>
	  <smooth/>
	  <mipmap/>
	</texture>
  </material>

  <material name="rail" type="metal">
	<friction>1.0</friction>
	<restitution>0.1</restitution>
	<rolling>1.0</rolling>
	<drag>0.0</drag>	
	<bump-amplitude>0.0</bump-amplitude>
	<bump-wavelength>1.0</bump-wavelength>
    <texture>
	  <file>textures/rail.png</file>
	  <width>0.34</width>
	  <length>10.0</length>
	  <smooth/>
	  <mipmap/>
	</texture>
  </material>

  <material name="wall" type="concrete">
	<friction>1.0</friction>
	<restitution>0.1</restitution>
	<rolling>1.0</rolling>
	<drag>0.0</drag>
	<bump-amplitude>0.0</bump-amplitude>
	<bump-wavelength>1.0</bump-wavelength>
	<texture>
	  <file>textures/wall.png</file>
	  <width>1.0</width>
	  <length>10.0</length>
	  <smooth/>
	  <mipmap/>
	</texture>
  </material>

  <material name="kerb" type="concrete">
	<friction>1.0</friction>
	<restitution>0.1</restitution>
	<rolling>1.0</rolling>
	<drag>0.0</drag>
	<bump-amplitude>0.06</bump-amplitude>
	<bump-wavelength>2.0</bump-wavelength>
	<texture>
	  <file>textures/red-kerb.png</file>
	  <width>2.0</width>
	  <length>4.0</length>
	  <smooth/>
	  <mipmap/>
	</texture>
  </material>

  <!-- Segment Properties -->

  <segment name="straight">
    [ wall grass kerb track kerb grass rail ]
  </segment>
  <segment name="straight pit">
    [ wall track kerb track kerb grass rail ]
  </segment>
  <segment name="left turn">
    [ wall grass kerb track kerb gravel tires ]
  </segment>
  <segment name="left turn pit">
    [ wall track kerb track kerb gravel tires ]
  </segment>
  <segment name="right turn">
    [ tires gravel kerb track kerb grass rail ]
  </segment>

  <!-- The track -->
'''

first_segment_extras = '''    <left-width>[ 0.0, 25.0 ]</left-width>
    <right-width>[ 0.0, 25.0 ]</right-width>
    <left-road-width>[ 0.0, 8.0 ]</left-road-width>
    <right-road-width>[ 0.0, 8.0 ]</right-road-width>
    <left-wall-height>1.0</left-wall-height>
    <right-wall-height>1.0</right-wall-height>
	<right-kerb>
	  <start>
        <distance>0.0</distance>
	    <transition>
		  <length>4.0</length>
		  <width>2.1</width>
		</transition>
	  </start>
	  <end>
        <distance>100.0</distance>
	    <transition>
		  <length>4.0</length>
		  <width>2.1</width>
		</transition>
	  </end>
	  <profile>[ 1.0, 0.1 ][ 2.0, 0.1 ][ 2.1, 0.0 ]</profile>
	</right-kerb>
	<left-kerb>
	  <start>
        <distance>0.0</distance>
	    <transition>
		  <length>4.0</length>
		  <width>2.1</width>
		</transition>
	  </start>
	  <end>
        <distance>100.0</distance>
	    <transition>
		  <distance>10.0</distance>
		  <length>4.0</length>
		  <width>2.1</width>
		</transition>
	  </end>
	  <profile>[ 1.0, 0.1 ][ 2.0, 0.1 ][ 2.1, 0.0 ]</profile>
	</left-kerb>'''

def angle_to_radius (angle, length):
    radius = length / (math.pi * (angle / 180.0))
    return radius
    if radius > 1e-6:
        return max (radius, limit)
    elif radius < -1e-6:
        return min (radius, -limit)
    return 0.0;

def random_in_range (limits):
    return random.uniform (limits [0], limits [1])

class road_segment:
    '''A straight or curved segment of track'''

    # Turns with a radius smaller than this get braking markers, gravel runoff,
    # and tire barriers.
    turn_radius_threshold = 200.0
    turn_angle_threshold = 1

    def __init__ (self, length, radius):
        '''Create a segment with the given properties'''
        self.length = length
        self.radius = radius
        angle = 0
        if radius != 0:
            angle = abs (length/radius)
        self.name = 'straight'
        if (abs (self.radius) < self.turn_radius_threshold
            and angle > self.turn_angle_threshold):
            if self.radius > 0.0:
                self.name = 'left turn'
            elif self.radius < 0.0:
                self.name = 'right turn'
        self.elevation = []
        self.braking_marker_side = None
        self.camera_position = None
        self.camera_range = None

    def add_elevation (self, distance, elevation):
        self.elevation.append ([distance, elevation])

    def add_camera (self, position, camera_range):
        self.camera_position = position
        self.camera_range = camera_range
        # Put the camera on the outside of the turn for greatest visibility of
        # the adjoining straights.
        if self.radius > 0.0:
            self.camera_position[1] *= -1.0

    def add_braking_markers (self, side):
        self.braking_marker_side = side

    def print_segment (self, first, turn = None):
        '''Print the XML for this segment'''
        if turn != None:
            print '  <!-- Turn %d -->' % turn
        print '''  <road segment="%s">
    <resolution>2</resolution>
    <length>%.1f</length>
    <radius>%.1f</radius>''' % (self.name, self.length, self.radius)
        if first:
            print first_segment_extras
        for e in self.elevation:
            print '    <elevation>[ %.1f, %.1f ]</elevation>' % (e[0], e[1])
        if self.braking_marker_side != None:
            for distance in [ 50, 100, 150 ]:
                if distance > self.length:
                    break;
                print '''    <braking-marker>
      <file>textures/%d.png</file>
      <distance>%d.0</distance>
      <size>[ 1.4, 0.7 ]</size>
      <offset>[ 2.0, 0.0 ]</offset>
      <side>%s</side>
    </braking-marker>''' % (distance, distance, self.braking_marker_side)
        if self.camera_position != None:
            print '''    <camera>
      <position>[ %.1f, %.1f, %.1f ]</position>
      <range>%.1f</range>
    </camera>''' % (self.camera_position[0], 
                    self.camera_position[1], 
                    self.camera_position[2],
                    self.camera_range)
        print '  </road>\n'

class vamos_track ():
    elevation_distance_range = [30.0, 400.0]
    elevation_change_range = [-1.0, 1.0]
    elevation_vertical_factor = 0.1
    first_segment_length = 200.0

    def  __init__ (self, close):
        self.close = close

        # Accumulated data
        self.elevation = []
        self.segments = []

        self.length = 0
        self.elevation;
        self.e_start = 0 # The indices of the elevation array
        self.e_end = 0   # to be used for a given segment.
        self.camera_distance = 0.0

    def construct (self):
        self.build_track ()
        self.build_elevation ()
        self.add_braking_markers ()

    def add_braking_markers (self):
        '''Go through the segments in reverse to find straights that precede
        curves.''' 
        last_type = ''
        for s in reversed (self.segments):
            if s.name == 'straight':
                if last_type == 'left turn':
                    s.add_braking_markers ('right')
                elif last_type == 'right turn':
                    s.add_braking_markers ('left')
            last_type = s.name

    def build_elevation (self):
        '''Construct an array of (distance, elevation) points for the entire
        track'''

        # Keep the starting area relatively flat.
        distance = self.first_segment_length
        height = 0.0
        elevation = [[distance/2, height], [distance, height]]
        while distance <= self.length:
            delta_d = random_in_range (self.elevation_distance_range)
            distance += delta_d
            height += (random_in_range (self.elevation_change_range)
                       * self.elevation_vertical_factor 
                       * delta_d)
            elevation.append ([distance, height])

        # For circuits, scale the elevation so it's back to zero at the start
        # line. 
        if self.close:
            e_last = elevation[-2]
            for e in elevation[:-1]:
                e[1] -= e_last[1] * e[0]/e_last[0]
            elevation[-1][1] = elevation[0][1]

        distance = 0.0
        i = 0
        for s in self.segments:
            end_distance = distance + s.length
            while elevation [i][0] < end_distance:
                s.add_elevation (elevation [i][0] - distance, elevation [i][1])
                i += 1
            distance = end_distance

        # Avoid extrapolating the end of an open track.
        if not self.close:
            last = self.segments [-1]
            last.add_elevation (last.length, elevation [-1][1])

    def add_segment (self, length, radius):
        s = road_segment (length, radius)
        self.camera_distance += length/2.0

        # On each turn, add a camera halfway through. Switch to that camera
        # halfway between this camera and the last camera. 
        if s.radius != 0.0:
            s.add_camera ([length/2.0, 20.0, 5.0], self.camera_distance/2.0)
            self.camera_distance = 0.0
        self.camera_distance += length/2.0

        self.segments.append (s)
        self.e_start = self.e_end
        self.length += length

    def write_track (self, output_file):
        out = open (output_file, 'w')
        sys.stdout = out
        print header
        first = True
        turn = 0
        for s in self.segments:
            if s.name != 'straight':
                turn += 1
                s.print_segment (first, turn)
            else:
                s.print_segment (first)
            first = False
        if self.close:
            print '  <circuit/>'
            print '  <timing-line>%d</timing-line>' % 10
            print '  <timing-line>%d</timing-line>' % (self.length/3)
            print '  <timing-line>%d</timing-line>' % (2*self.length/3)
        else:
            for s in range (1000, int (self.length), 1000):
                print '  <timing-line>%d</timing-line>' % s
        print '</track>'
        out.close ()
