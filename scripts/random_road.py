#!/usr/bin/python

import vamos_track
import random
import sys


class random_road (vamos_track.vamos_track):
    '''A track made from segments with random lengths, radii, and elevations'''

    # Adjustables
    length_range = [15.0, 350.0]
    angle_range = [-140.0, 140.0]

    def __init__ (self, total_length):
        vamos_track.vamos_track.__init__ (self, False)
        self.straight_weight = 2 # Curve and straight are equally likely  
        self.total_length = total_length

    def long_enough (self):
        return self.length >= self.total_length

    def build_track (self):
        self.add_segment (self.first_segment_length, 0.0) 
        last_radius = 0.0
        while not self.long_enough ():
            length = vamos_track.random_in_range (self.length_range)
            radius = 0.0
            type = random.randint (1, self.straight_weight);
            if type == 1: # curve
                angle = vamos_track.random_in_range (self.angle_range)
                radius = vamos_track.angle_to_radius (angle, length)
                self.straight_weight *= 3 # Straight will be more likely next
                if last_radius * radius > 0.0:
                    self.add_segment (20.0, 0.0)
            else: # straight
                self.straight_weight = 1 # Next segment will be a curve
            self.add_segment (length, radius)
            last_radius = radius

seed = 0
if len (sys.argv) == 2:
    seed = int (sys.argv [1])
else:
    seed = random.randint (0, sys.maxint)
print seed
random.seed (seed)

track = random_road (10000)
track.construct ()
track.write_track ('/tmp/random-road.xml')
