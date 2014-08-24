#!/usr/bin/python
#
# random_circuit: Generate closed tracks for Vamos
#
# Tracks are generated by placing circles with random radii at random
# (non-overlapping) locations.  A path is then found by treating the circles as
# pulleys and threading an imaginary belt around them.
#
# The --show option causes a Clutter window to display the circles and the path
# through them.  Clicking, or hitting a key other than 'q' generates a new one.
# When the window is closed (by typing 'q' or through the window manager) the
# track file is written to /tmp/random-circuit.xml.

import vamos_track
import random
import sys
import math
import copy

def distance (p1, p2):
    return math.sqrt ((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

class Point ():
    def __init__ (self, x, y):
        self.x = x
        self.y = y

class Random_Curve ():
    min_radius = 25

    def __init__ (self):
        self.x = random.uniform (-500, 500)
        self.y = random.uniform (-500, 500)
        self.r = random.uniform (self.min_radius, 150)

        self.start_angle = 0
        self.end_angle = 2*math.pi

        # 1 if CCW, -1 if CW, 0 if not yet decided.
        self.direction = 0

        # True if this curve's circle is entirely within the tangent lines
        # connecting the neighboring circles.
        self.hidden = False

    def center (self):
        return Point (self.x, self.y)

    def distance (self, c):
        return distance (self.center (), c.center ())

    def overlap (self, c):
        # True if this circle overlaps any part of c.
        return self.distance (c) < (self.r + c.r)

    def set_start_angle (self, angle):
        while angle < 0: angle += 2*math.pi
        self.start_angle = angle

    def set_end_angle (self, angle):
        while angle < self.start_angle: angle += 2*math.pi
        self.end_angle = angle

    def _edge_point (self, angle):
        return Point(self.x + self.r*math.cos (angle), self.y + self.r*math.sin (angle))

    def arc_start_point (self):
        return self._edge_point (self.start_angle)

    def arc_end_point (self):
        return self._edge_point (self.end_angle)

    def angle (self):
        delta = self.direction * (self.end_angle - self.start_angle)
        while delta > 2*math.pi: delta -= 2*math.pi
        while delta < 0: delta += 2*math.pi
        return delta

    def arc_length (self):
        return self.r * self.angle ()

    def signed_r (self):
        assert (self.direction != 0)
        return self.r * self.direction

    def reduce_radius (self, point):
        '''Shrink the radius by a random amount keeping the given point fixed.'''
        r = self.r
        self.r = random.uniform (self.min_radius, 0.9*self.r)
        a = self.r/r
        b = 1.0 - a
        self.x = a*self.x + b*point.x
        self.y = a*self.y + b*point.y

def overlap (new_c, curves):
    '''True if new_c overlaps any curve in curves.'''
    for c in curves:
        if new_c.overlap (c):
            return True
    return False

def turn (c1, c2, c3):
    '''Helper function for set_directions() below.'''
    x1 = c1.x - c2.x
    y1 = c1.y - c2.y
    x2 = c3.x - c2.x
    y2 = c3.y - c2.y
    cross = x2*y1 - x1*y2

    a12 = math.atan2 (c2.y - c1.y, c2.x - c1.x)
    a13 = math.atan2 (c3.y - c1.y, c3.x - c1.x)
    da = a13 - a12

    b12 = math.acos ((c1.r - c2.r)/c1.distance (c2))
    b13 = math.acos ((c1.r - c3.r)/c1.distance (c3))

    # True if the tangent angles for 1,2 are both within the tangent angles of
    # 1,3. 
    hidden = b12 + da < b13 and b12 - da < b13
    if cross > 0: 
        return (1, hidden)
    else:
        return (-1, hidden)

def set_directions (curves):
    '''For each circle, determine if the road should go around clockwise or
    counterclockwise.'''
    last_directed = -1
    for i in range (0, len (curves)):
        c1 = curves [(i-1) % len (curves)]
        c2 = curves [i]
        c3 = curves [(i+1) % len (curves)]
        (c2.direction, c2.hidden) = turn (c1, c2, c3)
        if not c2.hidden:
            last_directed = i
    # Should not be possible to have all curves hidden.
    assert (last_directed != -1)
    # Start threading from a non-hidden curve.
    for i in range (last_directed + 1, len (curves) + last_directed):
        c1 = curves [(i-1) % len (curves)]
        c2 = curves [i % len (curves)]
        c3 = curves [(i+1) % len (curves)]
        if c2.hidden and c3.direction == c1.direction:
            c2.direction = -c1.direction
    return curves

def split_curve (c):
    '''Turn one curve into two curves with smaller radii.'''
    c2 = copy.copy (c)
    c.reduce_radius (c.arc_start_point ())
    c2.reduce_radius (c.arc_end_point ())
    return c2

def connect_curves (curves):
    '''Find the endpoints of the straights that connect the curves.'''
    for i in range (0, len (curves)):
        c1 = curves [i]
        c2 = curves [(i+1) % len (curves)]

        angle = math.atan2 (c2.y - c1.y, c2.x - c1.x)
        r = c1.r
        if c2.direction == c1.direction:
            r -= c2.r
        else:
            r += c2.r
        phi = 0

        if abs (r) < c1.distance (c2):
            phi = math.acos (r/c1.distance (c2))

        if c1.direction < 0:
            angle += phi
        else:
            angle -= phi

        c1.set_end_angle (angle)
        if c2.direction != c1.direction:
            angle += math.pi
        c2.set_start_angle (angle)

    split_curves = []
    for c in curves:
        # Remove any nearly full-circle curves.
        if c.angle () < 0.9*2.0*math.pi:
            split_curves.append (c)
            # Turn > 180 curves into double-apex curves.
            if c.angle () > math.pi:
                split_curves.append (split_curve (c))

    return split_curves

class Random_Circuit (vamos_track.vamos_track):
    '''Generates and writes a closed circuit. Overrides vamos_track.build_track().'''
    def __init__ (self):
        '''Pick clockwise or counterclockwise, and generate a list of curves.'''
        vamos_track.vamos_track.__init__ (self, True)
        # 1: increasing angle (counterclockwise), -1: decreasing angle (clockwise)
        self.direction = random.choice ([-1, 1])
        self.curves = set_directions (self.generate_curves (random.randint (6, 10)))
        changed = True
        while changed:
            n_curves = len (self.curves)
            self.curves = connect_curves (self.curves)
            changed = len (self.curves) != n_curves

    def circle_angle (self, c):
        return self.direction * math.atan2 (c.y, c.x)

    def generate_curves (self, corners):
        '''Generate circles with random positions and radii. Discard circles
        that overlap any already generated. Sort the circles by angle from the
        center of the field to center of circle.'''
        curves = []
        for i in range (corners):
            c = Random_Curve ()
            while (overlap (c, curves)):
                c = Random_Curve ()
            curves.append (c)
        curves.sort (key = self.circle_angle)
        return curves

    def build_track (self):
        '''Build the track by adding curves from the start to end points of each
        circle. Add straights between the end and start points of neighboring
        circles.'''
        # Find the longest straight so it can be at the start/finish.
        max_index = 0
        max_d = 0.0
        distances = []
        n = len (self.curves)
        for i in range (n):
            c1 = self.curves [(i-1) % n]            
            c2 = self.curves [i]
            d = distance (c1.arc_end_point (), c2.arc_start_point ())
            if d > max_d:
                max_index = i
                max_d = d
            distances.append (d)

        self.add_segment (distances [max_index]/2, 0)
        self.segments [0].add_camera ([200.0, -20.0, 5.0], 400.0)
        for i in range (len (self.curves) - 1):
            j = (i + max_index) % n
            k = (j + 1) % n
            c1 = self.curves [j]
            c2 = self.curves [k]
            self.add_segment (c1.arc_length (), c1.signed_r ())
            self.add_segment (distances [k], 0)
        end = (max_index - 1) % n
        self.add_segment (self.curves [end].arc_length (), self.curves [end].signed_r ())
        self.add_segment (distances [max_index]/2, 0)

# class Circuit_Viewer (clutter.Stage):
#     def __init__ (self):
#         clutter.Stage.__init__ (self)
#         self.set_size (600, 600)
#         self.set_color (clutter.Color (10, 10, 100))
#         self.connect ('key-press-event', self.handle_event)
#         self.connect ('button-press-event', self.handle_event)
#         self.connect ('destroy', clutter.main_quit)
#         self.track = None
#         self.new_track ()

#     def handle_event (self, stage, event):
#         done = False
#         try:
#             done = (event.keyval == clutter.keysyms.Escape 
#                     or chr (event.keyval) == 'q')
#         except:
#             pass
#         if done:
#             self.write_quit ()
#         else:
#             self.new_track ()

#     def new_track (self):
#         self.remove_all ()
#         self.track = Random_Circuit ()
#         self.track.construct ()
#         self.show_curves ()

#     def write_quit (self):
#         self.track.write_track ('/tmp/random-circuit.xml')
#         clutter.main_quit ()

#     def show_curves (self):
#         canvas = clutter.CairoTexture (int (self.get_width ()), 
#                                        int (self.get_height ()))
#         self.add (canvas)
#         cr = canvas.cairo_create ()
#         cr.translate (self.get_width ()/2, self.get_height ()/2)
#         # Flip y so that +ve y is up.  This makes +ve theta CCW.
#         cr.scale (self.get_width ()/1400, -self.get_height ()/1400)
#         cr.set_line_width (3)
#         # Mark the center.
#         cr.arc (0, 0, 3, 0, 2*math.pi)
#         cr.set_source_rgb (0.5, 0.0, 0.0)
#         cr.fill ()
#         # Draw and connect the circles.
#         cr.move_to (0, 0)
#         for c in self.track.curves:
#             cr.line_to (c.x, c.y)
#             cr.set_source_rgb (0.0, 0.0, 0.5)
#             cr.stroke ()
#             cr.arc (c.x, c.y, c.r, 0, 2*math.pi)
#             cr.set_source_rgb (0.5, 0.5, 0.5)
#             cr.stroke ()
#             cr.move_to (c.x, c.y)
#         cr.line_to (self.track.curves [0].x, self.track.curves [0].y)
#         cr.set_source_rgb (0.2, 0.2, 0.5)
#         cr.stroke ()
#         # Draw the track.
#         cr.set_source_rgb (0.0, 1.0, 0.0)
#         c = self.track.curves [-1]
#         p = c.arc_end_point ()
#         cr.move_to (p.x, p.y)
#         for c in self.track.curves:
#             p = c.arc_start_point ()
#             cr.line_to (p.x, p.y)
#             cr.stroke ()
#             if c.direction < 0:
#                 cr.arc_negative (c.x, c.y, c.r, c.start_angle, c.end_angle)
#             else:
#                 cr.arc (c.x, c.y, c.r, c.start_angle, c.end_angle)
#             cr.stroke ()
#             p = c.arc_end_point ()
#             cr.move_to (p.x, p.y)

if __name__ == '__main__':
    sys.argv = sys.argv [1:]

    show = False
    if len (sys.argv) > 0 and sys.argv [0] == '--show':
        show = True
        sys.argv = sys.argv [1:]

    seed = 0
    if len (sys.argv) > 0:
        seed = int (sys.argv [0])
        sys.argv = sys.argv [1:]
    else:
        seed = random.randrange (0, 2**64)
    print (seed)

    random.seed (seed)

    if show:
        import clutter
        viewer = Circuit_Viewer ()
        viewer.show_all ()
        clutter.main ()
    else:
        track = Random_Circuit ()
        track.construct ()
        track.write_track ('/tmp/random-circuit.xml')
