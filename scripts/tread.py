#! /usr/bin/env python
# Tread - a track editor for Vamos Automotive Simulator
# Copyright (C) 2010  Sam Varner
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# TBD:
# pan and zoom
# - can't pan
# selection
# - multiple seletion
# join straight segments
# - put straight between 2 selected circles
# set start/finish
# set timing lines
# set track length (scale)
# read XML files
# write XML files

import clutter
import gtk # only used for key names.  Can we avoid the dependency?

import geometry
from geometry import Point
from geometry import Circle

import unittest

class Track_Editor (clutter.Stage):
    def __init__ (self, background_image_file):
        clutter.Stage.__init__ (self)
        track = clutter.cogl.texture_new_from_file (background_image_file,
                                                    clutter.cogl.TEXTURE_NO_SLICING,
                                                    clutter.cogl.PIXEL_FORMAT_ANY)
        self.image_width = track.get_width ()
        self.image_height = track.get_height ()
        # Zoom in on this point.
        self.center = Point (self.image_width/2, self.image_height/2)
        self.set_size (track.get_width (), track.get_height ())
        self.background = clutter.Texture ()
        self.background.set_cogl_texture (track)
        self.add (self.background)
        self.set_reactive (True)
        self.connect ('button-press-event', self.press)
        self.connect ('button-release-event', self.release)
        self.connect ('key-press-event', self.key)
        self.connect ('motion-event', self.move)
        self.scale = 1.0
        self.scale_factor = 1.1
        self.pan_step = 10
        self.selected_child = None

    def active_children (self):
        return [self.get_nth_child (i) for i in range (self.get_n_children ()) [-1:0:-1]]

    def press (self, stage, event):
        for c in self.active_children ():
            if c.try_press (Point (event.x, event.y)):
                self.selected_child = c
                break
        if not self.selected_child:
            layer = Curve (self.image_width,
                           self.image_height,
                           self.background,
                           Point (event.x, event.y))
            self.add (layer)

    def release (self, stage, event):
        self.selected_child = None
        for c in self.active_children ():
            c.try_release ()

    def move (self, stage, event):
        moved = False
        for c in self.active_children ():
            if moved:
                c.deselect ()
            elif c.try_move (Point (event.x, event.y), self.selected_child == None):
                moved = True

    def key (self, stage, event):
        name = gtk.gdk.keyval_name (event.keyval)
        if name == 'equal':
            self.zoom (self.scale_factor)
        elif name == 'minus':
            self.zoom (1.0 / self.scale_factor)
        elif name == 'Right':
            self.pan (-self.pan_step, 0)
        elif name == 'Left':
            self.pan (self.pan_step, 0)
        elif name == 'Up':
            self.pan (0, self.pan_step)
        elif name == 'Down':
            self.pan (0, -self.pan_step)
        elif name == 'q':
            clutter.main_quit ()
        elif name == 'Delete':
            for c in self.active_children ():
                if c.is_selected ():
                    self.remove (c)
                    break

    def pan (self, dx, dy):
        print self.center.x
        self.center -= Point (dx, dy)
        self.background.move_by (dx, dy)

    def zoom (self, factor):
        self.scale *= factor
        self.background.set_scale_full (self.scale, self.scale,
                                        self.center.x, self.center.y)
        for c in self.active_children ():
            c.zoom ()

class Curve (clutter.CairoTexture):
    def __init__ (self, width, height, background, center):
        clutter.CairoTexture.__init__ (self, width, height)
        self.set_reactive (True)
        self.pressed = True
        self.selected = False
        self.background = background
        center = self.untransform (center)
        self.current_circle = Circle (center, center)
        self.pointer = Point (0, 0)
        self.pressed_delta = Point (0, 0)
        self.line_width = 1
        self.target_size = 5
        self.selection = 'none'
        self.colors = { 'none': (0, 0, 0),
                        'center': (1, 0, 0),
                        'edge': (0, 1, 0),
                        'circle': (0, 0, 1) }

    def try_press (self, p):
        p = self.untransform (p)
        self.pressed = self.is_selected ()
        self.pressed_delta = p - self.current_circle.center
        return self.pressed

    def try_release (self):
        self.pressed = False
        return True

    def try_move (self, p, select):
        '''Respond to motion events handled by the stage.'''
        self.pointer = p
        p = self.untransform (self.pointer)
        if self.pressed:
            if self.selection == 'edge':
                self.move_edge (p)
            elif self.selection == 'circle':
                self.move_circle (p)
            else:
                self.selection = 'center'
                self.move_center (p)
            return True
        elif select:
            return self.try_select ()

    def try_select (self):
        p = self.untransform (self.pointer)
        d = self.current_circle.distance (p)
        if d < self.target_size:
            self.select ('center')
            return True
        elif self.current_circle.edge_distance (p) < self.target_size:
            self.select ('edge')
            return True
        elif abs (d - self.current_circle.radius ()) < self.target_size / 2:
            self.select ('circle')
            return True
        else:
            self.deselect ()
            return False

    def is_selected (self):
        return self.selection != 'none'

    def select (self, state):
        if self.selection == state: return
        self.selection = state
        self.draw ()

    def deselect (self):
        self.select ('none')

    def move_circle (self, p):
        self.current_circle.move_to (p - self.pressed_delta)
        self.draw ()
        return True

    def move_center (self, p):
        self.current_circle.set_center (p)
        self.draw ()
        return True

    def move_edge (self, p):
        self.current_circle.set_edge (p)
        self.draw ()
        return True

    def zoom (self):
        self.try_select ()
        self.draw ()

    def transform (self, p):
        v = self.background.apply_transform_to_point (clutter.Vertex (p.x, p.y, 0))
        return Point (v.x, v.y)

    def untransform (self, p):
        (x, y) = self.background.transform_stage_point (p.x, p.y)
        return Point (x, y)

    def draw (self):
        self.clear ()
        cr = self.cairo_create ()
        cr.set_line_width (self.line_width)
        (r, g, b) = self.colors [ self.selection ]
        cr.set_source_rgb (r, g, b)
        c = self.current_circle
        center = self.transform (c.center)
        edge = self.transform (c.edge)
        r = geometry.distance (center, edge)
        # center target
        cr.arc (center.x, center.y, r, 0, geometry.full_circle)
        cr.stroke ()
        # circle
        cr.move_to (center.x - self.target_size, center.y)
        cr.line_to (center.x + self.target_size, center.y)
        cr.move_to (center.x, center.y - self.target_size)
        cr.line_to (center.x, center.y + self.target_size)
        cr.stroke ()
        # edge target
        cr.arc (edge.x, edge.y, self.target_size, 0, geometry.full_circle)
        cr.stroke ()

class Mock_Actor (clutter.Actor):
    def __init__ (self, width, height):
        self.set_size (width, height)

class Test_Curve (unittest.TestCase):
    def setUp (self):
        t = clutter.Texture ()
        t.set_size (100, 200)
        self.c = Curve (100, 200, t, Point (50, 100))

    def test_circle (self):
        c = self.c.current_circle
        self.assertEqual (c.center, Point (50, 100))
        self.assertEqual (c.edge, Point (50, 100))

curve_suite = unittest.TestLoader().loadTestsFromTestCase (Test_Curve)
suite = unittest.TestSuite ([geometry.suite, curve_suite])
unittest.TextTestRunner ().run (suite)

stage = Track_Editor ('/home/samv/programs/Vamos/Reference/Silverstone/Silverstone-small.png')
stage.show_all ()
stage.connect ('destroy', clutter.main_quit)

# main clutter loop
clutter.main()
