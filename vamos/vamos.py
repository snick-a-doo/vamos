#!/usr/bin/env python3
#  This file is part of Vamos Automotive Simulator
#  
#  Copyright 2014 Sam Varner
#
#  Vamos is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
# 
#  Vamos is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

import sys
import os
import pygame
from pygame.locals import Rect

#! Should be able to get paths from 'configure'.
sys.path = (['../%s/.libs' % subdir for subdir in 
            ['body', 'geometry', 'media', 'track', 'world']]
            + sys.path)

import config
from body import Gl_Car
from geometry import Three_Matrix, Three_Vector
from media import XML_Exception
from track import Strip_Track
from world import Atmosphere, Gl_World, Interactive_Driver, Robot_Driver, Sounds

class Entry:
    def __init__ (self, file, interactive, time):
        self.file = file
        self.interactive = interactive
        self.time = time
    def get_time (self):
        return self.time

def get_path (opt, file, section, extend):
    path = opt.data_dir + section + '/' + file
    if extend: path += '.xml'
    if os.path.exists (path):
        return path
    return file

def read_input (opt):
    entries = []
    with open (opt.input_file) as f:
        line = f.readlines ()
        opt.track_file = line [0][:-1]
        for entry in line [4:]:
            e = entry.split ()
            entries.append (Entry (e[0], e[2] == 'interactive', float (e[4])))
    entries.sort (key = Entry.get_time)
    opt.number_of_cars = len (entries)
    return entries

def get_entries (opt):
    if len (opt.input_file) > 0:
        return read_input (opt)
    opt.track_file = get_path (opt, opt.track_file, 'tracks', True)
    car_files = []
    car_path = get_path (opt, opt.car_file, 'cars', False)
    if os.path.isdir (car_path):
        for file in os.listdir (car_path):
            if os.path.splitext (file)[1] == '.xml':
                car_files.append (car_path + '/' + file)
    else:
        car_files.append (get_path (opt, opt.car_file, 'cars', True))
    car_files.sort ()
    entries = []
    while len (entries) < opt.number_of_cars:
        entries.append (Entry (car_files [len (entries) % len (car_files)], False, 0))
    assert (len (entries) == opt.number_of_cars)
    if not opt.demo:
        entries[-1].interactive = True
    return entries

import time
from OpenGL.GL import *

def vamos (opt):
#    pygame.init ()
#    screen = pygame.display.set_mode ((1000, 800), pygame.OPENGL)
#    track = Strip_Track ()
#    track.read (opt.data_dir, get_path (opt, opt.track_file, 'tracks', True))
#    track.draw ()
#    time.sleep (2)
#    return

    cars = []
    drivers = []
    try:
        entries = get_entries (opt)
        air = Atmosphere (1.2, Three_Vector())
        sounds = Sounds (opt.volume)
        track = Strip_Track ()
        world = Gl_World (track, air, sounds, opt.full_screen)
        track.read (opt.data_dir, get_path (opt, opt.track_file, 'tracks', True))
        world.display ()
        glOrtho (-500, 500, -500, 500, -1000, 1000)
        track.draw_map_background ()
        track.draw ()
        time.sleep (10)
        return

        if not opt.map_mode:
            robots = False
            interactive = False
            for entry in entries:
                cars.append (Gl_Car (track.grid_position (len (cars) + 1,
                                                          opt.number_of_cars,
                                                          opt.qualifying),
                                     Three_Matrix()))
                car = cars [-1]
                car.read (opt.data_dir, entry.file)
                car.start_engine ()
                if entry.interactive:
                    interactive = True
                    drivers.append (Interactive_Driver (car))
                    world.add_car (car, drivers [-1])
                    world.set_focused_car (len (cars) - 1)
                else:
                    robots = True
                    car.adjust_robot_parameters (opt.performance, 
                                                       opt.performance,
                                                       opt.performance)
                    drivers.append (Robot_Driver (car, track, world.get_gravity ()))
                    driver = drivers [-1]
                    if opt.qualifying:
                        driver.qualify ()
                    driver.interact (opt.interact)
                    driver.show_steering_target (opt.show_line)
                    world.add_car (car, driver)

            if robots and not interactive:
                world.set_focused_car (0)
                world.cars_can_interact (opt.interact)

            world.read (get_path (opt, opt.world_file, 'worlds', True), 
                        get_path (opt, opt.controls_file, 'controls', True))
            sounds.read (opt.data_dir + 'sounds/', 'default-sounds.xml')

    # Handle file exceptions.
    except RuntimeError:
        print (sys.exc_info()[1])
        return

    if opt.show_line or opt.demo or opt.number_of_cars > 1:
        track.build_racing_line ()
        track.show_racing_line (opt.show_line)

    world.start (opt.qualifying, opt.laps)
    world.write_results ('/tmp/vamos' + ( '-qualifying' if opt.qualifying else '-race'))


if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option ('-c', '--car', dest = 'car_file', default = 'GT')
    parser.add_option ('-t', '--track', dest = 'track_file', default = 'Peanut')
    parser.add_option ('--world', dest = 'world_file', default = 'earth')
    parser.add_option ('--controls', dest = 'controls_file', default = 'stick')
    parser.add_option ('-o', '--opponents', dest = 'number_of_cars',
                       type = 'int', default = 0)
    parser.add_option ('-p', '--laps', dest = 'laps', type = 'int', default = 5)
    parser.add_option ('-z', '--performance', dest = 'performance',
                       type = 'int', default = 100)
    parser.add_option ('-v', '--volume', dest = 'volume', type = 'int', default = 100)
    parser.add_option ('-l', '--show-line', 
                       action = 'store_true', dest = 'show_line', default = False)
    parser.add_option ('-m', '--map',
                       action = 'store_true', dest = 'map_mode', default = False)
    parser.add_option ('-d', '--demo',
                       action = 'store_true', dest = 'demo', default = False)
    parser.add_option ('-q', '--qualifying',
                       action = 'store_true', dest = 'qualifying', default = False)
    parser.add_option ('-w', '--window',
                       action = 'store_false', dest = 'full_screen', default = True)
    parser.add_option ('-n', '--no-interaction',
                       action = 'store_false', dest = 'interact', default = True)
    parser.add_option ('-r', '--random-track',
                       action = 'store_true', dest = 'random_track', default = False)
    parser.add_option ('--version',
                       action = 'store_true', dest = 'show_version', default = False)

    (options, arguments) = parser.parse_args()

    # Turn percentage of full performance into fraction of attenuation.
    options.performance = options.performance / 100.0 - 1.0

    options.volume /= 100.0

    if not options.demo or options.number_of_cars < 2:
        options.number_of_cars += 1

    options.input_file = ''
    if len (arguments) > 0:
        options.input_file = arguments [0]

    options.data_dir = '../data/'
    if not os.path.exists (options.data_dir):
        options.data_dir = config.datadir + "/";
        if not os.path.exists (options.data_dir):
            print ("Couldn't find the data direcory, ../data or %s\n" % data_dir) 
            sys.exit (1)

    if options.random_track:
        sys.path.insert (0, '../scripts')
        import random_circuit
        track = random_circuit.Random_Circuit ()
        track.construct ()
        track_file = '/tmp/random-circuit.xml'
        track.write_track (track_file)
        options.track_file = track_file

    if options.show_version:
        print (config.version)
    else:
        vamos (options)
