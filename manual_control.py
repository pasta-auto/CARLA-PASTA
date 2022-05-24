#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Space        : toggle hand-brake
    P            : toggle autopilot
    O            : set new autopilot goal
    [            : toggle display autopilot waypoints
    M            : toggle manual transmission
    ,/.          : gear up/down

    V            : toggle HUD display between PASTA and CARLA data source (defaults PASTA)
    K            : toggle between keyboard and controller mode (default controller if one is detected at startup)

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    sys.path.append(glob.glob(os.path.join(sys.path[0],'../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64')))[0])
except IndexError:
    print("Failed to import egg")
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import time
import logging
import math
import random
import re
import weakref
import socket
import json
import threading
import queue
import itertools
import datetime
from common import load_CAN_ID

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_g
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_l
    from pygame.locals import K_i
    from pygame.locals import K_z
    from pygame.locals import K_x
    from pygame.locals import K_k
    from pygame.locals import K_v
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
    from pygame.locals import K_RETURN
    from pygame.locals import K_LEFTBRACKET
    from pygame.locals import K_1
    from pygame.locals import K_2
    from pygame.locals import K_3
    from pygame.locals import K_4
    from pygame.locals import K_5
    from pygame.locals import K_6
    from pygame.locals import K_7
    from pygame.locals import K_8
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

sys.path.append(
    os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(__file__))
        , '../PythonAPI/carla'
    ))
)
from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from corrupted_vehicle import CorruptedVehicle
from configparser import ConfigParser
# ==============================================================================
# -- Globals ----------------------------------------------------------
# ==============================================================================
carlaIDMapCMD = {}
carlaIDMapRPT = {}
fullSLCANMap = {}
#TODO bit of a hack to avoid another thread / socket but letting the GUI send "kill"
#     to handle_PASTA_sock thread to quit this
g_quit = False
g_ignition_on = False
# TODO bit of a hack but need to respawn agent when back space is pressed
g_respawn_agent = False
g_reroute_agent = False
g_reached_goal = False
# TODO also bit of hack lets do only 1 calculation of RPM per tick
g_rpm_this_tick = 0
# TODO going to have to refactor KeyboardControl as need a third global to fix
# TODO idea make parse_event object so return quit yes/no + a 2nd object with all of these
g_window_resize_event = False
g_window_resize_event_size = (0,0)
# have glo0bals for another thread pass; TODO should this be LifoQueue? basically am going to have writing faster than inputing 
# so would need to do something like if empty: keep writing current
# but don't think need a queue as doesn't matter if have partial write I don't think (i.e throttle from time t-1 brake from t)
#
g_send_mutex = threading.Lock() # need this for vehicle change or else will crash send thread very quickly
g_cur_control = carla.VehicleControl()
g_pre_player  = None
g_ger_control = 0 # name is kinda bad bbut want it to match other for gear
g_ger_external_idx = 1
g_current_lights = None

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.can_addr = (args.can_host, args.can_port)
        self.din_addr = (args.data_in_host, args.data_in_port)

    def set_hud(self, hud):
        self.hud = hud
        self.world.on_tick(hud.on_world_tick)
         # Set up the sensors.
        self.collision_sensor.set_hud(self.hud)
        self.lane_invasion_sensor.set_hud(self.hud)
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0

        self.camera_manager.sensor.destroy()
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)

        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        filts = self._actor_filter.split(",")
        filteredLibs = []
        for i in filts:
            actors = self.world.get_blueprint_library().filter(i)
            if actors:
                filteredLibs.append(actors)
        if filteredLibs == []:
            print("Error: no vehicles for filter:", self._actor_filter, ". Please specify a less restricitve filter via the --filter option.")
            quit()
        filteredLibs = list(itertools.chain.from_iterable(filteredLibs))
        finalLibs = []

        # remove any non vehicles and motorcycles
        for i in filteredLibs:
            try:
                numWheels = i.get_attribute("number_of_wheels")
                if numWheels.as_int() > 2:
                    finalLibs.append(i)
            except:
                pass
        filteredLibs = finalLibs

        blueprint = random.choice(filteredLibs)

        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)
        global g_respawn_agent
        g_respawn_agent = True
        print('Actor id:', self.player.id)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

def carla_to_pasta_light_map(current_lights):
    # NOTE reverse order here carla as you cycle through these | them together PASTA they are distinct so just taking the highest one available
    pastaPassing = 0
    if   current_lights & carla.VehicleLightState.HighBeam:
        if not (current_lights & carla.VehicleLightState.LowBeam or current_lights & carla.VehicleLightState.Position):
            pastaPassing = 1
            pastaFrontLights = 6
        else:
            if current_lights & carla.VehicleLightState.LowBeam:
                pastaFrontLights = 7
            else:
                pastaFrontLights = 5
    elif current_lights & carla.VehicleLightState.LowBeam:
        pastaFrontLights = 3
    elif current_lights & carla.VehicleLightState.Position:
        pastaFrontLights = 1
    else:
        pastaFrontLights = 0      

    if  (current_lights & carla.VehicleLightState.LeftBlinker) and (current_lights & carla.VehicleLightState.RightBlinker):
        pastaTurnLights = 4
    elif current_lights & carla.VehicleLightState.LeftBlinker:
        pastaTurnLights = 1
    elif current_lights & carla.VehicleLightState.RightBlinker:
        pastaTurnLights = 2
    else: 
        pastaTurnLights = 0

    return pastaFrontLights, pastaTurnLights, pastaPassing

def carla_to_pasta_steer_map(steer):
    pastaSteer = (int)(steer * 0x1FF)
    if (pastaSteer & (1 << (16 - 1))) != 0:
        pastaSteer += 1<<16
    return pastaSteer

class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot, controller_type, mode):
        self._autopilot_enabled = start_in_autopilot
        self.controller_type = controller_type
        self.mode = mode
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._last_joystick = carla.VehicleControl() # need to save values when autopilot is enabled to make switches cleaner
            self._physics_control = world.player.get_physics_control()
            self._control.manual_gear_shift = True
            self._control.hand_brake = True
            self._lights = carla.VehicleLightState.NONE
            self._otherLights = carla.VehicleLightState.NONE
            self._turnLights  = carla.VehicleLightState.NONE
            self._frontLights = carla.VehicleLightState.NONE
            self._external_gear_idx = 0
            self._external_gears = ["P", "R", "N", "D", "L"]
            self._gear_change = 0
            self._last_gear = world.player.get_control().gear
            self._cycle_ascend = True
            # world.player.set_autopilot(self._autopilot_enabled) instead using agent 
            world.player.set_light_state(self._lights) 
            world.player.apply_control(self._control)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.can_addr = world.can_addr
        self.din_addr = world.din_addr
        self.use_joystick = False

        pygame.joystick.init()
        numJoy = pygame.joystick.get_count()
        if numJoy >= 1:
            print("Found a joystick")
            print(self.controller_type)
            self._joystick = pygame.joystick.Joystick(0)
            if numJoy > 1:
                print("Multiple joysticks detected; using first one found")
        else:
            self._joystick = None

        if self._joystick:
            self._joystick.init()
            self.use_joystick = True
            # TODO explicitly removed config parser for now reasonbeing would need to make it much more complicated
            # to handle other controller types. Basically would need brake = 1; brake_func = (x + 1) / 2 or something
            # as xbox and the racing wheel are very different
            if   self.controller_type == "Xbox":
                self.steer_idx    = 0
                self.brake_idx    = 2
                self.throttle_idx = 5
                self.shift_up_idx  = 5
                self.shift_dwn_idx = 4
                self.light_idx     = 3
                self.left_turn_idx = 2
                self.rght_turn_idx = 1
                self.hand_brke_idx = 0
                self.passing_idx   = 9
                self.ignition_idx  = 9
            elif self.controller_type == "DrivingForce GT":
                self.steer_idx     = 0
                self.brake_idx     = 2
                self.throttle_idx  = 1
                self.shift_up_idx  = 13
                self.shift_dwn_idx = 12
                self.light_idx     = 3
                self.left_turn_idx = 1
                self.rght_turn_idx = 2
                self.hand_brke_idx = 0
                self.passing_idx   = 10
                self.ignition_idx  = 20
            else:
                print("Unrecognized controller type")

    def gear_up(self, world):
        self._control.gear = self._control.gear + 1
        self._gear_change = 1
        if self._external_gear_idx > 0:
            self._external_gear_idx = self._external_gear_idx - 1
            self._last_gear = world.player.get_control().gear
    
    def gear_down(self, world):
        self._control.gear = max(-1, self._control.gear - 1)
        self._gear_change = 2
        if self._external_gear_idx < len(self._external_gears) - 1:
            self._external_gear_idx = self._external_gear_idx + 1
            self._last_gear = world.player.get_control().gear

    def cycle_lights(self, world):
        if self._cycle_ascend:
            if not self._lights & carla.VehicleLightState.Position:
                world.hud.notification("Position lights")
                self._frontLights |= carla.VehicleLightState.Position
            else:
                world.hud.notification("Low beam lights")
                self._frontLights |= carla.VehicleLightState.LowBeam
            if self._lights & carla.VehicleLightState.LowBeam:
                world.hud.notification("High beam lights")
                self._frontLights |= carla.VehicleLightState.HighBeam
                self._cycle_ascend = False
        else:
            if self._lights & carla.VehicleLightState.HighBeam:
                world.hud.notification("Low beam lights")
                self._frontLights ^= carla.VehicleLightState.HighBeam
            elif self._lights & carla.VehicleLightState.LowBeam:
                world.hud.notification("Position lights")
                self._frontLights ^= carla.VehicleLightState.LowBeam
            else:
                world.hud.notification("Lights Off")
                self._frontLights ^= carla.VehicleLightState.Position
                self._cycle_ascend = True

    def parse_events(self, client, world, clock, args, auto_agent):
        if isinstance(self._control, carla.VehicleControl):
            current_lights    = self._lights
        global g_ignition_on
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.VIDEORESIZE:
                global g_window_resize_event
                global g_window_resize_event_size
                g_window_resize_event = True
                g_window_resize_event_size = event.dict['size']
            #elif event.type == pygame.VIDEOEXPOSE:  # handles window minimising/maximising TODO can I just ignore? this seems to always trigger with above
            elif event.type == pygame.JOYAXISMOTION:
                if args.debug_controller:
                    print(event)
                if not self.use_joystick:
                    continue
                if   self.controller_type == "Xbox":
                    if event.dict['axis'] == self.steer_idx: # already between [-1, 1]
                        self._last_joystick.steer = event.dict['value']
                    elif event.dict['axis'] == self.brake_idx:
                        self._last_joystick.brake = (event.dict['value'] - (-1))/2 # [-1,1] -> [0,1]
                    elif event.dict['axis'] == self.throttle_idx:
                        self._last_joystick.throttle = (event.dict['value'] - (-1))/2 # [-1,1] -> [0,1]

                elif self.controller_type == "DrivingForce GT":
                    if event.dict['axis'] == self.steer_idx: # already between [-1, 1] TODO
                        self._last_joystick.steer = event.dict['value']
                    elif event.dict['axis'] == self.brake_idx: 
                        self._last_joystick.brake    = (event.dict['value'] * -0.5) + 0.5 # [1, -1] - > [0, 1]
                    elif event.dict['axis'] == self.throttle_idx:
                        self._last_joystick.throttle = (event.dict['value'] * -0.5) + 0.5 # [1, -1] - > [0, 1]
                if not self._autopilot_enabled:
                    self._control.steer    = self._last_joystick.steer
                    self._control.brake    = self._last_joystick.brake
                    self._control.throttle = self._last_joystick.throttle
            elif event.type == pygame.JOYBUTTONUP:
                if not self.use_joystick:
                    continue
                if   event.dict['button'] == self.shift_up_idx:
                    self._gear_change = 0
                elif event.dict['button'] == self.shift_dwn_idx:
                    self._gear_change = 0

            elif event.type == pygame.JOYBUTTONDOWN:
                if args.debug_controller:
                    print(event)
                if not self.use_joystick:
                    continue
                if   event.dict['button'] == self.shift_up_idx:
                    self.gear_up(world)
                elif event.dict['button'] == self.shift_dwn_idx:
                    self.gear_down(world)
                elif event.dict['button'] == self.hand_brke_idx:
                    self._control.hand_brake = self._control.hand_brake = not self._control.hand_brake
                elif event.dict['button'] == self.light_idx:
                    self.cycle_lights(world)
                elif event.dict['button'] == self.left_turn_idx:
                    self._turnLights ^= carla.VehicleLightState.LeftBlinker
                elif event.dict['button'] == self.rght_turn_idx:
                    self._turnLights ^= carla.VehicleLightState.RightBlinker
                elif event.dict['button'] == self.passing_idx:
                    self._frontLights ^= carla.VehicleLightState.HighBeam
                elif event.dict['button'] == self.ignition_idx and args.mode == 0:
                    g_ignition_on = not g_ignition_on
            elif event.type == pygame.JOYHATMOTION:
                if args.debug_controller:
                    print(event)
            elif event.type == pygame.JOYBALLMOTION:
                if args.debug_controller:
                    print(event)
            elif event.type == pygame.KEYDOWN:
                # want this to send 1 while holding up and have KEYUP event resent to sending 0
                # TODO probably can kill manual_gear shift / vehicle control checks here; should probably never have a walker or automatic gear for manual control anymore I think
                if isinstance(self._control, carla.VehicleControl) and not self.use_joystick:
                    if   self._control.manual_gear_shift and event.key == K_COMMA:
                        self.gear_down(world)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self.gear_up(world)
                    elif event.key == K_SPACE: # bit weird having this here but need it on down to mimic how PASTA works
                        self._control.hand_brake = not self._control.hand_brake
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_k:
                    self.use_joystick = not self.use_joystick
                    world.hud.notification(
                            'Controller %s' % ('On' if self.use_joystick else 'Off'))
                elif event.key == K_BACKSPACE:
                    g_send_mutex.acquire()
                    #taking this out not using world.set_autopilot
                    #if self._autopilot_enabled:
                    #    world.player.set_autopilot(False)
                    #    world.restart()
                    #    world.player.set_autopilot(True)
                    #else:
                    world.restart()
                    # want to save this but if spawning a new vehicle need to reget the physics control
                    self._physics_control = world.player.get_physics_control()
                    g_send_mutex.release()
                elif event.key == K_v:
                    world.hud.toggle_data_source()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_o:
                        global g_reroute_agent
                        world.hud.notification('Setting a new autopilot route')
                        g_reroute_agent = True
                    elif event.key == K_LEFTBRACKET:
                        args.debug_auto_waypoint = not args.debug_auto_waypoint
                        world.hud.notification(
                            'Show autopilot waypoints %s' % ('On' if args.debug_auto_waypoint else 'Off'))
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                        # when turn off autopilot need to load joystick values changes we got while it was on
                        if not self._autopilot_enabled:
                            self._control.throttle   = self._last_joystick.throttle
                            self._control.steer      = self._last_joystick.steer
                            self._control.brake      = self._last_joystick.brake
                            self._control.hand_brake = self._last_joystick.hand_brake
                            self._control.gear       = self._last_joystick.gear
                    if not self.use_joystick:
                        if event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                            self._otherLights ^= carla.VehicleLightState.Special1
                        elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                            self._frontLights ^= carla.VehicleLightState.HighBeam
                        elif event.key == K_l:
                            # Use 'L' key to switch between lights:
                            self.cycle_lights(world)
                        elif event.key == K_i:
                            self._otherLights ^= carla.VehicleLightState.Interior
                        elif event.key == K_z:
                            self._turnLights ^= carla.VehicleLightState.LeftBlinker
                        elif event.key == K_x:
                            self._turnLights ^= carla.VehicleLightState.RightBlinker
                        elif self._control.manual_gear_shift and event.key == K_COMMA:
                            self._gear_change = 0
                        elif self._control.manual_gear_shift and event.key == K_PERIOD:
                            self._gear_change = 0
                    if event.key == K_RETURN:
                        if args.mode == 0:
                            g_ignition_on = not g_ignition_on

            #else:
            #    print("Unknown event", event)
        #if not self._autopilot_enabled:
        if isinstance(self._control, carla.VehicleControl):
            if (not self._autopilot_enabled) and not self.use_joystick:
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            elif self._autopilot_enabled:
                #TODO refactor
                global g_respawn_agent, g_reached_goal 
                if g_respawn_agent: # don't try and run_step on a destroyed object
                    return
                if (self._external_gear_idx == 3 or self._external_gear_idx == 4) and not g_reached_goal: # only let auto pilot do stuff if we are in D or L and haven't reached goal
                    agent_control = auto_agent.run_step(debug=args.debug_auto_waypoint)
                    self._control.throttle          = agent_control.throttle
                    self._control.steer             = agent_control.steer
                    self._control.brake             = agent_control.brake
                    self._control.manual_gear_shift = True # don't want auto pilot to be able to move in park so just handle gear changes ourselves
                    self._control.gear              = agent_control.gear
                else: # not in D or L just 0 it
                    self._control.throttle          = 0
                    self._control.steer             = 0
                    self._control.brake             = 0
                    self._control.manual_gear_shift = True 

                # print(agent_control)
                # TODO no lights?
                # TODO hand brake
                
            self._control.reverse = self._external_gear_idx == 1

            current_lights =  self._frontLights | self._turnLights | self._otherLights

            # Set automatic control-related vehicle lights
            if self._control.brake:
                current_lights |= carla.VehicleLightState.Brake
            else: # Remove the Brake flag
                current_lights &= carla.VehicleLightState.All ^ carla.VehicleLightState.Brake
            if self._control.reverse:
                current_lights |= carla.VehicleLightState.Reverse
            else: # Remove the Reverse flag
                current_lights &= carla.VehicleLightState.All ^ carla.VehicleLightState.Reverse
    
            # regardless of mode send data in the current ID
            self.sock.sendto(bytes("{:03x}".format(world.player.id), "ascii"), self.din_addr)

            self._lights = current_lights

            prec = world.player.get_control()

            # TODO all of this is to get state to the sending thread and it is messy should really refactor and think of a better way to do this
            # for now think this is ok 
            global g_cur_control, g_pre_player, g_ger_control, g_ger_external_idx, g_current_lights
            g_cur_control = self._control
            g_pre_player  = world.player
            g_ger_control = self._gear_change
            g_ger_external_idx = self._external_gear_idx
            g_current_lights = current_lights

            engine_rpm = self._physics_control.max_rpm * prec.throttle
            if prec.gear >= 0:
                try: 
                    engine_rpm *= self._physics_control.forward_gears[prec.gear].ratio
                except:
                    pass
            if engine_rpm > 10000:
                engine_rpm = 10000
            if engine_rpm < 600: 
                engine_rpm = 600
            engine_rpm = engine_rpm if g_ignition_on else 0
            global g_rpm_this_tick
            g_rpm_this_tick = engine_rpm
    
            if args.apply_control:
                if not g_ignition_on:
                    self._control.throttle = 0
                # TODO is there a reason to ever not do this if applying control? gut says no always want to try and map CARLA gears to PASTA (-1,0,1.. -> P,R,N etc.)
                if self._external_gear_idx == 0 or self._external_gear_idx == 2: #P/N
                    self._control.gear = 0
                elif self._external_gear_idx == 1:
                    self._control.gear = -1
                elif self._external_gear_idx == 3:
                    self._control.gear = 3
                elif self._external_gear_idx == 4:
                    self._control.gear = 1
                world.hud.set_gear(self._external_gears[self._external_gear_idx])

                if prec.gear == 1 and (self._external_gear_idx != 4):
                    if self._control.throttle != 1.0:
                        self._control.throttle += 0.000001 #* clock.get_time()
                    else:
                        self._control.throttle -= 0.000001 #* clock.get_time()

                #print("self  ", self._control)
                #print("world ", prec)

                world.player.apply_control(self._control)
                world.player.set_light_state(carla.VehicleLightState(self._lights)) 

        elif isinstance(self._control, carla.WalkerControl):
            self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
            world.player.apply_control(self._control)    
            
    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.01, 1)
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            # TODO do we want this the same as throttle?
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        # TODO whhy lock this to 0.7? for now switching to 1
        self._steer_cache = min(1.0, max(-1.0, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)

    def _parse_walker_keys(self, keys, milliseconds, world):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height, disable_light_state_display, mode):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        self.mode = mode
        self.use_external_gear = True #TODO HUD make it just mode or rename to more general
        self.use_pasta_values = mode != 0
        self.pasta_control = carla.VehicleControl()
        self.pasta_rpm = 0
        self.pasta_kmh = 0
        self.pasta_gears = ["P", "R", "N", "D", "L"] # TODO reusing the thing in Keyboard control naming external == pasta
        self.external_gear = 'P'
        self.ls_display = not disable_light_state_display
        self.auto_pilot_goal = carla.Location()
        self.auto_pilot_next = carla.Location()
        self.auto_pilot_pts_until_goal = 21
        self.sensor_type = ''
        self.sensor_params = []

    def set_autopilot_targets(self, auto_pilot_goal, auto_pilot_next, pts):
        self.auto_pilot_goal = auto_pilot_goal
        self.auto_pilot_next = auto_pilot_next
        self.auto_pilot_pts_until_goal = pts

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def toggle_data_source(self):
        self.use_pasta_values = not self.use_pasta_values
        if self.use_pasta_values:
            self.notification("HUD using PASTA values")
        else:
            self.notification("HUD using CARLA values")

    def set_gear(self, gear):
        self.external_gear = gear

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        # these do not ping the simulator
        t = world.player.get_transform()
        v = world.player.get_velocity()
        if not self.use_pasta_values:
            c = world.player.get_control()
            kmh = (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
            rpm = g_rpm_this_tick
        else:
            c = self.pasta_control
            # probably always want carlaGear supplied by server
            c.gear = world.player.get_control().gear
            if self.mode == 1:
                kmh = (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
                rpm = g_rpm_this_tick
            else:
                kmh = self.pasta_kmh
                rpm = self.pasta_rpm
        # this does
        if self.ls_display:
            try:
                l = world.player.get_light_state()
            except:
                l = carla.VehicleLightState.NONE
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            'Actor ID %20s' % str(world.player.id),
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % kmh,
            'RPM:   % 17d' % rpm,
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location  :(%5.1f,%5.1f)' % (t.location.x, t.location.y),
            'Next Point:(%5.1f,%5.1f)' % (self.auto_pilot_next.x, self.auto_pilot_next.y),
            'Goal Point:(%5.1f,%5.1f)' % (self.auto_pilot_goal.x, self.auto_pilot_goal.y),
            'Estimated Left: %d' % (self.auto_pilot_pts_until_goal),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl): 
            self._info_text += [
                ('Ignition:', (bool)(g_ignition_on)),
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift)]
            if not self.use_external_gear:
                self._info_text += ['Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
            else:
                self._info_text += ['Gear:        %s' % self.external_gear] 
                self._info_text += ['CarlaGear:   %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        if self.ls_display:
            lightDescrip = ''
            if   (l & carla.VehicleLightState.LeftBlinker) and (l & carla.VehicleLightState.RightBlinker):
                lightDescrip += 'Hazard'
            elif (l & carla.VehicleLightState.LeftBlinker):
                lightDescrip += 'Left Turn'
            elif (l & carla.VehicleLightState.RightBlinker):
                lightDescrip += 'Right Turn'
            if   (l & carla.VehicleLightState.Fog):
                lightDescrip += ' Fog'
            if (l & carla.VehicleLightState.HighBeam):
                lightDescrip += ' High Beam'
            if (l & carla.VehicleLightState.LowBeam):
                lightDescrip += ' Low Beam'
            if (l & carla.VehicleLightState.Position):
                lightDescrip += ' Position'
            if   (l & carla.VehicleLightState.Brake):
                lightDescrip += ' Brake'
            if   (l & carla.VehicleLightState.Reverse):
                lightDescrip += ' Reverse'
            if lightDescrip == '':
                lightDescrip = 'Off'
            self._info_text += [
                'Light state: %d' % l,
                lightDescrip,
            ]

        self._info_text += [
            'Collision:',
            collision,
            '']

        self._info_text += ['Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def set_pasta_control(self, control, rpm, kmh, gear):
        self.pasta_control = control
        self.pasta_rpm = rpm
        self.pasta_kmh = kmh
        if gear == 0:
            gear = 1
        if gear > len(self.pasta_gears):
            gear = len(self.pasta_gears)
        if self.mode !=0: # if we are applying gear ourselves don't want PASTA changing it
            self.external_gear = self.pasta_gears[gear - 1]

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(200)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (900, len(lines) * self.line_space + 12)
        self.pos = (0.6 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def set_hud(self, hud):
        self.hud = hud

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    def set_hud(self, hud):
        self.hud = hud

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        Attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
            (carla.Transform(carla.Location(x=5.5, y=1.5, z=1.5)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-8.0, z=6.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-1, y=-bound_y, z=0.5)), Attachment.Rigid)]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
                'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}]]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '50')
            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            #print(self.sensors[index][-1], self.transform_index)
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            if points.shape[0] % 3 != 0:
                points = points[0:points.shape[0]-(points.shape[0] % 3)]
            points = np.reshape(points, (int(points.shape[0] / 3), 3))

            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================
def route_agent(world, agent, set_destination=True):
    spawn_points = world.map.get_spawn_points()
    random.shuffle(spawn_points)
    if spawn_points[0].location != agent.vehicle.get_location():
        destination = spawn_points[0].location
    else:
        destination = spawn_points[1].location
    agent.reroute(spawn_points)
    agent.get_local_planner()._waypoint_buffer.clear()
    # This is kind of weird but need it for that attacks in spawn agent need things to happen in between
    if set_destination:
        agent.set_destination(agent.vehicle.get_location(), destination, clean=True)
    return destination

def spawn_agent(args, world):
    corrupted_vehcile = CorruptedVehicle(args, world.player)
    # TODO removed other autopilot agents here handling roaming / basic is very different removing for now (check examples if want to readd)
    agent = BehaviorAgent(corrupted_vehcile, ignore_traffic_light=args.attack_ignore_lights, behavior=args.behavior)
    spawn_points = world.map.get_spawn_points()
    random.shuffle(spawn_points)
    if spawn_points[0].location != agent.vehicle.get_location():
        destination = spawn_points[0].location
    else:
        destination = spawn_points[1].location
    if args.attack_zero_location:
        corrupted_vehcile.set_zero_location(False)
    if args.attack_init_location:
        corrupted_vehcile.set_zero_location(True)
    agent.set_destination(agent.vehicle.get_location(), destination, clean=True)
    if args.attack_init_location:
        corrupted_vehcile.set_zero_location(False)
    if args.attack_zero_location:
        corrupted_vehcile.set_zero_location(True)
    return agent

def respawn_agent(args, agent, world):
    agent.vehicle = world.player
    agent.get_local_planner()._vehicle = world.player


def handle_PASTA_sending(sock, mode, can_addr):
    current_time = datetime.datetime.now()
    time_period = datetime.timedelta(milliseconds=50)
    goal = current_time + time_period
    prevHandBrake = 0
    lastIgnition = 0
    while True:
        
        time.sleep(0.01) # TODO want to do a busy loop for better accuracy?
        current_time = datetime.datetime.now()
        send_50ms_period_msg = False
        if current_time > goal:
            goal = current_time + time_period
            send_50ms_period_msg = True

        if g_pre_player == None:
            continue
        g_send_mutex.acquire()
        try:
            prec = g_pre_player.get_control()
            v    = g_pre_player.get_velocity()
            kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
            engine_rpm = g_rpm_this_tick
            engine_rpm = int(engine_rpm)
            if mode == 1: # SLCAN
                sock.sendto(bytes("{:03x} {:0{datasize}x}".format(carlaIDMapRPT['gear']      , g_pasta_gear          , datasize=2*fullSLCANMap[carlaIDMapRPT['gear']      ]['datasize']), 'ascii'), can_addr)
                sock.sendto(bytes("{:03x} {:0{datasize}x}".format(carlaIDMapRPT['hand_brake'], (int)(prec.hand_brake), datasize=2*fullSLCANMap[carlaIDMapRPT['hand_brake']]['datasize']), 'ascii'), can_addr)
                # previous frames data
                sock.sendto(bytes("{:03x} {:0{datasize}x}"               .format(carlaIDMapRPT['brake']     , (int) (prec.brake    * 0x3FF)       , datasize=2*fullSLCANMap[carlaIDMapRPT['brake']      ]['datasize']), 'ascii'), can_addr)
                sock.sendto(bytes("{:03x} {:0{datasize}x}"               .format(carlaIDMapRPT['throttle']  , (int) (prec.throttle * 0x3FF)       , datasize=2*fullSLCANMap[carlaIDMapRPT['throttle']   ]['datasize']), 'ascii'), can_addr)
                # not 2x data size as has 2 variable worth so half for each
                sock.sendto(bytes("{:03x} {:0{datasize}x}{:0{datasize}x}".format(carlaIDMapRPT['rpm']       , engine_rpm, kmh                     , datasize=  fullSLCANMap[carlaIDMapRPT['rpm']        ]['datasize']), 'ascii'), can_addr)
                # TODO 0 for torque
                sock.sendto(bytes("{:03x} {:0{datasize}x}0000"           .format(carlaIDMapRPT['steer']     , carla_to_pasta_steer_map(prec.steer), datasize=2*fullSLCANMap[carlaIDMapRPT['steer']      ]['datasize']), 'ascii'), can_addr)
                # currently no way to get tire angle seperate from steer in CARLA for now just using same steering angle
                sock.sendto(bytes("{:03x} {:0{datasize}x}"               .format(carlaIDMapRPT['tire_angle'], carla_to_pasta_steer_map(prec.steer), datasize=2*fullSLCANMap[carlaIDMapRPT['tire_angle'] ]['datasize']), 'ascii'), can_addr)
                if send_50ms_period_msg:
                    sock.sendto(bytes("{:03x} {:0{datasize}x}"               .format(carlaIDMapRPT['kmh']       , kmh          , datasize=2*fullSLCANMap[carlaIDMapRPT['kmh']   ]['datasize']), 'ascii'), can_addr)
                    sock.sendto(bytes("{:03x} {:0{datasize}x}"               .format(carlaIDMapRPT['engine']    , g_ignition_on, datasize=2*fullSLCANMap[carlaIDMapRPT['engine']]['datasize']), 'ascii'), can_addr)
        except Exception as e:
            logging.debug("PASTA send thread exception")
            logging.debug(e)
        finally:
            g_send_mutex.release()


# TODO doing this way too much
g_pasta_control = carla.VehicleControl()
g_pasta_rpm     = 0
g_pasta_kmh     = 0 
g_pasta_gear = 1
g_pasta_ud_gear = 1
def handle_pasta_sock(sock, mode):
    control = carla.VehicleControl()
    control.manual_gear_shift = True
    rpm = 0
    kmh = 0
    gear = 1
    last_ignition = 0
    last_gear = 0
    external_gear_idx = 0
    while True:
        try:
            inputData = sock.recv(1024)
        except socket.timeout:
            inputData = None
        if inputData == None:
            continue
        control.reverse = False
        # TODO this has a lot of overlap with data in (though not 100%) can make this a common func?
        # TODO need to swap command values for report ones split IDMap into report and command Map

        if inputData == b"kill":
            global g_quit
            g_quit = True
            return 
        if mode == 0: # maybe unnescary but in mode 0 ignore messages if we get them
            continue
        try:
            can = inputData.split(b' ')
            id = int(can[0], 16)
            dataSize = int(can[1], 16) #don't need datasize as we know always 8 for SLCAN
            data = int(can[2], 16)


            if   (id == carlaIDMapCMD['throttle']):
                control.throttle = data / 0x3FF
            elif (id == carlaIDMapCMD['brake']):
                control.brake = data / 0x3FF
            elif (id == carlaIDMapRPT['rpm']): #0x67 TODO others are commands this is a report
                # can't use data as has kmh and rpm values
                idx = len(can[2]) - 4 # RPM has no leading 0 followed by 4 byte kmh
                if idx > 1:
                    rpm = int(can[2][0:idx], 16)
                    kmh = int(can[2][idx:], 16)
                else:
                    rpm = 0
                    kmh = 0
            elif (id == carlaIDMapCMD['steer']):
                if (data & (1 << (16 - 1))) != 0:
                    data = data - (1 << 16)
                control.steer = data / 0x1FE 
            elif (id == carlaIDMapCMD['gear']): # TODO this will probably break if sent both make sure either sending CMD or RPT for gear not both
                if last_gear != data:
                    if   data == 1: # gear up   : L towards P
                        external_gear_idx = max(0, external_gear_idx - 1) # can't go past P
                    elif data == 2: # gear down : P towards L
                        external_gear_idx = min(4, external_gear_idx + 1) # 4 == len(external_gears) - 1 not bothering putting it here but P,R,N,D,L
                last_gear = data
                gear = external_gear_idx + 1
            elif (id == carlaIDMapRPT['gear']): # if getting gear to apply directly
                if   data == 1: # P
                    control.gear = 0
                elif data == 2: # R
                    control.reverse = True
                    control.gear = -1
                elif data == 3: # N
                    control.gear = 0
                elif data == 4: # D
                    control.gear = 3
                elif data == 5:
                    control.gear = 1 # L
                gear = data
                # TODO #add a set_gear here? maybe take it out of the send?
            elif (id == carlaIDMapCMD['hand_brake']):
                control.hand_brake = data==1
            elif (id == 24): # TODO not a canID but just kmh
                kmh = data
            # going to use engine status instead
            #elif (id == carlaIDMapCMD['ignition']):
            #    if last_ignition == 0 and data == 1:
            #        global g_ignition_on
            #        g_ignition_on = not g_ignition_on
            #    last_ignition = data
            elif (id == carlaIDMapRPT['engine']):
                global g_ignition_on
                g_ignition_on = data
        except Exception as e: # couldn't parse data or otherwise got a bad value clear it and wait for more input
            print(e)
            print(inputData)
            inputData = None
            continue

        
        #TODO only on change? should this lock or do care if have parital write?
        global g_pasta_control, g_pasta_rpm, g_pasta_kmh, g_pasta_gear
        g_pasta_control = control
        g_pasta_rpm     = rpm
        g_pasta_kmh     = kmh
        g_pasta_gear    = gear

def update_hud(hud):
    global g_pasta_control, g_pasta_rpm, g_pasta_kmh, g_pasta_gear        
    hud.set_pasta_control(g_pasta_control, g_pasta_rpm, g_pasta_kmh, g_pasta_gear)

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    tot_target_reached = 0
    num_min_waypoints = 21

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        try:
            display = pygame.display.set_mode(
                (args.width, args.height),
                pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE)
        except Exception as e:
            print("Could not create the pygame window:", e)
            quit()

        hud = HUD(args.width, args.height, args.disable_light_state_display, args.mode)
        try:
            world = World(client.get_world(), hud, args)
        except RuntimeError as e:
            print("No response from CARLA server:", e)
            quit()

        pasta_rec_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        try:
            pasta_rec_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except AttributeError:
            pass
        pasta_rec_sock.setblocking(True)
        pasta_rec_sock.bind((args.manual_control_ip, args.manual_control_port))
        pasta_rec_sock.settimeout(1)
        pasta_rec_hud_thread = threading.Thread(target = handle_pasta_sock, args = [pasta_rec_sock, args.mode])
        pasta_rec_hud_thread.setDaemon(True) # kill this when parent dies
        pasta_rec_hud_thread.start()

        send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        pasta_send_thread = threading.Thread(target = handle_PASTA_sending, args = [send_sock, args.mode, (args.can_host, args.can_port)])
        pasta_send_thread.setDaemon(True)
        if args.mode != 0:
            pasta_send_thread.start()

        controller = KeyboardControl(world, args.autopilot, args.controller, args.mode)

        agent = spawn_agent(args, world)

        # need to do a fake run_step as queue is updated after first one 
        #agent.update_information(world)
        agent.update_information()
        agent.run_step()
        
        # sometimes random path gives us a very short path
        qLen = len(agent.get_local_planner().waypoints_queue)
        while qLen < num_min_waypoints:
            route_agent(world, agent)
            qLen = len(agent.get_local_planner().waypoints_queue)

        # this is a bit messy so basing goal on front point in the waypoint buffer
        # so with point x1,x2,x_goal want to display x_goal which we will be at when the front of buffer is x_goal+1 
        auto_pilot_goal = agent.get_local_planner().waypoints_queue[-1* num_min_waypoints]   [0].transform.location
        display_goal    = agent.get_local_planner().waypoints_queue[-1*(num_min_waypoints-1)][0].transform.location

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)

            # if args.agent == "Behavior": TODO intentionally leaving this here if want to add back agent but is broken for respawn currently
            agent.update_information()
            # Set new destination when target has been reached
            global g_reroute_agent, g_reached_goal
            if g_reroute_agent:
                g_reroute_agent = False
                g_reached_goal  = False

                route_agent(world, agent)
                qLen = len(agent.get_local_planner().waypoints_queue)
                while qLen < num_min_waypoints:
                    route_agent(world, agent)
                    qLen = len(agent.get_local_planner().waypoints_queue)

                auto_pilot_goal = agent.get_local_planner().waypoints_queue[-1* num_min_waypoints]   [0].transform.location
                display_goal    = agent.get_local_planner().waypoints_queue[-1*(num_min_waypoints-1)][0].transform.location
            pts_left = (len(agent.get_local_planner().waypoints_queue) - num_min_waypoints) + len(agent.get_local_planner()._waypoint_buffer) + 1
            if not g_reached_goal and pts_left <= 0:
                g_reached_goal = True
                controller._autopilot_enabled = False
                tot_target_reached += 1
                world.hud.notification("The target has been reached " +
                                        str(tot_target_reached) + " times. Stopping Autopillot", seconds=4.0)

            if len(agent.get_local_planner()._waypoint_buffer) > 0:
                auto_pilot_next = agent.get_local_planner()._waypoint_buffer[0][0].transform.location
            else:
                auto_pilot_next = agent.get_local_planner().waypoints_queue[0][0].transform.location
            if controller.parse_events(client, world, clock, args, agent):
                g_send_mutex.acquire() # about to quit so just grab this, never releasing, to prevent sending with a dead actor
                return

            if g_quit:
                return

            global g_respawn_agent
            if g_respawn_agent:
                # TODO if ever add back attacks needs corrupted w/e here 
                respawn_agent(args, agent, world)
                g_respawn_agent = False

            global g_window_resize_event
            global g_window_resize_event_size
            if g_window_resize_event:
                display.fill((0, 0, 0))
                display = pygame.display.set_mode(
                   (g_window_resize_event_size[0], g_window_resize_event_size[1]),
                    pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE)
                hud = HUD(g_window_resize_event_size[0], g_window_resize_event_size[1], args.disable_light_state_display, args.mode)
                world.set_hud(hud)
                g_window_resize_event = False
            hud.set_autopilot_targets(display_goal, auto_pilot_next, pts_left)
            update_hud(hud)
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:

        if args.send_kill_on_quit:
            send_sock.sendto(bytes("kill", 'ascii'), world.can_addr)
            send_sock.sendto(bytes("kill", 'ascii'), world.din_addr)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of the possible agent behaviors (default: normal) ',
        default='normal')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--can_port',
        metavar='P',
        default=3000,
        type=int,
        help='UDP port to listen to (default: 3000)')
    argparser.add_argument(
        '--can_host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '--data_in_port',
        default=3002,
        type=int,
        help='UDP port to send actor ID to (default: 3002)')
    argparser.add_argument(
        '--data_in_host',
        default='127.0.0.1',
        help='IP of the data in server (default: 127.0.0.1)')
    argparser.add_argument(
        '--manual_control_port',
        type=int,
        default=3003,
        help='UDP port to send data from PASTA to the HUD (default: 3003)')
    argparser.add_argument(
        '--manual_control_ip',
        default='127.0.0.1',
        help='IP to listen to (default: 127.0.0.1)')
    argparser.add_argument(
        '--can_id',
        default='CAN_ID.json',
        help='Path to file containing CAN_IDs for messages (default CAN_ID.json in current directory)'
    )
    argparser.add_argument(
       '--debug_auto_waypoint',
        dest='debug_auto_waypoint', 
        action='store_true',
        default=False,
        help='Display target point for auto agent'
    )
    argparser.add_argument(
       '--attack_ignore_lights',
        dest='attack_ignore_lights', 
        action='store_true',
        default=False,
        help='Causes the auto agent to ignore traffic lights.'
    )
    argparser.add_argument(
       '--attack_swerve',
        dest='attack_swerve', 
        action='store_true',
        default=False,
        help='Causes the auto agent to serve in lane. Note this can cause agent to ignor traffic lights depending on swerve timing.'
    )
    argparser.add_argument(
       '--attack_zero_location',
        dest='attack_zero_location', 
        action='store_true',
        default=False,
        help='Causes the auto agent to get bad location data (0) except for the initial route planning. This is done to not completely break the agent.' 
    )
    argparser.add_argument(
       '--attack_init_location',
        dest='attack_init_location', 
        action='store_true',
        default=False,
        help='Causes the auto agent to get bad location data the first time it asks for it. Note This badly breaks route planning causing the agent to attempt to drive through walls.' 
    )
    argparser.add_argument(
       '--apply_control',
        dest='apply_control',
        action='store_true',
        default=False,
        help='If set will apply vehicle control here rather than in sepearte data in module.' 
    )
    argparser.add_argument(
       '--debug_controller',
        dest='debug_controller', 
        action='store_true',
        default=False,
        help='If set will print controller event messages.' 
    )
    argparser.add_argument(
        '-c', '--controller', type=str,
        dest='controller',
        choices=["Xbox", 'DrivingForce GT'],
        help='Choose one of the possible controllers (default: DrivingForce GT) ',
        default='DrivingForce GT')
    argparser.add_argument(
       '--disable_light_state_display',
        action='store_true',
        default=False,
        help='Enables the light state display on the HUD (textual display of which lights are currently on. Default:Off).' 
    )
    argparser.add_argument(
       '--send_kill_on_quit',
        action='store_true',
        default=False,
        help='Send a message on quitting which will cause the connected can_udp_server and data_in programs to also exit. (Default:Off).' 
    )
    argparser.add_argument(
        '--mode',
        default=0,
        type = int,
        choices=[0,1],
        help='Mode to opearate in see main.py for full details.'
    )
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    global carlaIDMapCMD, carlaIDMapRPT, fullSLCANMap
    # load the CAN-IDs with command tag for variables we care about
    # (want to send status changes here)
    carlaIDMapCMD = load_CAN_ID('command', args.can_id)
    carlaIDMapRPT = load_CAN_ID('report' , args.can_id)
    data = ''
    with open(os.path.join(sys.path[0], args.can_id), 'r') as file:
        data = file.read()
    temp = json.loads(data)['can_id']
    for k, v in temp.items():
        fullSLCANMap[int(k)] = v

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
