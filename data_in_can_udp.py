#!/usr/bin/env python

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
    pass

import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import time
import socket
import struct
import json
import threading

from common import load_CAN_ID
carlaIDMap = {}
carlaIDMapRPT = {}

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


class World(object):
    def __init__(self, carla_world, args):
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.sensors = None
        self.actor = None
        self.control = None

    def tick(self):
        self.sensors.tick()

    def getActor(self, id):
        print("Actor ID set", id)
        self.actor = self.world.get_actor(id)
        if self.actor:
            try:
                self.control = self.actor.get_control()
            except AttributeError:
                print("Actor ID", id, "is not controllable")
                self.actor = None
        else:
            print("Nothing exists for ID", id)
        
    def get_control(self):
        return self.control
        
    def get_player(self):
        return self.actor
		
# TODO how bad is this? makes code real simple but global; I think this is ok just for a simple reader / writer thread
# handle_vid_scok thread listens on the socket and updates these variables
# if the new_id flag has been set 
g_new_vehicle_id_available = False # global if there is a new id on the socket; set to True in sock handler set False after used
g_new_id = 0 # the new id gotten
g_kill = False
def handle_vid_sock(sock, initId):
    prevId = initId
    while True:
        try:
            viddata = sock.recv(1024)
        except socket.timeout:
            viddata = None

        if viddata is not None:
            if viddata == b'kill':
                global g_kill
                g_kill = True
                continue
            vid = int(viddata, 16)
            global g_new_vehicle_id_available
            global g_new_id
            # if the id has changed or we have not finished changing the last id
            if prevId != vid and not g_new_vehicle_id_available:
                g_new_id = vid
                prevId = vid
                g_new_vehicle_id_available = True
            viddata = None

def input_loop(args):
    world = None
    sensors = None

    try:
        client = carla.Client(args.carla_host, args.carla_port)
        client.set_timeout(2.0)

        try:
            world = World(client.get_world(), args)
        except RuntimeError as e:
            print("No response from CARLA server:", e)
            quit()
        
        control = world.get_control()
        player = world.get_player()
        frontLights  = carla.VehicleLightState.NONE
        turnLights   = carla.VehicleLightState.NONE
        brakeLights  = carla.VehicleLightState.NONE
        currentLight = carla.VehicleLightState.NONE


        data = ''
        with open(os.path.join(sys.path[0], args.can_id), 'r') as file:
            data = file.read()
        testDict = json.loads(data)
        data = ''

        vidsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        try:
            vidsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except AttributeError:
            pass
        vidsock.setblocking(True)
        vidsock.bind((args.vid_ip, args.vid_port))
        vidsock.settimeout(1)

        if args.actor_id:
            world.getActor(args.actor_id)
            control = world.get_control()
            player = world.get_player()
            vid = args.actor_id
            
        while control is None:
            try:
                viddata = vidsock.recv(1024)
            except socket.timeout:
                viddata = None

            if viddata is not None:
                if viddata == b'kill':
                    print("Got kill message; exitting")
                    quit()
                vid = int(viddata, 16)
                world.getActor(vid)
                control = world.get_control()
                player = world.get_player()
                viddata = None

        vid_thread = threading.Thread(target = handle_vid_sock, args = (vidsock, vid))
        vid_thread.setDaemon(True) # kill this when parent dies
        vid_thread.start()

        external_gears    = ["P", "R", "N", "D", "L"]
        external_gear_idx = 0
        last_gear = 0
        last_ignition = 0
        ignition_on = False

        if isinstance(control, carla.VehicleControl):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            except AttributeError:
                pass
            sock.setblocking(True)
            sock.bind((args.rec_ip, args.rec_port))
            sock.settimeout(1)

            control.manual_gear_shift = 1
            physCtrl = player.get_physics_control()
            maxGear = len(physCtrl.forward_gears) - 1
            lastGearData = 0
            while True:
                if g_kill:
                    print("Got kill message; exitting")
                    quit()
                global g_new_vehicle_id_available
                if g_new_vehicle_id_available:
                    world.getActor(g_new_id)
                    control = world.get_control()
                    control.manual_gear_shift = 1
                    player = world.get_player()
                    g_new_vehicle_id_available = False

                try:
                    data = sock.recv(1024)
                except socket.timeout:
                    data = None

                if data is None:
                    continue

                can = data.split(b" ") #id , datasize, data
                
                id = int(can[0], 16)
                dataSize = int(can[1], 16) #don't need datasize as we know always 8 for SLCAN
                data = int(can[2], 16)
                
                #logging.debug("Receive id: 0x%x, size: %x, data: %x", id, dataSize, data)
                
                # TODO range on these same betwen SLCAN and EXU? I think so but basing EXU off of trial and error
                if  (id == carlaIDMap['throttle']):
                    control.throttle = data / 0x3FF # scale to 0 - 1
                elif (id == carlaIDMap['steer']):
                    # 2's compliment convert on 2 bytes
                    if (data & (1 << (16 - 1))) != 0:
                        data = data - (1 << 16)
                    control.steer = data / 0x1FE # scale to -1 - 1
                elif (id == carlaIDMap['brake']):
                    control.brake = data / 0x3FF # scale to 0 - 1
                    if control.brake > 0: # TODO cheating a little bit doing this instead of based on EXU message
                        brakeLights |= carla.VehicleLightState.Brake
                    else: 
                        brakeLights &= carla.VehicleLightState.All ^ carla.VehicleLightState.Brake
                # in mode 1 for all other values looking for command for just gear want report 
                #elif id == 119: #in mode 1 109 and 119 are the same?
                elif (id == carlaIDMap['gear']):
                    if last_gear != data:
                        if   data == 1: # gear up   : L towards P
                            external_gear_idx = max(0, external_gear_idx - 1) # can't go past P
                        elif data == 2: # gear down : P towards L
                            external_gear_idx = min(len(external_gears) - 1, external_gear_idx + 1)
                    last_gear = data
                    data = external_gear_idx + 1
                    # comment out above if applying directly
                    #print("Inc:", last_gear, "gear:", external_gears[external_gear_idx])
                    control.reverse = False
                    if   data == 1: # P
                        control.gear = 0
                    elif data == 2: # R
                        control.reverse = True
                        control.gear = -1
                    elif data == 3: # N
                        control.gear = 0
                    elif data == 4: # D TODO? should switch off manual shift for this and L?
                        control.gear = 3
                    elif data == 5:
                        control.gear = 1 # L
                    else:
                        logging.debug("Bad value for gear %d", data)

                    if control.reverse:
                        brakeLights |= carla.VehicleLightState.Reverse
                    else:
                        brakeLights &= carla.VehicleLightState.All ^ carla.VehicleLightState.Reverse
                # https://carla.readthedocs.io/en/latest/python_api/#carlavehiclelightstate
                elif (id == carlaIDMap['lightFront']):
                    #print("light front ", hex(id), ":", data)
                    if   data == 0:
                        frontLights = carla.VehicleLightState.NONE
                    elif data == 1:
                        frontLights = carla.VehicleLightState.Position
                    elif data == 3:
                        frontLights = carla.VehicleLightState.LowBeam  | carla.VehicleLightState.Position
                    elif data == 5:
                        frontLights = carla.VehicleLightState.HighBeam | carla.VehicleLightState.Position
                    elif data == 7:
                        frontLights = carla.VehicleLightState.HighBeam | carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position
                    elif data == 6: # passing
                        frontLights = carla.VehicleLightState.HighBeam
                    else:
                        logging.error("Unknown front light state: " + str(data))
                elif (id == carlaIDMap['lightTurn']):
                    #print("light turn ", id, ":", data)
                    if   data == 0:
                        turnLights = carla.VehicleLightState.NONE
                    elif data == 1:
                        turnLights = carla.VehicleLightState.LeftBlinker 
                    elif data == 2:
                        turnLights = carla.VehicleLightState.RightBlinker 
                    elif data & 4 != 0:
                        turnLights = carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.RightBlinker 
                elif (id == carlaIDMap['hand_brake']):
                    control.hand_brake = data==1
                #elif (id == carlaIDMap['ignition']):
                #    if last_ignition == 0 and data == 1:
                #        ignition_on = not ignition_on
                #    last_ignition = data
                elif (id == carlaIDMap['engine']):
                    ignition_on = data
                #else:
                #    logging.error(f"Unknown ID {id}")
                currentLight = frontLights | turnLights | brakeLights
                player.set_light_state(carla.VehicleLightState(currentLight))

                if not ignition_on:
                    control.throttle = 0

                if player.get_control().gear == 1 and (external_gear_idx != 4):
                    if control.throttle != 1.0:
                        control.throttle += 0.000001
                    else:
                        control.throttle -= 0.000001

                player.apply_control(control)

        else:
            print("This actor id is not a vehicle\n")

    finally:
        print("Exiting")


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
        '--rec_ip',
        default='127.0.0.1',
        help='Receive IP (default: 127.0.0.1)')
    argparser.add_argument(
        '--rec_port',
        metavar='P',
        default=3001,
        type=int,
        help='TCP port to listen to (default: 3001)')
    argparser.add_argument(
        '--vid_ip',
        default='127.0.0.1',
        help='Vehicle ID Receive IP (default: 127.0.0.1)')
    argparser.add_argument(
        '--vid_port',
        default=3002,
        type=int,
        help='TCP port to listen to (default: 3002)')
    argparser.add_argument(
        '--id',
        dest='actor_id',
        type=int,
        help='[Optional] intial value to use for actor ID. Will be overwritten by IDs incoming on UDP connection on vid_ip:vid_port'
    )
    argparser.add_argument(
        '--carla_host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '--carla_port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--can_id',
        default='CAN_ID.json',
        help='Path to file containing CAN_IDs for messages (default CAN_ID.json in current directory)'
    )
    argparser.add_argument(
        '--use_ECU_Tester',
        action='store_true',
        default=False,
        help='Wheter to receive command messages (default) or report.'
    )
    argparser.add_argument(
        '--mode',
        default=1,
        type = int,
        choices=[1],
        help='Mode to opearate in see main.py for full details.'
    )
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s and %s:%s', args.rec_ip, args.rec_port, args.vid_ip, args.vid_port)

    global carlaIDMap, carlaIDMapRPT
    # load the CAN-IDs with report tag for variables we care about
    # (want to read status not change it)
    canType = 'report' if args.use_ECU_Tester else 'command'
    carlaIDMap = load_CAN_ID(canType, args.can_id)
    # bit messy but need the engine report in all isntances
    if 'engine' not in carlaIDMap:
        carlaIDMapRPT = load_CAN_ID('report', args.can_id)
        carlaIDMap['engine'] = carlaIDMapRPT['engine']

    try:
        input_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
