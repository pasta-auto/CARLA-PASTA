import sys
import subprocess
import argparse
import serial
import glob
import os
from common import ErrorCode
import random
import itertools

try:
    sys.path.append(glob.glob(os.path.join(sys.path[0],'../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64')))[0])
except IndexError:
    pass
import carla

def test_COM_port(port):
    try:
        ser = serial.serial_for_url(port)
    except Exception as e:
        print("Error: could not connect to COM port \"" + port + "\". Please check that the COM port url is correct, the device is connected and no other process is using the port.")
        quit(ErrorCode.COM.value)

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description='PASTA - CARLA')
    # NOTE: explicitly taking defaults out of the arg parser here as that will be handled by the components
    argparser.add_argument(
        '--carla_port',
        type=int,
        default=2000,
        help='TCP port of the CARLA server (default: 2000)')
    argparser.add_argument(
        '--can_port',
        type=int,
        default=3000,
        help='UDP port of the CAN UDP server (default: 3000)')
    argparser.add_argument(
        '--data_in_port_can',
        type=int,
        default=3001,
        help='UDP port to send commands to (default: 3001)')
    argparser.add_argument(
        '--data_in_port_actor_id',
        type=int,
        default=3002,
        help='UDP port to send actor ID to (default: 3002)')
    argparser.add_argument(
        '--manual_control_port',
        type=int,
        default=3003,
        help='UDP port to send data from PASTA to the HUD (default: 3003)')
    argparser.add_argument(
       '--disable_light_state_display',
        action='store_true',
        default=False,
        help='Enables the light state display on the HUD (textual display of which lights are currently on.) Note this display lowers framerate substantially so reccomended to specify this option if not using the display. It is only helpful for cars without lights in the model. Default:Display is on.' 
    )
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--apply_control',
        action='store_true',
        default=False,
        help='Sends CARLA commands directly instead of going through PASTA; because of this, data_in is not started.'
    )
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=0,
        type=int,
        help='number of vehicles (default: 0)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=0,
        type=int,
        help='number of walkers (default: 0)')
    argparser.add_argument(
       '--lfa6u',
        metavar='COM_URL',
        help='COM port URL for lfa6u device'
    )
    argparser.add_argument(
       '--mode',
       default=1,
       choices=[0, 1],
       type=int,
       help='Mode to opearate in. For mode 0 CARLA operates independently from PASTA. For mode 1 PASTA controls the vehicle and communication happens to synchornize PASTA / CARLA state via a LFA6U connection'
    )
    args = argparser.parse_args()

    if args.mode == 0: # mode 0 directly apply control without PASTA communication
        args.apply_control = True

    # check that the lfa6u is connected properly
    if args.mode == 1:
        if not args.lfa6u and not args.apply_control: # only need to specify lfa6u when not doing apply_control
            argparser.error('No COM device of lfa6u specified add --lfa6u with the COM port URL. Example --lfa6u COM3')
            argparser.exit()
        if args.lfa6u: # if provided make sure it works
            test_COM_port(args.lfa6u)

    # check that can connect to carla
    try:
        client = carla.Client('127.0.0.1', args.carla_port)
        client.set_timeout(2.0)
        world = client.get_world()
    except RuntimeError as e:
        print("Error: no response from CARLA server:", e, "and also there is no firewall settings preventing the connection.")
        quit(ErrorCode.CARLA.value)

    manual_control_args = ''
    can_udp_server_args = ''
    data_in_args        = ''
    spawn_npc_args      = ''

    # TODO don't need the if around some of these if having a default in anyways; 
    #      added default for ports as theoretically want to do tests before opening subwindows
    if args.carla_port:
        manual_control_args += ' --port '       + str(args.carla_port)
        data_in_args        += ' --carla_port ' + str(args.carla_port)
    if args.can_port:
        manual_control_args += ' --can_port ' + str(args.can_port)
        can_udp_server_args += ' --port '     + str(args.can_port)
    if args.data_in_port_can:
        can_udp_server_args += ' --data_in_port ' + str(args.data_in_port_can)
        data_in_args        += ' --rec_port '     + str(args.data_in_port_can)
    if args.data_in_port_actor_id:
        manual_control_args += ' --data_in_port ' + str(args.data_in_port_actor_id)
        data_in_args        += ' --vid_port '     + str(args.data_in_port_actor_id)
    if args.manual_control_port:
        manual_control_args += ' --manual_control_port ' + str(args.manual_control_port)
        can_udp_server_args += ' --manual_control_port ' + str(args.manual_control_port)
    if args.lfa6u:
        can_udp_server_args += ' --lfa6u ' + args.lfa6u
    if args.disable_light_state_display:
        manual_control_args += ' --disable_light_state_display'
    if args.filter:
        filts = args.filter.split(",")
        filteredLibs = []
        for i in filts:
            actors = world.get_blueprint_library().filter(i)
            if actors:
                filteredLibs.append(actors)
        filteredLibs = list(itertools.chain.from_iterable(filteredLibs))
        finalLibs = []
        # remove any non vehicles and motorcycles
        for i in filteredLibs:
            try:
                numWheels = i.get_attribute("number_of_wheels")
                if numWheels.as_int() > 2:
                    finalLibs.append(i)
            except Exception as e:
                print(e)
                pass
        filteredLibs = finalLibs
        if filteredLibs == []:
            print("Error: no vehicles for filter:", args.filter, ". Please specify a less restricitve filter.")
            quit(ErrorCode.FILTER.value)

        manual_control_args += ' --filter \"' + args.filter + '\"'
    if args.apply_control:
        manual_control_args += ' --apply_control'

    manual_control_args += ' --mode ' + str(args.mode)
    can_udp_server_args += ' --mode ' + str(args.mode)
    data_in_args        += ' --mode ' + str(args.mode)

    manual_control_args += ' --send_kill_on_quit'

    spawn_npc_args += ' -n ' + str(args.number_of_vehicles) + ' -w ' + str(args.number_of_walkers)

    manual_control_final = sys.executable + " " + os.path.join(sys.path[0], "manual_control.py ")  + manual_control_args
    can_udp_server_final = sys.executable + " " + os.path.join(sys.path[0], "can_udp_server.py ")  + can_udp_server_args
    data_in_final        = sys.executable + " " + os.path.join(sys.path[0], "data_in_can_udp.py ") + data_in_args
    spawn_npc_final      = sys.executable + " " + os.path.join(sys.path[0], "spawn_npc.py ")       + spawn_npc_args

    print("Launching manual control: " + manual_control_final)
    subprocess.Popen(manual_control_final, creationflags=subprocess.CREATE_NEW_CONSOLE)

    if (args.lfa6u) is not None: # only start if using lfa6u
        print("Launching can udp server: " + can_udp_server_final)
        subprocess.Popen(can_udp_server_final, creationflags=subprocess.CREATE_NEW_CONSOLE)

    if not args.apply_control:
        print("Launching datain can udp: " + data_in_final)
        subprocess.Popen(data_in_final   , creationflags=subprocess.CREATE_NEW_CONSOLE)

    if args.number_of_vehicles > 0 or args.number_of_walkers > 0:
        print("Launching spawn npc: " + spawn_npc_final)
        subprocess.Popen(spawn_npc_final , creationflags=subprocess.CREATE_NEW_CONSOLE)
