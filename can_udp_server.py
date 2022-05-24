#!/usr/bin/env python

import argparse
import serial
import serial.threaded
import socket
import logging
import time
import io
import threading
import traceback
import datetime
import json
import atexit

from multiprocessing import Process
            
from common import load_CAN_ID
carlaIDMapCMD = {}
carlaIDMapRPT = {}

Args = None
g_quit = False
            
class CanHandler(serial.threaded.LineReader):
    TERMINATOR = b'\r'
    ENCODING = 'ascii'
    
    sockOut = None
    outAddr = None
    # LFA6U only have 1 line so need to read ID and determine where to send based on that
    hudAddr = None 
    hudIDs = [] # still sending some to HUD some to data_in but don't need to switch on IDs
    fullSLCANDict = None

    DEBUG_name = None
    DEBUG_id = None

    def connection_made(self, transport):
        super(CanHandler, self).connection_made(transport)
        print("Serial port opened", transport.name)

        self.sockOut = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockOut.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
        
    def connection_lost(self, exc):
        if exc:
            print(exc)
        print("Serial port closed")
    
    # just use write_line to handle writing

    def handle_line(self, input):
        if input == None or len(input) < 5:
            return
        if (input[0] == 't'):
            #logging.debug("Received %s", input)
            id       = int(input[1:4], 16)
            dataSize = int(input[4]  , 16) # don't really need this always 8 for SLCAN messages
            rawData  = input[5:] # doing conversion here makes handling things between HUD and not easier 
            try: # try and find data size for the particular ID
                dataSForID = self.fullSLCANDict['can_id'][str(id)]['datasize']
            except KeyError as e:
                # print("ID not in CAN-ID json: ", e)
                return
            try:
                # only want the first datasize bytes
                reducedData   = rawData[0:dataSForID*2] # *2 for hex digits
            except Exception as e:
                print(e)
                return
            if id in self.hudIDs: # if in the HUD id list also send it there
                self.sockOut.sendto(bytes("{:03x} {:01x} {:s}".format(id, 8, reducedData), "ascii"), self.hudAddr)

            self.sockOut.sendto(bytes("{:03x} {:01x} {:s}".format(id, 8, reducedData), "ascii"), self.outAddr)
            if self.DEBUG_id and id == self.DEBUG_id:
                logging.debug("Rec: " + str(input) + " -> " + str(id) + " : " + str(reducedData))
        else:
            logging.debug("Received unknown message %s", input)

class CanServer(object):
    lfa6uSer      = None

    lfa6uCanHandler      = None 

    sockIn  = None
    sockOut = None

    eventRec  = None
    eventSend = None

    recT = None

    def __init__(self):
        self.eventRec  = threading.Event()
        self.eventSend = threading.Event()

        def cleanup():
            print("Exitting closing serial ports.")

            if Args.lfa6u != None:
                if self.lfa6uCanHandler.is_alive():
                    self.lfa6uCanHandler.stop()
                if self.lfa6uSer.isOpen():
                    self.lfa6uSer.write(bytes("C\r", "ascii"))
                    self.lfa6uSer.close()

        atexit.register(cleanup)

    def serverLoop(self):
           
        if Args.lfa6u:
            try:
                self.lfa6uSer = serial.serial_for_url(Args.lfa6u)
            except Exception as e:
                print("Could not open specified lfa6u:", e)
                quit()

            self.lfa6uSer.write(bytes("O1\r", "ascii"))
            self.lfa6uSer.write(bytes("S6\r", "ascii"))
            lfa6uModeCommand = "t7D0800FF0" + str(Args.mode) + "FFFFFFFFFF\r"
            self.lfa6uSer.write(bytes(lfa6uModeCommand, "ascii"))
            lfa6uCanHandlerType = type('lfa6uCanHandlerType', (CanHandler,), {})
            lfa6uCanHandlerType.outAddr = (Args.data_in_ip       , Args.data_in_port)
            lfa6uCanHandlerType.hudAddr = (Args.manual_control_ip, Args.manual_control_port)
            # send all command IDs to manual_control HUD instead of data_in
            lfa6uCanHandlerType.hudIDs = [v for k,v in carlaIDMapCMD.items()]
            lfa6uCanHandlerType.hudIDs.append(carlaIDMapRPT['engine'])
            data = ''
            with open(Args.can_id, 'r') as file:
                data = file.read()
            lfa6uCanHandlerType.fullSLCANDict = json.loads(data)
            if Args.debug_rec_id:
                lfa6uCanHandlerType.DEBUG_id = Args.debug_rec_id
            self.lfa6uCanHandler = serial.threaded.ReaderThread(self.lfa6uSer, lfa6uCanHandlerType)
            self.lfa6uCanHandler.start()

        self.sockIn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockIn.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sockIn.setblocking(True)
        self.sockIn.bind((Args.host, Args.port))
        
        self.recT = threading.Thread(target=self.receiveThread, group=None, daemon=True)

        self.recT.start()
        while(True):
            time.sleep(1)
            if g_quit:
                return
            
            
    def receiveThread(self):
        while(True):
            recData = None
            try:
                while(True):
                    recData = self.sockIn.recv(1024) #1024 buffer size
                    
                    # still going to receive just so manual control has a place to send to 
                    if (recData != None): 
                        if recData == b"kill":
                            print("Got kill message; exitting")
                            global g_quit
                            g_quit = True
                            quit()
                        if Args.lfa6u:
                            can = recData.split(b" ") #id , datasize, data
                            id = int(can[0], 16)

                            data = can[1].decode("ascii")
                            datahexsize = len(data)
                            remainderHex= 16 - datahexsize 
                            formatedStr = "t{:03x}{:01x}{:s}{:0{remainder}x}".format(id, 8, data, 0, remainder=remainderHex)
                            if Args.debug_send_id and Args.debug_send_id == id:
                                logging.debug("sent: " + formatedStr)
                            self.lfa6uSer.write(bytes(formatedStr+"\r\n", "ascii"))

            except Exception as e:
                logging.error("Data processing error")
                logging.error(e)
                quit()

    def sendThreadlfa6u(self):
        self.lfa6uCanHandler.run()
            

def main():
    argparser = argparse.ArgumentParser(
        description='CAN UDP Server')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=3000,
        type=int,
        help='UDP port to listen to (default: 3000)')
    argparser.add_argument(
        '--data_in_port',
        metavar='P',
        default=3001,
        type=int,
        help='UDP port to send to (default: 3001)')
    argparser.add_argument(
        '--manual_control_port',
        type=int,
        default=3003,
        help='UDP port to send data from PASTA to the HUD (default: 3003)')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '--data_in_ip',
        default='127.0.0.1',
        help='IP of data_in_can_udp (default: 127.0.0.1)')
    argparser.add_argument(
        '--manual_control_ip',
        default='127.0.0.1',
        help='IP of manual_control (default: 127.0.0.1)')
    argparser.add_argument(
       '--lfa6u',
        metavar='COM_URL',
        help='COM port URL for lfa6u device'
    )
    argparser.add_argument(
        '--can_id',
        default='CAN_ID.json',
        help='Path to file containing CAN_IDs for messages (default CAN_ID.json in current directory)'
    )
    argparser.add_argument(
       '--debug_send_id',
        help='print send messages for given ID',
        type=int
    )
    argparser.add_argument(
       '--debug_rec_id',
        help='print recieved messages for given ID',
        type=int
    )
    argparser.add_argument(
        '--mode',
        default=1,
        type = int,
        choices=[1],
        help='Mode to opearate in see main.py for full details.'
    )
    global Args
    Args = argparser.parse_args()
    
    if not (Args.lfa6u):
        argparser.error('No COM device specified add--lfa6u with the COM port URL. Example --lfa6u COM3')
        argparser.exit()

    global carlaIDMapCMD, carlaIDMapRPT
    # load the CAN-IDs with report tag for variables we care about
    # (want to read status not change it)
    carlaIDMapCMD = load_CAN_ID('command', Args.can_id)
    carlaIDMapRPT = load_CAN_ID('report' , Args.can_id)

    log_level = logging.DEBUG if Args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening on %s:%s', Args.host, Args.port)
    logging.info('Sending   to %s:%s', Args.data_in_ip, Args.data_in_port)
    
    server = CanServer()
    try:
        server.serverLoop()
    except KeyboardInterrupt:
        print('Keyboard interrupt, Server closing')

if __name__ == '__main__':
    # p = Process(target=main)
    #p.start()
    #p.join()
    main()
