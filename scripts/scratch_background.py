#!/usr/bin/python
# This code is copyright Dawn Robotics under GPL v2 12/Nov/2013
# derived from Graham Taylor under GPL v2 1/Sep/2013
# derived from Simon Walters under GPL v2
# derived from scratch_handler by Thomas Preston

import socket
import time
import sys
import errno
import multiprocessing
import math

PORT = 42001
DEFAULT_HOST = '127.0.0.1'
SOCKET_TIMEOUT = 1

ROVER_START_X = 0
ROVER_START_Y = -150
ROVER_START_HEADING_DEGREES = 0


#---------------------------------------------------------------------------------------------------
def byte2int(bstr, width=32):
    val = sum(ord(b)<< 8*n for (n, b) in enumerate(reversed(bstr)))
    if val >= (1 << (width - 1)):
        val = val - (1 << width)
    return val

#---------------------------------------------------------------------------------------------------
def int2byte(val, width=32): 
    if val < 0: 
        val = val + (1 << width) 
    return ''.join([chr((val >> 8*n) & 255) for n in reversed(range(width/8))]) 

#---------------------------------------------------------------------------------------------------
def isNumeric(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

cycle_trace = "inactive"

#---------------------------------------------------------------------------------------------------
class ScratchBase( multiprocessing.Process ):
    def __init__( self, socket ):
        multiprocessing.Process.__init__( self )
        self.scratchSocket = socket
        self._stop = multiprocessing.Event()

    def stop( self ):
        self._stop.set()

    def stopped( self ):
        return self._stop.is_set()

#---------------------------------------------------------------------------------------------------
class ScratchListener( ScratchBase ):
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, socket, commandQueue ):
        ScratchBase.__init__( self, socket )
        self.commandQueue = commandQueue
        
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        global cycle_trace

        while not self.stopped():
            try:
                read_len = self.scratchSocket.recv(4)
                packet = self.scratchSocket.recv(byte2int(read_len))
                if len(packet) == 0:
                    if cycle_trace == 'running':
                        cycle_trace = 'disconnected'
                        break

            except socket.timeout:
                #print "Error timeout in receiver"
                continue

            print "got packet", packet

            if packet.startswith('sensor-update'):
                #get sensor_name and sensor_value
                #sensor_name,sp,sensor_value = packet[14:].strip().rpartition(' ')
                #sensor_name = sensor_name.strip('"')
                #val = sensor_value.lower()

                ##print '[' + sensor_name + '] = [' + val + ']'
                #if sensor_name=='allpins':
                #if val=='1' or val=='on' or val=='high':
                    #for key in OUTPUTS.keys():
                        #self.physical_pin_update(key,1)
                #if val=='0' or val=='off' or val=='low':
                    #for key in OUTPUTS.keys():
                        #self.physical_pin_update(key,0)

                #for key in OUTPUTS.keys():
                    #if sensor_name==key:
                        #if val=='1' or val=='on' or val=='high':
                            #self.physical_pin_update(key,1)
                        #if val=='0' or val=='off' or val=='low':
                            #self.physical_pin_update(key,0)
                        
                continue

            if packet.startswith('broadcast'):
                #get broadcast message
                m = packet[11:-1].lower()
                #print 'message = ' + m

                print "m is", m

                self.commandQueue.put( m )

#---------------------------------------------------------------------------------------------------
def createSocket(host, port):
    while True:
        try:
            print 'Trying'
            scratch_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            scratch_sock.connect((host, port))
            break
        except socket.error:
            print "Error connecting to Scratch!"
            print "No Mesh session at host: %s, port: %s please Enable remote sensor connections." % (host, port) 
            time.sleep(3)

    return scratch_sock

#---------------------------------------------------------------------------------------------------
def cleanupProcesses( processes ):
    for process in processes:
        process.stop()

    for process in processes:
        process.join()

