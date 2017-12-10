#!/usr/bin/python
# -*- coding: utf-8 -*-
# The MIT License (MIT)
# Copyright (c) 2016 Bruno Tibério
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

""" Module Razor IMU

:Date: 22 Dez 2016

:Version: 0.1

:Author: Bruno Tibério

:Contact: bruno.tiberio@tecnico.ulisboa.pt


This module contains a few functions to interact with Sparkfun Razor 9DOF IMU.

"""
import sys
from datetime import datetime
import logging
import os
import serial
import struct
import threading
from time import sleep
import crcmod.predefined

if sys.version_info.major == (3):
    import queue
else:
    import Queue


class RazorIMU(object):
    '''
    classdocs
    '''
    keys = ('syncAA', 'syncBB', 'ID', 'Acc', 'Gyro', 'Mag', 'euler', 'crc8maxim')

    def __init__(self, sensorName="RazorIMU 1"):
        '''
        Constructor
        '''
        self.name = sensorName
        self.myPort = ""
        self.isOpen = 0  # is port open?*/
        self.baudRate = 500000  # current communication baudRate*/
        self.openError = 0  # if any error during any process occurs this will be set */
        self.logFile = ""
        self.dataQueue = None
        self.exitFlag = threading.Event()
        if sys.version_info.major == (3):
            self.orders = queue.Queue()
        else:
            self.orders = Queue.Queue()

    def crc8Value(self, i):
        crc8_maxim = crcmod.predefined.mkPredefinedCrcFun('crc-8-maxim')
        return crc8_maxim(i)

    def begin(self, dataQueue,
              comPort="/dev/ttyUSB0",
              baudRate=500000):
        ''' Initializes the Sparkfun razor IMU.

        This function resets the current port to factory default and setup the
        gps receiver to be able to acept new commands. Also creates the
        necessary log files used to save data transmited. If connection to razor
        is made, it launchs a thread used to parse messages comming from razor.

        Args:
            comPort: system port where receiver is connected.
            dataQueue: a Queue object to store incoming razor messages.
            baudRate: baudrate to configure port.

        Returns:
            True or False if the setup has gone as expected or not.

        :Example:
          .. code-block:: python

              razor.begin(comPort="<port>",
                  dataQueue=<your Queue obj>,
                  baudRate=50000)

        **Default values**

        :comPort:  "/dev/ttyUSB0"
        :baudRate:  500000

        .. warning::
            This class uses module ``logging`` wich must be configured in your
            main program using the ``basicConfig`` method. Check documentation
            of `module logging`_ for more info.

        .. _module logging: https://docs.python.org/2/library/logging.html

        '''

        self.log = logging.getLogger(self.name)

        # checking if port exists on system
        if not os.path.exists(comPort):
            self.log.warning('Port is not available: {0}'.format(comPort))
            return False
        else:
            # port exists, open it
            self.myPort = serial.Serial(comPort, baudrate=baudRate, exclusive=True)
            if not self.myPort.is_open:
                self.log.warning("Error opening port: {0}".format(comPort))
                self.isOpen = False
                return False
            self.baudRate = baudRate
            self.isOpen = True
            # open dataFile to save GPS data
            self.dataQueue = dataQueue
            # start thread to handle GPS responces
            self.threadID = threading.Thread(name="Logger", target=self.parseResponces)
            self.threadID.start()
            self.log.info("Started Logger Thread")
            sleep(0.1)
            return True

    def parseResponces(self):
        '''
        A thread  to parse responses from device
        '''
        self.log.info("Entering Thread logger")
        if(not self.isOpen):
            self.log.warning('Port is not open: {0}'.format(self.myPort))
            self.log.info("Exiting Thread logger")
            return
        MYPORT = self.myPort
        # dataFile = self.dataFile
        indice = 0
        while(self.exitFlag.isSet() == False):
            syncAA = 0
            syncBB = 0
            dataFrame = [0] * 8
            syncAA = ord(MYPORT.read(1))
            if(syncAA == 0xAA):
                syncBB = ord(MYPORT.read(1))
                if(syncBB == 0xBB):
                    # correct sync bytes received
                    # read rest of the packet 52bytes - 2 sync bytes
                    serialBuffer = MYPORT.read(50)
                    dataFrame[0] = syncAA
                    dataFrame[1] = syncBB
                    # get ID Index Acc Gyro Mag CRC8
                    dataFrame[2] = struct.unpack_from('<B', serialBuffer, 0)
                    # to remove tuples
                    dataFrame[2] = dataFrame[2][0]
                    # get acc floats
                    dataFrame[3] = [0, 0, 0]
                    dataFrame[3][0:] = struct.unpack_from('<fff', serialBuffer, 1)
                    # get gyro floats
                    dataFrame[4] = [0, 0, 0]
                    dataFrame[4][0:] = struct.unpack_from('<fff', serialBuffer, 13)
                    # get mag floats
                    dataFrame[5] = [0, 0, 0]
                    dataFrame[5][0:] = struct.unpack_from('<fff', serialBuffer, 25)
                    # get euler floats
                    dataFrame[6] = [0, 0, 0]
                    dataFrame[6][0:] = struct.unpack_from('<fff', serialBuffer, 37)
                    # get crc8 maxim
                    dataFrame[7] = struct.unpack_from('<B', serialBuffer, 49)
                    dataFrame[7] = dataFrame[7][0]
                    dataFrame = dict(zip(self.keys, dataFrame))
                    # add timestamp
                    currentTime = datetime.now()
                    myTime = '{0:%Y-%m-%d %H:%M:%S}'.format(currentTime) + '.{0:03.0f}'.format(round(currentTime.microsecond / 1000.0))
                    outMessage = dict(Time=myTime, Indice=indice)
                    outMessage.update(dataFrame)
                    self.dataQueue.put_nowait(outMessage)
                    indice = indice + 1
                else:
                    pass
            else:
                # self.log.info("New Byte received: 0x{0:X}".format(syncAA))
                pass
        self.log.info("Exiting Thread logger")
        return

    def shutdown(self):
        '''Prepare for exiting program

        Returns:
            always returns true after all tasks are done.

        * exit logger thread
        * close port

        '''
        self.exitFlag.set()
        self.threadID.join()
        self.myPort.close()
        self.isOpen = False
        self.log.info("Shuting down")
        return True


def main():
    '''Set of test to run to see if class behaves as expected.

    Creates a razor class object and execute the following commands:

    - begin: on default port or given port by argv[1].
    - print messages received from razor
    - wait for 10 seconds
    - shutdown: safely disconnects.

    '''
    import optparse

    def printData(dataQueue, exitFlag):
        ''' prints data to console

        Thread used to print data from razor to the console.

        Args:
            dataQueue: queue class object where data is stored
            exitFlag: a flag to control the exit of program gracefully

        '''
        print("Index,Time,ID,accx,accy,accz,gyrox,gyroy,gyroz,magx,magy,magz,psi,theta,phi,crc8maxim\n")
        while(exitFlag.isSet() == False):
            if(dataQueue.empty() == False):
                newData = dataQueue.get()
                print('{0:5d},{1},{2},{3:.2f},{4:.2f},{5:.2f},{6:.2f},'
                      '{7:.2f},{8:.2f},{9:.2f},{10:.2f},{11:.2f},'
                      '{12:.2f},{13:.2f},{14:.2f},{15}'
                      '\n'.format(newData['Indice'],
                                  newData['Time'],
                                  newData['ID'],
                                  newData['Acc'][0],
                                  newData['Acc'][1],
                                  newData['Acc'][2],
                                  newData['Gyro'][0],
                                  newData['Gyro'][1],
                                  newData['Gyro'][2],
                                  newData['Mag'][0],
                                  newData['Mag'][1],
                                  newData['Mag'][2],
                                  newData['euler'][0],
                                  newData['euler'][1],
                                  newData['euler'][2],
                                  newData['crc8maxim']
                                  ))
            else:
                sleep(0.01)
        return
    ############################################################################
    #
    # Start of main part
    #
    ############################################################################
    parser = optparse.OptionParser(usage="usage: %prog [options] args")
    parser.add_option("-p", "--port", action="store", type="string",
                      dest="port", default="/dev/ttyUSB0")
    parser.add_option("-n", "--name", action="store", type="string",
                      dest="name", default="RAZOR1")
    parser.add_option("--log", action="store", type="string",
                      dest="log", default="output.log")
    parser.add_option("--log-level", action="store", type="int",
                      dest="logLevel", default=20)
    (opts, args) = parser.parse_args()
    if len(args) > 4:
        parser.error("incorrect number of arguments")
        return

    logging.basicConfig(filename=opts.log,
                        level=opts.logLevel,
                        format='[%(asctime)s] [%(threadName)-10s] %(levelname)-8s %(message)s',
                        filemode="w")
    # event flag to exit
    exitFlag = threading.Event()
    # create a queue to receive comands
    dataFIFO = Queue.Queue()
    # create a thread to parse responses
    thread1 = threading.Thread(name="printData", target=printData,
                               args=(dataFIFO, exitFlag))

    # instanciate a class object
    razor = RazorIMU(opts.name)
    # begin
    if(razor.begin(dataFIFO, comPort=opts.port) != 1):
        print("Not able to begin device properly... check logfile")
        return
    thread1.start()
    # wait 10 seconds
    sleep(10)
    razor.shutdown()
    # stop print thread
    exitFlag.set()
    thread1.join()
    logging.shutdown()
    # exit
    print('Exiting now')
    return


if __name__ == '__main__':
    main()
