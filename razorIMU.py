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

:Version: 0.2

:Author: Bruno Tibério

:Contact: bruno.tiberio@tecnico.ulisboa.pt


This module contains a few functions to interact with Sparkfun Razor 9DOF IMU.

"""
from datetime import datetime
import logging
import os
import serial
import struct
import threading
from time import sleep
import crcmod.predefined

try:
    import queue
except ImportError:
    import Queue as queue


class RazorIMU(object):
    """
    classdocs
    """
    keys = ('syncAA', 'syncBB', 'ID', 'Acc', 'Gyro', 'Mag', 'euler', 'crc8maxim')

    def __init__(self, sensor_name="RazorIMU 1"):
        """
        Constructor
        """
        self.name = sensor_name
        self.device = ""
        self.isOpen = 0  # is port open?*/
        self.baudRate = 500000  # current communication baudRate*/
        self.openError = 0  # if any error during any process occurs this will be set */
        self.logFile = ""
        self.dataQueue = None
        self.exitFlag = threading.Event()
        self.orders = queue.Queue()
        self.log = None
        self.threadID = None

    @staticmethod
    def crc8_value(i):
        crc8_maxim = crcmod.predefined.mkPredefinedCrcFun('crc-8-maxim')
        return crc8_maxim(i)

    def begin(self, data_queue,
              com_port="/dev/ttyUSB0",
              baud_rate=500000):
        """ Initializes the Sparkfun razor IMU.

        This function resets the current port to factory default and setup the
        gps receiver to be able to acept new commands. Also creates the
        necessary log files used to save data transmited. If connection to razor
        is made, it launchs a thread used to parse messages comming from razor.

        Args:
            com_port: system port where receiver is connected.
            data_queue: a Queue object to store incoming razor messages.
            baud_rate: baud rate to configure port.

        Returns:
            True or False if the setup has gone as expected or not.

        :Example:
          .. code-block:: python

              razor.begin(com_port="<port>",
                  data_queue=<your Queue obj>,
                  baud_rate=50000)

        **Default values**

        :com_port:  "/dev/ttyUSB0"
        :baud_rate:  500000

        .. warning::
            This class uses module ``logging`` which must be configured in your
            main program using the ``basicConfig`` method. Check documentation
            of `module logging`_ for more info.

        .. _module logging: https://docs.python.org/3/library/logging.html

        """

        self.log = logging.getLogger(self.name)

        # checking if port exists on system
        if not os.path.exists(com_port):
            self.log.warning('Port is not available: {0}'.format(com_port))
            return False
        else:
            # port exists, open it
            self.device = serial.Serial(com_port, baudrate=baud_rate, exclusive=True)
            if not self.device.is_open:
                self.log.warning("Error opening port: {0}".format(com_port))
                self.isOpen = False
                return False
            self.baudRate = baud_rate
            self.isOpen = True
            # open dataFile to save GPS data
            self.dataQueue = data_queue
            # start thread to handle GPS responces
            self.threadID = threading.Thread(name="Logger", target=self.parseResponses)
            self.threadID.start()
            self.log.info("Started Logger Thread")
            sleep(0.1)
            return True

    def parseResponses(self):
        """
        A thread  to parse responses from device
        """
        self.log.info("Entering Thread logger")
        if not self.isOpen:
            self.log.warning('Port is not open: {0}'.format(self.device))
            self.log.info("Exiting Thread logger")
            return
        device = self.device
        # dataFile = self.dataFile
        index = 0
        while not self.exitFlag.isSet():
            syncAA = 0
            syncBB = 0
            data_frame = [0] * 8
            syncAA = ord(device.read(1))
            if syncAA == 0xAA:
                syncBB = ord(device.read(1))
                if syncBB == 0xBB:
                    # correct sync bytes received
                    # read rest of the packet 52bytes - 2 sync bytes
                    buffer = device.read(50)
                    data_frame[0] = syncAA
                    data_frame[1] = syncBB
                    # get ID Index Acc Gyro Mag CRC8
                    data_frame[2] = struct.unpack_from('<B', buffer, 0)
                    # to remove tuples
                    data_frame[2] = data_frame[2][0]
                    # get acc floats
                    data_frame[3] = [0, 0, 0]
                    data_frame[3][0:] = struct.unpack_from('<fff', buffer, 1)
                    # get gyro floats
                    data_frame[4] = [0, 0, 0]
                    data_frame[4][0:] = struct.unpack_from('<fff', buffer, 13)
                    # get mag floats
                    data_frame[5] = [0, 0, 0]
                    data_frame[5][0:] = struct.unpack_from('<fff', buffer, 25)
                    # get euler floats
                    data_frame[6] = [0, 0, 0]
                    data_frame[6][0:] = struct.unpack_from('<fff', buffer, 37)
                    # get crc8 maxim
                    data_frame[7] = struct.unpack_from('<B', buffer, 49)
                    data_frame[7] = data_frame[7][0]
                    data_frame = dict(zip(self.keys, data_frame))
                    # add timestamp
                    current_time = datetime.now()
                    my_time = '{0:%Y-%m-%d %H:%M:%S}'.format(current_time) + '.{0:03.0f}'.format(round(current_time.microsecond / 1000.0))
                    outMessage = dict(Time=my_time, Index=index)
                    outMessage.update(data_frame)
                    self.dataQueue.put_nowait(outMessage)
                    index = index+ 1
                else:
                    pass
            else:
                # self.log.info("New Byte received: 0x{0:X}".format(syncAA))
                pass
        self.log.info("Exiting Thread logger")
        return

    def shutdown(self):
        """Prepare for exiting program

        Returns:
            always returns true after all tasks are done.

        * exit logger thread
        * close port

        """
        self.exitFlag.set()
        self.threadID.join()
        self.device.close()
        self.isOpen = False
        self.log.info("Shutting down")
        return True


def main():
    """Set of test to run to see if class behaves as expected.

    Creates a razor class object and execute the following commands:

    - begin: on default port or given port by argv[1].
    - print messages received from razor
    - wait for 10 seconds
    - shutdown: safely disconnects.

    """
    import argparse

    def printData(dataQueue, exitFlag):
        """ prints data to console

        Thread used to print data from razor to the console.

        Args:
            dataQueue: queue class object where data is stored
            exitFlag: a flag to control the exit of program gracefully

        """
        print("Index,Time,ID,accx,accy,accz,gyrox,gyroy,gyroz,magx,magy,magz,psi,theta,phi,crc8maxim\n")
        while(exitFlag.isSet() == False):
            if(dataQueue.empty() == False):
                newData = dataQueue.get()
                print('{0:5d},{1},{2},{3:.2f},{4:.2f},{5:.2f},{6:.2f},'
                      '{7:.2f},{8:.2f},{9:.2f},{10:.2f},{11:.2f},'
                      '{12:.2f},{13:.2f},{14:.2f},{15}'
                      '\n'.format(newData['Index'],
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
    parser = argparse.ArgumentParser(add_help=True, description="interface for razor sensors")
    parser.add_argument("-p", "--port", action="store", type=str,
                        dest="port", default="/dev/ttyUSB0", help='serial port used')
    parser.add_argument("-n", "--name", action="store", type=str,
                        dest="name", default="RAZOR", help='ID of sensor, in case of multiple units')
    parser.add_argument('--log', action='store', type=str, dest='log', default='razor.log',
                        help='log file to be used')
    parser.add_argument("--log-level", action="store", type=str,
                        dest="logLevel", default='info',
                        help='Log level to be used. See logging module for more info',
                        choices=['critical', 'error', 'warning', 'info', 'debug'])
    parser.add_argument("-b", "--baud", action="store", type=str, dest="baud", default=500000,
                        help="device baud rate")

    args = parser.parse_args()

    log_level = {'error': logging.ERROR,
                 'debug': logging.DEBUG,
                 'info': logging.INFO,
                 'warning': logging.WARNING,
                 'critical': logging.CRITICAL
                 }

    logging.basicConfig(filename=args.log,
                        level=log_level[args.logLevel],
                        format='[%(asctime)s] [%(name)-20s] [%(threadName)-10s] %(levelname)-8s %(message)s',
                        filemode="w")

    # ---------------------------------------------------------------------------
    # define a Handler which writes INFO messages or higher in console
    # ---------------------------------------------------------------------------
    console = logging.StreamHandler()
    console.setLevel(log_level[args.logLevel])
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-20s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)
    # ---------------------------------------------------------------------------
    # event flag to exit
    exitFlag = threading.Event()
    # create a queue to receive commands
    dataFIFO = queue.Queue()

    # create a thread to parse responses
    thread1 = threading.Thread(name="printData", target=printData,
                               args=(dataFIFO, exitFlag))

    # instantiate a class object
    razor = RazorIMU(args.name)
    # begin
    if razor.begin(dataFIFO, com_port=args.port, baud_rate=args.baud) != 1:
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
