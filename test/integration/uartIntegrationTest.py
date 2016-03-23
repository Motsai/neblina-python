#!/usr/bin/env python
###################################################################################
#
# Copyright (c)     2010-2016   Motsai
#
# The MIT License (MIT)
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
#
###################################################################################

import unittest
import serial
import serial.tools.list_ports
import time
import csv
import array
import logging

from neblina import *
from neblinaAPI import NeblinaAPI
from test import neblinaTestUtilities

###################################################################################


def getSuite(comPort):
    UARTIntegrationTest.comPort = comPort
    return unittest.TestLoader().loadTestsFromTestCase(UARTIntegrationTest)

###################################################################################


class UARTIntegrationTest(unittest.TestCase):
    setupHasAlreadyRun = False
    comPort = None

    def setUp(self):
        if not self.comPort:
            raise unittest.SkipTest("No COM port specified.")

        # Give it a break between each test
        time.sleep(1)

        self.api = NeblinaAPI(Interface.UART)
        self.api.open(self.comPort)
        if not self.api.isOpened(self.comPort):
            self.fail("Unable to connect to COM port.")

    def tearDown(self):
        self.api.close()

    def testStreamEuler(self):
        self.api.motionStream(Commands.Motion.EulerAngle, 100)

    def testStreamIMU(self):
        self.api.motionStream(Commands.Motion.IMU, 100)

    def testVersion(self):
        versions = self.api.debugFWVersions()
        logging.info(versions)
        self.assertNotEqual(versions[2][0], 255)

    def testMEMSComm(self):
        logging.debug('Checking communication with the LSM9DS1 chip by getting the temperature...')
        temp = self.api.getTemperature()
        logging.info("Board Temperature: {0} degrees (Celsius)".format(temp))

    def testPMICComm(self):
        batteryLevel = self.api.getBatteryLevel()
        logging.info("Board Battery: {0}\%".format(batteryLevel))

    def testUARTPCLoopbackComm(self):
        #dataString = "Test#1: Loopback test with KL26 by sending 1000 empty packets..."
        for x in range(1, 1001):
            #logging.debug('Loopback test packet %d\r' % (x), end="", flush=True)
            self.api.sendCommand(SubSystem.Debug, Commands.Debug.SetInterface, True)
            self.api.waitForAck(SubSystem.Debug, Commands.Debug.SetInterface)

    def testMotionEngine(self):
        testInputVectorPacketList = neblinaTestUtilities.csvVectorsToList('motEngineInputs.csv')
        testOutputVectorPacketList = neblinaTestUtilities.csvVectorsToList('motEngineOutputs.csv')
        self.api.debugUnitTestEnable(True)
        for idx,packetBytes in enumerate(testInputVectorPacketList):
            # logging.debug('Sending {0} to stream'.format(binascii.hexlify(packetBytes)))
            packet = self.api.debugUnitTestSendBytes(packetBytes)
            # self.api.comslip.sendPacketToStream(self.api.sc, packetBytes)
            # packet = self.api.waitForPacket(PacketType.RegularResponse, \
            #                                 SubSystem.Debug, Commands.Debug.UnitTestMotionData)
            self.assertEqual(testOutputVectorPacketList[idx], packet.stringEncode())
            print("Sent %d testVectors out of %d\r" % (idx, len(testInputVectorPacketList)), end="", flush=True)
        print("\r")
        self.api.debugUnitTestEnable(False)

    def testLEDs(self):
        for i in range(0, 10):
            self.api.setLED(i, 1)
            #self.assertEqual(1, self.api.getLED(i))
        for i in range(0, 10):
            self.api.setLED(i, 0)
            #self.assertEqual(0, self.api.getLED(i))