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

import logging

from neblina import *
from neblinaCore import NeblinaCore
from neblinaData import *
from neblinaError import *
from neblinaUtilities import NebUtilities

###################################################################################


class NeblinaAPI(object):

    def __init__(self):
        self.core = NeblinaCore()

    def close(self):
        self.core.close()

    def open(self, address):
        self.core.open(address)

    def isOpened(self, port=None):
        return self.core.isOpened()

    def receivePacket(self):
        raise NotImplementedError("receivePacket not override in child.")

    def getBatteryLevel(self):
        return self.core.getBatteryLevel()

    def getTemperature(self):
        self.core.sendCommand(SubSystem.Power, Commands.Power.GetTemperature, True)
        packet = self.core.waitForAck(SubSystem.Power, Commands.Power.GetTemperature)
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Power, Commands.Power.GetTemperature)
        return packet.data.temperature

    def setDataPortState(self, interface, state):
        assert(type(state) is bool)
        self.core.sendCommand(SubSystem.Debug, Commands.Debug.InterfaceState, state, interface=interface)
        logging.debug('Waiting for the module to set data port state...')
        packet = self.core.waitForAck(SubSystem.Debug, Commands.Debug.InterfaceState)
        logging.debug('Module has change its data port state')

    def setInterface(self, interface=Interface.BLE):
        self.core.sendCommand(SubSystem.Debug, Commands.Debug.SetInterface, interface)
        logging.debug('Waiting for the module to switch its interface...')
        packet = self.waitForAck(SubSystem.Debug, Commands.Debug.SetInterface)
        logging.debug("Module has switched its interface.")
        numTries = 0
        while (packet == None):
            numTries += 1
            if numTries > 5:
                logging.error('The unit is not responding. Exiting...')
                exit()
            logging.warning('Trying again...')
            self.core.sendCommand(SubSystem.Debug, Commands.Debug.SetInterface, interface)
            packet = self.waitForAck(SubSystem.Debug, Commands.Debug.SetInterface)

    def getMotionStatus(self):
        self.core.sendCommand(SubSystem.Debug, Commands.Debug.MotAndFlashRecState)
        self.core.waitForAck(SubSystem.Debug, Commands.Debug.MotAndFlashRecState)
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Debug, Commands.Debug.MotAndFlashRecState)
        return packet.data.motionStatus

    def getRecorderStatus(self):
        self.core.sendCommand(SubSystem.Debug, Commands.Debug.MotAndFlashRecState)
        self.core.waitForAck(SubSystem.Debug, Commands.Debug.MotAndFlashRecState)
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Debug, Commands.Debug.MotAndFlashRecState)
        return packet.data.recorderStatus

    def setDownsample(self, factor):
        # Limit to factor of 20 and between 20 and 1000.
        assert factor % 20 == 0 and 20 <= factor <= 1000
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.Downsample, factor)
        logging.debug('Sending downsample command. Waiting for acknowledge.')
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.Downsample)
        logging.debug('Acknowledgment received.')

    def setAccelerometerRange(self, factor):
        # Limit factor to 2, 4, 8 and 16
        assert factor == 2 or factor == 4 or factor == 8 or factor == 16
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.AccRange, factor)
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.AccRange)

    def resetTimestamp(self):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.ResetTimeStamp, True)
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.ResetTimeStamp)

    def disableStreaming(self):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.DisableStreaming, True)
        logging.debug("Sending disable streaming command. Waiting for acknowledge.")
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.DisableStreaming)
        logging.debug("Acknowledgment received.")

    def getEulerAngle(self):
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Motion, Commands.Motion.EulerAngle)
        logging.debug("Received EulerAngle.")
        return packet

    def getExternalForce(self):
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Motion, Commands.Motion.ExtForce)
        logging.debug("Received ExternalForce.")
        return packet

    def getFingerGesture(self):
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Motion, Commands.Motion.FingerGesture)
        logging.debug("Received FingerGesture.")
        return packet

    def getIMU(self):
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Motion, Commands.Motion.IMU)
        logging.debug("Received IMU.")
        return packet

    def getMAG(self):
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Motion, Commands.Motion.MAG)
        logging.debug("Received MAG.")
        return packet

    def getMotionState(self):
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Motion, Commands.Motion.MotionState)
        logging.debug("Received MotionState.")
        return packet

    def getPedometer(self):
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Motion, Commands.Motion.Pedometer)
        logging.debug("Received Pedometer.")
        return packet

    def getQuaternion(self):
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Motion, Commands.Motion.Quaternion)
        logging.debug("Received Quaternion.")
        return packet

    def getRotationInfo(self):
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Motion, Commands.Motion.RotationInfo)
        logging.debug("Received RotationInfo.")
        return packet

    def getSittingStanding(self):
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Motion, Commands.Motion.SittingStanding)
        logging.debug("Received SittingStanding.")
        return packet

    def getTrajectoryInfo(self):
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Motion, Commands.Motion.TrajectoryInfo)
        logging.debug("Received TrajectoryInfo.")
        return packet

    def streamEulerAngle(self, state):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.EulerAngle, state)
        logging.debug("Sending streamEulerAngle. Waiting for acknowledge.")
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.EulerAngle)
        logging.debug("Acknowledgment received.")

    def streamExternalForce(self, state):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.ExtForce, state)
        logging.debug("Sending streamExternalForce. Waiting for acknowledge.")
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.ExtForce)
        logging.debug("Acknowledgment received.")

    def streamFingerGesture(self, state):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.FingerGesture, state)
        logging.debug("Sending streamFingerGesture. Waiting for acknowledge.")
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.FingerGesture)
        logging.debug("Acknowledgment received.")

    def streamIMU(self, state):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.IMU, state)
        logging.debug("Sending streamIMU. Waiting for acknowledge.")
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.IMU)
        logging.debug("Acknowledgment received.")

    def streamMAG(self, state):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.MAG, state)
        logging.debug("Sending streamMAG. Waiting for acknowledge.")
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.MAG)
        logging.debug("Acknowledgment received.")

    def streamMotionState(self, state):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.MotionState, state)
        logging.debug("Sending streamMotionState. Waiting for acknowledge.")
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.MotionState)
        logging.debug("Acknowledgment received.")

    def streamPedometer(self, state):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.Pedometer, state)
        logging.debug("Sending streamPedometer. Waiting for acknowledge.")
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.Pedometer)
        logging.debug("Acknowledgment received.")

    def streamQuaternion(self, state):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.Quaternion, state)
        logging.debug("Sending streamQuaternion. Waiting for acknowledge.")
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.Quaternion)
        logging.debug("Acknowledgment received.")

    def streamRotationInfo(self, state):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.RotationInfo, state)
        logging.debug("Sending streamRotationInfo. Waiting for acknowledge.")
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.RotationInfo)
        logging.debug("Acknowledgment received.")

    def streamSittingStanding(self, state):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.SittingStanding, state)
        logging.debug("Sending streamSittingStanding. Waiting for acknowledge.")
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.SittingStanding)
        logging.debug("Acknowledgment received.")

    def streamTrajectoryInfo(self, state):
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.TrajectoryInfo, state)
        logging.debug("Sending streamTrajectoryInfo. Waiting for acknowledge.")
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.TrajectoryInfo)
        logging.debug("Acknowledgment received.")

    # def motionStopStream(self, streamingType):
    #     self.core.sendCommand(SubSystem.Motion, streamingType, False)
    #     logging.debug("Sending stop motion command. Waiting for acknowledge.")
    #     self.core.waitForAck(SubSystem.Motion, streamingType)
    #     logging.debug("Acknowledgment received.")
    #
    # def motionStartStream(self, streamingType):
    #     # Send command to start streaming
    #     self.core.sendCommand(SubSystem.Motion, streamingType, True)
    #     logging.debug("Sending start motion command. Waiting for acknowledge.")
    #     packet = self.core.waitForAck(SubSystem.Motion, streamingType)
    #     logging.debug("Acknowledgment received.")
    #     return packet

    # # Motine Engine commands
    # def motionStream(self, streamingType, numPackets=None):
    #     errorList = []
    #     packet = self.motionStartStream(streamingType)
    #
    #     # Timeout mechanism.
    #     numTries = 0
    #     while (packet == None):
    #         logging.warning('Timed out. Trying again.')
    #         self.core.sendCommand(SubSystem.Motion, streamingType, True)
    #         packet = self.waitForAck(SubSystem.Motion, streamingType)
    #         numTries += 1
    #         if numTries > 5:
    #             logging.error('Tried {0} times and it doesn\'t respond. Exiting.'.format(numTries))
    #             exit()
    #     numTries = 0
    #
    #     # Stream forever if the number of packets is unspecified (None)
    #     keepStreaming = (numPackets == None or numPackets > 0)
    #     while(keepStreaming):
    #         try:
    #             packet = self.receivePacket()
    #             if (packet.header.subSystem == SubSystem.Motion and packet.header.command == streamingType):
    #                 logging.info(packet.data)
    #             elif (packet.header.subSystem != SubSystem.Debug):
    #                 logging.warning('Unexpected packet: {0}'.format(packet.stringEncode()))
    #             if (numPackets != None):
    #                 numPackets -= 1
    #             keepStreaming = (numPackets == None or numPackets > 0)
    #         except NotImplementedError as nie:
    #             logging.error("NotImplementedError : " + str(nie))
    #         # In the event of Ctrl-C
    #         except KeyboardInterrupt as ki:
    #             break
    #         except CRCError as e:
    #             logging.error("CRCError : " + str(e))
    #         except TimeoutError as te:
    #             if ( streamingType != Commands.Motion.RotationInfo and \
    #                  streamingType != Commands.Motion.Pedometer and \
    #                  streamingType != Commands.Motion.FingerGesture and \
    #                  streamingType != Commands.Motion.TrajectoryInfo):
    #                 logging.warning('Timed out, sending command again.')
    #                 numTries += 1
    #                 self.core.sendCommand(SubSystem.Motion, streamingType, True)
    #                 if numTries > 3:
    #                     logging.error('Tried {0} times and it doesn\'t respond. Exiting.'.format(numTries))
    #                     exit()
    #         except Exception as e:
    #             logging.error("Exception : " + str(e))
    #
    #     # Stop whatever it was streaming
    #     self.motionStopStream(streamingType)

    def eepromRead(self, readPageNumber):
        assert 0 <= readPageNumber <= 255
        self.core.sendCommand(SubSystem.EEPROM, Commands.EEPROM.Read, pageNumber=readPageNumber)
        logging.debug("Sending EEPROM Read command. Waiting for acknowledgment.")
        packet = self.core.waitForAck(SubSystem.EEPROM, Commands.EEPROM.Read)
        logging.debug("Acknowledge received.")
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.EEPROM, Commands.EEPROM.Read)
        logging.debug("EEPROM Read packet received.")
        return packet.data.dataBytes

    def eepromWrite(self, writePageNumber, dataString):
        assert 0 <= writePageNumber <= 255
        self.core.sendCommand(SubSystem.EEPROM, Commands.EEPROM.Write, \
                         pageNumber=writePageNumber, dataBytes=dataString)
        packet = self.core.waitForAck(SubSystem.EEPROM, Commands.EEPROM.Write)

    def getLEDs(self, ledIndices):
        if type(ledIndices) != list:
            logging.warning("Use this function with a list of leds you want to know the value as an argument.")
            return
        self.core.sendCommand(SubSystem.LED, Commands.LED.GetVal, ledIndices=ledIndices)
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.LED, Commands.LED.GetVal)
        return packet.data.ledTupleList

    def getLED(self, index):
        self.core.sendCommand(SubSystem.LED, Commands.LED.GetVal, ledIndices=[index])
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.LED, Commands.LED.GetVal)
        if not packet:
            return 0xF  # Return anything but 0x0 or 0x1
        return packet.data.ledTupleList[0]

    def setLEDs(self, ledValues):
        if type(ledValues) != list and type(ledValues[0]) == tuple:
            logging.warning("Use this function with a list of tuples as an argument.")
            return
        self.core.sendCommand(SubSystem.LED, Commands.LED.SetVal, ledValueTupleList=ledValues)
        self.core.waitForAck(SubSystem.LED, Commands.LED.SetVal)
        self.core.waitForPacket(PacketType.RegularResponse, SubSystem.LED, Commands.LED.GetVal)

    def setLED(self, ledIndex, ledValue):
        ledValues = [(ledIndex, ledValue)]
        self.core.sendCommand(SubSystem.LED, Commands.LED.SetVal, ledValueTupleList=ledValues)
        self.core.waitForAck(SubSystem.LED, Commands.LED.SetVal)
        self.core.waitForPacket(PacketType.RegularResponse, SubSystem.LED, Commands.LED.GetVal)

    def flashGetState(self):
        self.core.sendCommand(SubSystem.Debug, Commands.Debug.MotAndFlashRecState)
        self.core.waitForAck(SubSystem.Debug, Commands.Debug.MotAndFlashRecState)
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Debug, Commands.Debug.MotAndFlashRecState)
        return MotAndFlashRecStateData.recorderStatusStrings[packet.data.recorderStatus]

    def flashErase(self, eraseType=Erase.Quick):
        # Step 1 - Initialization
        self.core.sendCommand(SubSystem.Motion, Commands.Motion.DisableStreaming, True)
        logging.debug('Sending the DisableAllStreaming command, and waiting for a response...')

        # Step 2 - wait for ack
        self.core.waitForAck(SubSystem.Motion, Commands.Motion.DisableStreaming)
        logging.debug('Acknowledge packet was received!')

        # Step 3 - erase the flash command
        self.core.sendCommand(SubSystem.Storage, Commands.Storage.EraseAll, eraseType)
        logging.debug('Sent the EraseAll command, and waiting for a response...')

        # Step 4 - wait for ack
        self.core.waitForAck(SubSystem.Storage, Commands.Storage.EraseAll)
        logging.debug("Acknowledge packet was received!")
        logging.info("Started erasing... This takes up to around 3 minutes...")

        # Step 5 - wait for the completion notice
        self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Storage, Commands.Storage.EraseAll)
        logging.info('Flash erase has completed successfully!')

    def flashRecordStart(self, streamingType=None):
        if streamingType:
            self.core.sendCommand(SubSystem.Motion, Commands.Motion.DisableStreaming, False)
            logging.debug('Sending the stop streaming command, and waiting for a response...')

            self.waitForAck(SubSystem.Motion, Commands.Motion.DisableStreaming)
            logging.debug('Acknowledge packet was received!')

        # Step 1 - Initialization
        # self.sendCommand(SubSystem.Motion, Commands.Motion.DisableStreaming, True)
        # logging.debug('Sending the DisableAllStreaming command, and waiting for a response...')

        # Step 2 - wait for ack
        # self.waitForAck(SubSystem.Motion, Commands.Motion.DisableStreaming)
        # logging.debug('Acknowledge packet was received!')

        # Step 3 - Start recording
        self.core.sendCommand(SubSystem.Storage, Commands.Storage.Record, True)
        logging.debug('Sending the command to start the flash recorder, and waiting for a response...')

        # Step 4 - wait for ack and the session number
        self.core.waitForAck(SubSystem.Storage, Commands.Storage.Record)
        logging.debug("Acknowledge received.")
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Storage, Commands.Storage.Record)
        if packet.header.packetType == PacketType.ErrorLogResp:
            logging.warn("Flash is full, not recording.")
        else:
            logging.debug('Acknowledge packet was received with the session number {0}!'.format(packet.data.sessionID))
        sessionID = packet.data.sessionID

        if streamingType:
            # Step 5 - enable streaming
            self.core.sendCommand(SubSystem.Motion, streamingType, True)
            logging.debug('Sending the enable streaming command, and waiting for a response...')

            # Step 6 - wait for ack
            self.core.waitForAck(SubSystem.Motion, streamingType)
            logging.debug('Acknowledge packet was received!')

        return sessionID

    def flashRecordStop(self, streamingType=None):
        if streamingType:
            # Step 8 - Stop the streaming
            self.core.sendCommand(SubSystem.Motion, Commands.Motion.DisableStreaming, True)
            logging.debug('Sending the stop streaming command, and waiting for a response...')

            # Step 9 - wait for ack
            self.core.waitForAck(SubSystem.Motion, Commands.Motion.DisableStreaming)
            logging.debug('Acknowledge packet was received!')

        # Step 10 - Stop the recording
        self.core.sendCommand(SubSystem.Storage, Commands.Storage.Record, False)
        logging.debug('Sending the command to stop the flash recorder, and waiting for a response...')

        # Step 11 - wait for ack and the closed session confirmation
        self.core.waitForAck(SubSystem.Storage, Commands.Storage.Record)
        logging.debug("The acknowledge packet is received")
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Storage, Commands.Storage.Record)

        return packet.data.sessionID

    def flashRecord(self, numSamples, streamingType):
        sessionID = self.flashRecordStart(streamingType)

        # Step 7 Receive Packets
        for x in range(1, numSamples + 1):
            packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Motion, streamingType)
            print('Recording {0} packets, current packet: {1}'.format(numSamples, x), end="\r", flush=True)

        self.flashRecordStop(streamingType)

        logging.info("Session {0} is closed successfully".format(sessionID))

    def flashPlayback(self, pbSessionID, dump=False):
        self.core.sendCommand(SubSystem.Storage, Commands.Storage.Playback, True, sessionID=pbSessionID)
        logging.debug('Sent the start playback command, waiting for response...')
        # wait for confirmation
        self.core.waitForAck(SubSystem.Storage, Commands.Storage.Playback)
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Storage, Commands.Storage.Playback)
        if packet.header.packetType == PacketType.ErrorLogResp:
            logging.error('Playback failed due to an invalid session number request!')
            return 0
        else:
            pbSessionID = packet.data.sessionID
            logging.info('Playback routine started from session number {0}'.format(pbSessionID))
            packetList = self.storePacketsUntil(PacketType.RegularResponse, SubSystem.Storage,
                                                Commands.Storage.Playback)
            logging.info('Finished playback from session number {0}!'.format(pbSessionID))
            if dump:
                logging.info('Saving dump file. Waiting for completion...')
                NebUtilities.saveFlashPlayback(pbSessionID, packetList)
                logging.info('Dump file saving completed.')
            return len(packetList)

    def flashGetSessions(self):
        self.core.sendCommand(SubSystem.Storage, Commands.Storage.NumSessions)
        packet = self.core.waitForAck(SubSystem.Storage, Commands.Storage.NumSessions)
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Storage, Commands.Storage.NumSessions)
        return packet.data.numSessions

    def flashGetSessionInfo(self, sessionID):
        self.core.sendCommand(SubSystem.Storage, Commands.Storage.SessionInfo, sessionID=sessionID)
        self.core.waitForAck(SubSystem.Storage, Commands.Storage.SessionInfo)
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Storage, Commands.Storage.SessionInfo)
        if packet.data.sessionLength == 0xFFFFFFFF:
            return None
        else:
            return packet.data

    def storePacketsUntil(self, packetType, subSystem, command):
        packetList = []
        packet = None
        while not packet or \
            not packet.isPacketValid(packetType, subSystem, command):
            try:
                if (packet != None and packet.header.subSystem != SubSystem.Debug):
                    packetList.append(packet)
                    print('Received {0} packets'.format(len(packetList)), end="\r", flush=True)
                packet = self.receivePacket()
            except NotImplementedError as e:
                packet = None
                logging.error("Packet {0} - Dropped bad packet : {1}".format(len(packetList), str(e)))
                continue
            except KeyError as e:
                packet = None
                logging.error("Packet {0} - Tried creating a packet with an invalid subsystem or command : {1}".format(\
                    len(packetList), str(e)))
                continue
            except CRCError as e:
                packet = None
                logging.error("Packet {0} - CRCError : {1} ".format(len(packetList), str(e)))
                continue
            except Exception as e:
                packet = None
                logging.error("Packet {0} - Exception : {1}".format(len(packetList), str(e)))
                continue
            except:
                packet = None
                logging.error("Packet {0} - Unexpected error".format(len(packetList)), exc_info=True)
                continue

        logging.info('Total IMU Packets Read: {0}'.format(len(packetList)))
        return packetList

    def getFirmwareVersion(self):
        self.core.sendCommand(SubSystem.Debug, Commands.Debug.FWVersions)
        versionPacket = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Debug, Commands.Debug.FWVersions)
        return versionPacket.data

    def debugUnitTestEnable(self, enable=True):
        self.core.sendCommand(SubSystem.Debug, Commands.Debug.StartUnitTestMotion, enable)
        logging.debug("Sending Start UnitTest Motion. Waiting for acknowledgment.")
        self.core.waitForAck(SubSystem.Debug, Commands.Debug.StartUnitTestMotion)
        logging.debug("Acknowledgment received")

    def debugUnitTestSendBytes(self, bytes):
        self.core.sendCommandBytes(bytes)
        logging.debug("Sending UnitTest Motion Bytes. Waiting for packet.")
        packet = self.core.waitForPacket(PacketType.RegularResponse, SubSystem.Debug, Commands.Debug.UnitTestMotionData)
        logging.debug("Packet received.")
        return packet