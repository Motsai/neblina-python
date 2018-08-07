/***********************************************************************************
* Copyright (c) 2010 - 2017, Motsai
* All rights reserved.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************************/

#pragma once

/**********************************************************************************/

#include "neblina.h"

/**********************************************************************************/

#ifdef __cplusplus

namespace neblina
{

/**********************************************************************************/

class NEBLINA_EXTERN NeblinaCommunicationBase
{
public:
    NeblinaCommunicationBase();
    virtual ~NeblinaCommunicationBase();

    /// Receive all available data from a communication interface.
    ///
    /// \param  ppBytes     Received bytes.
    /// \param  pSize       Number of bytes received.
    ///
    /// \note   Calling receive() calls receiveBytes() to fill the receive buffer.
    /// \note   Calling receive(...) will directly fill the receive buffer.
    ///
    /// \note   In the event of a failed receive, multiple packet could have been receive succesfully.
    ///
    /// \return 0, if receive succesful.
    ///         1, if receive failed due to an invalid packet (CRC mismatch). The packet is drop.
    ///         2, if receive failed due to an unhandle packet.
    ///         3, if receive failed due to a bad packet. Receive buffer flush, Multiple packet drop.
    uint8_t receive();
    uint8_t receive( const uint8_t* ppBytes, uint32_t pSize );

    /// Functions to send command to Neblina
    bool sendCommandEEPROMRead( const NeblinaEEPROMRead_t& prRead );
    bool sendCommandEEPROMWrite( const NeblinaEEPROMData_t& prData );
    bool sendCommandFusionDownsample( const NeblinaFusionDownsample_t& prDownsample );
    bool sendCommandFusionEulerAngleStream( bool pState );
    bool sendCommandFusionExternalForceStream( bool pState );
    bool sendCommandFusionExternalHeadingCorrection( const NeblinaFusionHeadingCorrection_t& prCorrection );
    bool sendCommandFusionFingerGestureStream( bool pState );
    bool sendCommandFusionLockHeadingReference();
    bool sendCommandFusionMotionStateStream( bool pState );
    bool sendCommandFusionPedometerStream( bool pState );
    bool sendCommandFusionQuaternionStream( bool pState );
    bool sendCommandFusionRate( NeblinaRate_t pRate );
    bool sendCommandFusionRotationInfoStream( bool pState );
    bool sendCommandFusionSetFusionType( NeblinaFusionType_t pType );
    bool sendCommandFusionShockSegmentStream( const NeblinaFusionShockInfo_t& prShockInfo );
    bool sendCommandFusionSittingStandingStream( bool pState );
    bool sendCommandFusionTrajectoryRecord( bool pState );
    bool sendCommandFusionTrajectoryInfoStream( bool pState );
    bool sendCommandFusionCalibrateForwardPosition();
    bool sendCommandFusionCalibrateDownPosition();
    bool sendCommandFusionMotionDirectionStream( bool pState );
    bool sendCommandFusionAccelerometerCalibrationReset();
    bool sendCommandFusionAccelerometerCalibrationSetNewPosition();
    bool sendCommandFusionCalibratedAccelerometerStream( bool pState );
    bool sendCommandFusionInclinometerCalibrate();
    bool sendCommandFusionInclinometerStream( bool pState );
    bool sendCommandFusionMagnetometerACStream( bool pState );
    bool sendCommandFusionMotionIntensityTrendStream( const NeblinaFusionMotionIntensityTrendStreamInfo_t& prMotionTrendInfo );
    bool sendCommandGeneralAuthentication();
    bool sendCommandGeneralDeviceNameGet();
    bool sendCommandGeneralDeviceNameSet( const char* ppDeviceName );
    bool sendCommandGeneralDeviceReset();
    bool sendCommandGeneralDeviceShutdown();
    bool sendCommandGeneralDisableStreaming();
    bool sendCommandGeneralFirmwareVersion();    
    bool sendCommandGeneralFirmwareUpdate();
    bool sendCommandGeneralFusionStatus();
    bool sendCommandGeneralInterfaceState( const NeblinaInterfaceState_t& prData );
    bool sendCommandGeneralInterfaceStatus();
    bool sendCommandGeneralPowerStatus();
    bool sendCommandGeneralRecorderStatus();
    bool sendCommandGeneralResetTimestamp( NeblinaResetTimestamp_t pReset );
    bool sendCommandGeneralSetUnixTimestamp( const uint32_t& pTimestamp );
    bool sendCommandGeneralGetUnixTimestamp();
    bool sendCommandGeneralSensorStatus();
    bool sendCommandGeneralSystemStatus();
    bool sendCommandLEDState( const NeblinaLEDState_t& prData );
    bool sendCommandLEDStatus();
    bool sendCommandPowerBattery();
    bool sendCommandPowerChargeCurrent( const NeblinaPowerChargeCurrent_t& prCurrent );
    bool sendCommandPowerTemperature();
    bool sendCommandRecorderEraseAll( NeblinaRecorderErase_t pType );
    bool sendCommandRecorderPlayback( const NeblinaSessionStatus_t& prData );
    bool sendCommandRecorderRecord( bool pState, const char* ppName );
    bool sendCommandRecorderSessionCount();
    bool sendCommandRecorderSessionFusionInfo( uint16_t pSessionId );
    bool sendCommandRecorderSessionGeneralInfo( uint16_t pSessionId );
    bool sendCommandRecorderSessionSensorInfo( uint16_t pSessionId );
    bool sendCommandRecorderSessionName( uint16_t pSessionId );
    bool sendCommandRecorderSessionRead( const NeblinaSessionReadCommand_t& prCommand );
    bool sendCommandRecorderSessionDownload( const NeblinaSessionDownload_t& prCommand );
    bool sendCommandSensorGetBandwidth( const NeblinaSensorType_t& prSensor );
    bool sendCommandSensorGetDownsample( const NeblinaSensorStream_t& prStream );
    bool sendCommandSensorGetRange( const NeblinaSensorType_t& prSensor );
    bool sendCommandSensorGetRate( const NeblinaSensorType_t& prSensor );    
    bool sendCommandSensorSetBandwidth( const NeblinaSensorBandwidth_t& prData );
    bool sendCommandSensorSetDownsample( const NeblinaSensorDownsample_t& prData );
    virtual bool sendCommandSensorSetRange( const NeblinaSensorRange_t& prData );
    bool sendCommandSensorSetRate( const NeblinaSensorRate_t& prRate );
    bool sendCommandSensorAccelerometerStream( bool pState );
    bool sendCommandSensorAccelerometerGyroscopeStream( bool pState );
    bool sendCommandSensorAccelerometerMagnetometerStream( bool pState );
    bool sendCommandSensorGyroscopeStream( bool pState );
    bool sendCommandSensorHumidityStream( bool pState );
    bool sendCommandSensorMagnetometerStream( bool pState );
    bool sendCommandSensorPressureStream( bool pState );
    bool sendCommandSensorTemperatureStream( bool pState );
    bool sendPacket( NeblinaPacket_t* ppPacket );

    /// EXPERIMENTAL
    bool sendCommandFusionAnalysisCalibrate();
    bool sendCommandFusionAnalysisCreatePose( const NeblinaFusionMotionAnalysisPose_t& prPose );
    bool sendCommandFusionAnalysisGetActivePose();
    bool sendCommandFusionAnalysisGetPoseInfo( uint8_t pPoseId );
    bool sendCommandFusionAnalysisReset();
    bool sendCommandFusionAnalysisSetActivePose( uint8_t pPoseId );
    bool sendCommandFusionAnalysisStream( bool pState );

protected:
    /// Retrieve received packet header
    const NeblinaPacketHeader_t& getPacketHeader();

    /// Receive byte-array from communication interface.
    ///
    ///
    virtual uint32_t receiveBytes( uint8_t* ppBytes, uint32_t pSize );
    /// Send byte-array to communication interface.
    ///
    /// @note   Packet has already been prepare and is fully ready to transmit.
    virtual bool sendBytes( const uint8_t* ppBytes, uint32_t pSize );

/// Logging functions (for high-level application) called before sending and after receiving a packet
private:
    virtual void logPacketReceive( uint8_t pPacketType, uint8_t pSubSystem, uint8_t pCommand );
    virtual void logPacketSend( uint8_t pPacketType, uint8_t pSubSystem, uint8_t pCommand );

/// List of functions to override in order to process a received packet.
private:
    virtual bool processPacket( const NeblinaPacket_t& ppPacket );
    virtual bool processAck( const NeblinaPacketHeader_t& prHeader );
    virtual bool processCommandEEPROMRead( const NeblinaEEPROMRead_t& pPageId );
    virtual bool processCommandEEPROMWrite( const NeblinaEEPROMData_t& prData );
    virtual bool processCommandFusionAnalysisCalibrate();
    virtual bool processCommandFusionAnalysisCreatePose( const NeblinaFusionMotionAnalysisPose_t& prPose );
    virtual bool processCommandFusionAnalysisGetActivePose();
    virtual bool processCommandFusionAnalysisGetPoseInfo( uint8_t );
    virtual bool processCommandFusionAnalysisReset();
    virtual bool processCommandFusionAnalysisSetActivePose( uint8_t pPoseId );
    virtual bool processCommandFusionAnalysisStream( bool pState );
    virtual bool processCommandFusionDownsample( const NeblinaFusionDownsample_t& prDownsample );
    virtual bool processCommandFusionEulerAngleStream( bool pState );
    virtual bool processCommandFusionExternalForceStream( bool pState );
    virtual bool processCommandFusionExternalHeadingCorrection( const NeblinaFusionHeadingCorrection_t& prCorrection );
    virtual bool processCommandFusionFingerGestureStream( bool pState );
    virtual bool processCommandFusionLockHeadingReference();
    virtual bool processCommandFusionMotionStateStream( bool pState );
    virtual bool processCommandFusionPedometerStream( bool pState );
    virtual bool processCommandFusionQuaternionStream( bool pState );
    virtual bool processCommandFusionRate( NeblinaRate_t pRate );
    virtual bool processCommandFusionRotationInfoStream( const NeblinaFusionRotationInfo_t& prRotationInfo );
    virtual bool processCommandFusionSetFusionType( NeblinaFusionType_t pType );
    virtual bool processCommandFusionSittingStandingStream( bool pState );
    virtual bool processCommandFusionTrajectoryRecord( bool pState );
    virtual bool processCommandFusionTrajectoryInfoStream( bool pState );
    virtual bool processCommandFusionCalibrateForwardPosition();
    virtual bool processCommandFusionCalibrateDownPosition();
    virtual bool processCommandFusionMotionDirectionStream( bool pState );
    virtual bool processCommandFusionShockSegmentStream( const NeblinaFusionShockInfo_t& prShockInfo );
    virtual bool processCommandFusionAccelerometerCalibrationReset();
    virtual bool processCommandFusionAccelerometerCalibrationSetNewPosition();
    virtual bool processCommandFusionCalibratedAccelerometerStream( bool pState );
    virtual bool processCommandFusionInclinometerCalibrate();
    virtual bool processCommandFusionInclinometerStream( bool pState );
    virtual bool processCommandFusionMagnetometerACStream ( bool pState );
    virtual bool processCommandFusionMotionIntensityTrendStream ( const NeblinaFusionMotionIntensityTrendStreamInfo_t& prMotionTrendInfo );
    virtual bool processCommandFusionClusteringInfoStream( const NeblinaFusionClusteringConfig_t& prClusteringConfig );
    virtual bool processCommandGeneralAuthentication();
    virtual bool processCommandGeneralDeviceNameGet();
    virtual bool processCommandGeneralDeviceNameSet( const char* ppDeviceName );
    virtual bool processCommandGeneralDisableStreaming();
    virtual bool processCommandGeneralFirmwareVersion();
    virtual bool processCommandGeneralFirmwareUpdate();
    virtual bool processCommandGeneralFusionStatus();
    virtual bool processCommandGeneralInterfaceState( const NeblinaInterfaceState_t& prData );
    virtual bool processCommandGeneralInterfaceStatus();
    virtual bool processCommandGeneralPowerStatus();
    virtual bool processCommandGeneralRecorderStatus();
    virtual bool processCommandGeneralResetTimestamp( uint8_t pState );
    virtual bool processCommandGeneralSensorStatus();
    virtual bool processCommandGeneralDeviceShutdown();
    virtual bool processCommandGeneralDeviceReset();
    virtual bool processCommandGeneralSystemStatus();
    virtual bool processCommandGeneralSetUnixTimestamp( const uint32_t& prTimestamp );
    virtual bool processCommandGeneralGetUnixTimestamp();
    virtual bool processCommandLEDState( const NeblinaLEDState_t& prData );
    virtual bool processCommandLEDStatus();
    virtual bool processCommandPowerBattery();
    virtual bool processCommandPowerChargeCurrent( const NeblinaPowerChargeCurrent_t& prCurrent );
    virtual bool processCommandPowerTemperature();
    virtual bool processCommandRecorderEraseAll( uint8_t pType );
    virtual bool processCommandRecorderPlayback( const NeblinaSessionStatus_t& prData );
    virtual bool processCommandRecorderRecord( const NeblinaSessionRecordNameLength_t& prCommand );
    virtual bool processCommandRecorderSessionCount();
    virtual bool processCommandRecorderSessionGeneralInfo( const uint16_t& prSessionId );
    virtual bool processCommandRecorderSessionSensorInfo( const uint16_t& prSessionId );
    virtual bool processCommandRecorderSessionName( const uint16_t& prSessionId );
    virtual bool processCommandRecorderSessionFusionInfo( const uint16_t& prSessionId );
    virtual bool processCommandRecorderSessionRead( const NeblinaSessionReadCommand_t& prCommand );
    virtual bool processCommandRecorderSessionDownload( const NeblinaSessionDownload_t& prCommand );
    virtual bool processCommandSensorGetDownsample( const NeblinaSensorStream_t& prStream );
    virtual bool processCommandSensorGetRange( const NeblinaSensorType_t& prType );
    virtual bool processCommandSensorGetRate( const NeblinaSensorType_t& prType );
    virtual bool processCommandSensorSetDownsample( const NeblinaSensorDownsample_t& prData );
    virtual bool processCommandSensorSetRange( const NeblinaSensorRange_t& prData );
    virtual bool processCommandSensorSetRate( const NeblinaSensorRate_t& prRate );
    virtual bool processCommandSensorSetBandwidth( const NeblinaSensorBandwidth_t& prBandwidth );
    virtual bool processCommandSensorGetBandwidth( const NeblinaSensorType_t& prSensor );
    virtual bool processCommandSensorAccelerometerStream( bool pState );
    virtual bool processCommandSensorAccelerometerGyroscopeStream( bool pState );
    virtual bool processCommandSensorAccelerometerMagnetometerStream( bool pState );
    virtual bool processCommandSensorGyroscopeStream( bool pState );
    virtual bool processCommandSensorHumidityStream( bool pState );
    virtual bool processCommandSensorMagnetometerStream( bool pState );
    virtual bool processCommandSensorPressureStream( bool pState );
    virtual bool processCommandSensorTemperatureStream( bool pState );
    virtual bool processDataFusionCalibratedAccelerometerStream( const NeblinaAccelerometerFxpTs_t& prData );
    virtual bool processDataFusionEulerAngleStream( const NeblinaFusionEulerFpTs_t& prEuler );
    virtual bool processDataFusionExternalForceStream( const NeblinaFusionExternalForceFxpTs_t& prExtForce );
    virtual bool processDataFusionFingerGestureStream( const NeblinaFusionFingerGestureTs_t& prFingerGesture );
    virtual bool processDataFusionInclinometerStream( const NeblinaFusionInclinometerFpTs_t& prData );
    virtual bool processDataFusionMagnetometerACStream( const NeblinaFusionMagnetometerAcFpTs_t& prData );
    virtual bool processDataFusionMotionDirectionStream( const NeblinaFusionMotionDirectionFpTs_t& prDirection );
    virtual bool processDataFusionMotionIntensityTrendStream( const NeblinaFusionMotionIntensityTrendUnixTs_t& prData );
    virtual bool processDataFusionMotionStateStream( const NeblinaFusionMotionStateTs_t& prMotionState );
    virtual bool processDataFusionPedometerStream( const NeblinaFusionPedometerFpTs_t& prPedometer );
    virtual bool processDataFusionQuaternionStream( const NeblinaFusionQuaternionFpTs_t& prQuaternion);
    virtual bool processDataFusionRotationInfoStream( const NeblinaFusionRotationInfoTs_t& prRotationInfo );
    virtual bool processDataFusionShockSegmentStream( const NeblinaAccelerometerGyroscopeFxpTs_t& prData );
    virtual bool processDataFusionSittingStandingStream( const NeblinaFusionSittingStandingTs_t& prSittingStanding );
    virtual bool processDataFusionTrajectoryInfoStream( const NeblinaFusionTrajectoryInfoFpTs_t& prTrajectoryInfo );
    virtual bool processDataRecorderSessionDownload( const NeblinaSessionDownloadData_t& prData );
    virtual bool processDataSensorAccelerometerStream( const NeblinaAccelerometerFxpTs_t& prData );
    virtual bool processDataSensorAccelerometerGyroscopeStream( const NeblinaAccelerometerGyroscopeFxpTs_t& prData );
    virtual bool processDataSensorAccelerometerMagnetometerStream( const NeblinaAccelerometerMagnetometerFxpTs_t& prData );
    virtual bool processDataSensorGyroscopeStream( const NeblinaGyroscopeFxpTs_t& prData );
    virtual bool processDataSensorHumidityStream( const NeblinaHumidityFpTs_t& pHumidity );
    virtual bool processDataSensorMagnetometerStream( const NeblinaMagnetometerFxpTs_t& prData );
    virtual bool processDataSensorPressureStream( const NeblinaPressureFpTs_t& pPressure );
    virtual bool processDataSensorTemperatureStream( const NeblinaTemperatureFpTs_t& pTemperature );
    virtual bool processError( const NeblinaPacket_t& ppPacket );
    virtual bool processResponseDebugPrintf( const uint8_t* const ppBytes, uint32_t pSize );
    virtual bool processResponseEEPROMRead( const NeblinaEEPROMData_t& prData );
    virtual bool processResponseFusionAnalysisGetActivePose( uint8_t pPoseId );
    virtual bool processResponseFusionAnalysisGetPoseInfo( const NeblinaFusionMotionAnalysisPose_t& prPose );
    virtual bool processResponseFusionAnalysisStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionCalibratedAccelerometerStream( const NeblinaSensorStreamMotionInfo_t& prInfo );
    virtual bool processResponseFusionEulerAngleStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionExternalForceStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionFingerGestureStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionInclinometerStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionMagnetometerACStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionMotionDirectionStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionMotionIntensityTrendStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionMotionStateStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionPedometerStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionQuaternionStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionRotationInfoStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionShockSegmentStream( const NeblinaSensorStreamAccelerometerGyroscopeInfo_t& prInfo );
    virtual bool processResponseFusionSittingStandingStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseFusionTrajectoryInfoStream( const NeblinaFusionStreamInfo_t& prInfo );
    virtual bool processResponseGeneralDeviceNameGet( const char* ppDeviceName );
    virtual bool processResponseGeneralFirmwareVersion( const NeblinaFirmwareVersion_t& prData );
    virtual bool processResponseGeneralFusionStatus( const NeblinaFusionStatus_t& prData );
    virtual bool processResponseGeneralInterfaceStatus( const NeblinaInterfaceStatus_t& prData );
    virtual bool processResponseGeneralPowerStatus( const NeblinaPowerStatus_t& prData );
    virtual bool processResponseGeneralRecorderStatus( const NeblinaRecorderStatus_t& prData );
    virtual bool processResponseGeneralSensorStatus( const NeblinaSensorStatus_t& prStatus );
    virtual bool processResponseGeneralSystemStatus( const NeblinaSystemStatus_t& prStatus );
    virtual bool processResponseGeneralTimestampUnixGet( uint32_t pTimestamp );
    virtual bool processResponseLEDStatus( const NeblinaLEDStatus_t& prData );
    virtual bool processResponsePowerBattery( float pStateOfCharge );
    virtual bool processResponsePowerChargeCurrent( float pCurrent );
    virtual bool processResponsePowerTemperature( float pTemperature );
    virtual bool processResponseRecorderEraseAll();
    virtual bool processResponseRecorderPlayback( const NeblinaSessionStatus_t& prData );
    virtual bool processResponseRecorderRecord( const NeblinaSessionStatus_t& prData );
    virtual bool processResponseRecorderSessionCount( uint16_t pCount );
    virtual bool processResponseRecorderSessionDownload( const NeblinaSessionStatus_t& prData );
    virtual bool processResponseRecorderSessionFusionInfo( const NeblinaSessionFusionInfo_t& prData );
    virtual bool processResponseRecorderSessionGeneralInfo( const NeblinaSessionGeneralInfo_t& prData );
    virtual bool processResponseRecorderSessionName( const char* ppName );
    virtual bool processResponseRecorderSessionRead( const NeblinaSessionReadData_t& prData );
    virtual bool processResponseRecorderSessionSensorInfo( const NeblinaSessionSensorInfo_t& prData );
    virtual bool processResponseSensorGetBandwidth( const NeblinaSensorBandwidth_t& prData );
    virtual bool processResponseSensorGetDownsample( const NeblinaSensorDownsample_t& prData );
    virtual bool processResponseSensorGetRange( const NeblinaSensorRange_t& prData );
    virtual bool processResponseSensorGetRate( const NeblinaSensorRate_t& prData );
    virtual bool processResponseSensorAccelerometerStream( const NeblinaSensorStreamMotionInfo_t& prInfo );
    virtual bool processResponseSensorAccelerometerGyroscopeStream( const NeblinaSensorStreamAccelerometerGyroscopeInfo_t& prInfo );
    virtual bool processResponseSensorAccelerometerMagnetometerStream( const NeblinaSensorStreamMotionInfo_t& prInfo );
    virtual bool processResponseSensorGyroscopeStream( const NeblinaSensorStreamMotionInfo_t& prInfo );
    virtual bool processResponseSensorHumidityStream( const NeblinaSensorStreamEnvironmentInfo_t& prInfo );
    virtual bool processResponseSensorMagnetometerStream( const NeblinaSensorStreamMotionInfo_t& prInfo );
    virtual bool processResponseSensorPressureStream( const NeblinaSensorStreamEnvironmentInfo_t& prInfo );
    virtual bool processResponseSensorTemperatureStream( const NeblinaSensorStreamEnvironmentInfo_t& prInfo );

private:
    void preparePacket( NeblinaPacket_t* const ppPacket ) const;
    bool process( const NeblinaPacket_t* const ppPacket );
    bool processCommand( const NeblinaPacket_t* const ppPacket );
    bool processCommandDebug( const NeblinaPacket_t* const ppPacket );
    bool processCommandEEPROM( const NeblinaPacket_t* const ppPacket );
    bool processCommandFusion( const NeblinaPacket_t* const ppPacket );
    bool processCommandGeneral( const NeblinaPacket_t* const ppPacket );
    bool processCommandLED( const NeblinaPacket_t* const ppPacket );
    bool processCommandPower( const NeblinaPacket_t* const ppPacket );
    bool processCommandRecorder( const NeblinaPacket_t* const ppPacket );
    bool processCommandSensor( const NeblinaPacket_t* const ppPacket );
    bool processCommandTest( const NeblinaPacket_t* const ppPacket );
    bool processData( const NeblinaPacket_t* const ppPacket );
    bool processDataFusion( const NeblinaPacket_t* const ppPacket );
    bool processDataRecorder( const NeblinaPacket_t* const ppPacket );
    bool processDataSensor( const NeblinaPacket_t* const ppPacket );    
    bool processResponse( const NeblinaPacket_t* const ppPacket );
    bool processResponseDebug( const NeblinaPacket_t* const ppPacket );
    bool processResponseEEPROM( const NeblinaPacket_t* const ppPacket );
    bool processResponseFusion( const NeblinaPacket_t* const ppPacket );
    bool processResponseGeneral( const NeblinaPacket_t* const ppPacket );
    bool processResponseLED( const NeblinaPacket_t* const ppPacket );
    bool processResponsePower( const NeblinaPacket_t* const ppPacket );
    bool processResponseRecorder( const NeblinaPacket_t* const ppPacket );
    bool processResponseSensor( const NeblinaPacket_t* const ppPacket );

    bool send( uint8_t pSubSystem, uint8_t pPacketType, uint8_t pCommand, uint8_t* ppData, uint32_t pLength );
    bool sendAck( const NeblinaPacket_t* ppPacket );
    bool sendAck( uint8_t pSubSystem, uint8_t pCommand );
    bool sendCommand( uint8_t pSubSystem, uint8_t pCommand, uint8_t* ppData, uint32_t pLength );
    bool sendCommandEEPROM( uint8_t pCommand, uint8_t* ppData, uint32_t pLength );
    bool sendCommandFusion( uint8_t pCommand, uint8_t* ppData, uint32_t pLength );
    bool sendCommandGeneral( uint8_t pCommand, uint8_t* ppData, uint32_t pLength );
    bool sendCommandLED( uint8_t pCommand, uint8_t* ppData, uint32_t pLength );
    bool sendCommandPower( uint8_t pCommand, uint8_t* ppData, uint32_t pLength );
    bool sendCommandRecorder( uint8_t pCommand, uint8_t* ppData, uint32_t pLength );
    bool sendCommandSensor( uint8_t pCommand, uint8_t* ppData, uint8_t pLength );

    bool validatePacket( NeblinaPacket_t* ppPacket ) const;

private:
    uint8_t mBuffer[NEBLINA_PACKET_LENGTH_MAX];     // Buffer of received bytes
    uint8_t mBufferSize;                            // Number of bytes stored in buffer.
    uint8_t mTxBuffer[NEBLINA_PACKET_LENGTH_MAX];   // Buffer of sent bytes

    uint32_t mRxCnt;                                // Total number of received packet.
    uint32_t mRxBadCnt;                             // Total number of bad received packet.
    uint32_t mRxFailCnt;                            // Total number of fail (unprocess) packet
    uint32_t mRxInvalidCnt;                         // Total number of invalid
    uint32_t mTxCnt;                                // Total number of sent packet.
    uint32_t mTxBadCnt;                             // Total number of dropped packet.
};

/**********************************************************************************/

} // neblina namespace

#endif // __cplusplus

/**********************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

// C function prototype
#ifdef __cplusplus
}
#endif

/**********************************************************************************/
