/**
  HLK-LD2410 library v 0.9.1
  Name: HLK_LD2410
  Purpose: Arduino library for the Hi-Link LD2410 24Ghz FMCW radar sensor

  @author Dejan Gjorgjevikj
  @version 0.9.1, 1/2024

  This sensor is a Frequency Modulated Continuous Wave radar, which makes it good for presence detection and its sensitivity at different ranges to both static and moving targets can be configured.
 
 ...todo...

Known limitations:
...

Resources:
  The code in this library was developed from scratch based on the manufacturer datasheet(s) https://drive.google.com/drive/folders/1p4dhbEJA3YubyIjIIC7wwVsSo8x29Fq-

This library is distributed in the hope that it will be useful, but
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE either express or implied.
Released into the public domain.
Released under LGPL-2.1 see https://github.com/Gjorgjevikj/HLK_LD2410/blob/master/LICENSE for full license

https://github.com/Gjorgjevikj/HLK_LD2410.git
*/


#pragma once

#ifndef _HLK_LD2410_h
#define _HLK_LD2410_h
#include <Arduino.h>

#define HLK_LD_LIB_VERSION 0.9.1

#ifndef _LOG_LEVEL 
#define _LOG_LEVEL 2
#endif

//#define LOG_FATAL(...) Serial.printf(__VA_ARGS__)
#if _LOG_LEVEL > 0
#define R_LOG_ERROR(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_ERROR(...) { }
#endif
#if _LOG_LEVEL >= 2
#define R_LOG_WARN(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_WARN(...) { }
#endif
#if _LOG_LEVEL >= 4
#define R_LOG_INFO(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_INFO(...) { }
#endif
#if _LOG_LEVEL >= 6
#define R_LOG_DEBUG(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_DEBUG(...) { }
#endif
#if _LOG_LEVEL >= 7
#define DEBUG_DUMP_FRAME(...) _dumpFrame(__VA_ARGS__)
#else
#define DEBUG_DUMP_FRAME(...) { }
#endif
#if _LOG_LEVEL > 7
#define R_LOG_VERBOSE(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_VERBOSE(...) { }
#endif

// This library is made for little endian systems only!
#define TEST_LITTLE_ENDIAN (((union { unsigned x; unsigned char c; }){1}).c)
#ifdef TEST_LITTLE_ENDIAN

#define RECEIVE_BUFFER_SIZE 80

class HLK_LD2410
{
public:
    enum FrameType : uint8_t 
    { 
        UNIDENTIFIED_FRAME = 0x00,
        ENGINEERING_DATA = 0x01,
        BASIC_DATA = 0x02,
        UNKNOWN_DATA_FORMAT = 0x0f,
        ACK_FRAME = 0xff,
        UNKNOWN_FRAME = 0xfe
    };
    enum RadarCommand : uint16_t 
    {
        ENABLE_CONFIG_MODE = 0x00FF,  // enable configuration mode
        DISABLE_CONFIG_MODE = 0x00FE,  // disable configuration mode
        SET_MAX_DIST_AND_DUR = 0x0060,  // set the maximum Distance Gate and Unmanned Duration Parameter
        READ_PARAMETER = 0x0061,  // read the parameters from the radar
        ENABLE_ENGINEERING_MODE = 0x0062,  // enable engineering mode of the radar
        DISABLE_ENGINEERING_MODE = 0x0063,  // disable engineering mode
        SET_GATE_SENS_CONFIG = 0x0064,  // set sensitivity 0-100% for moving and stationary gates
        READ_FIRMWARE_VERSION = 0x00A0,  // read  the firmware version
        SET_BAUDRATE = 0x00A1,  // set serial baud rate
        FACTORY_RESET = 0x00A2,  // Factory Reset
        RESTART = 0x00A3,  // Restart the radar
        SET_BLUETOOTH = 0x00A4, // Turn on/off bluetooth
        GET_MAC_ADDRESS = 0x00A5 // Read the MAC ADDRES
    };

    /// <summary>
    /// Communication speed to the radar sensor UART
    /// </summary>
    enum BaudRate : uint16_t
    {
        BAUD_9600   = 0X0001,
        BAUD_19200  = 0X0002,
        BAUD_38400  = 0X0003,
        BAUD_57600  = 0X0004,
        BAUD_115200 = 0X0005,
        BAUD_230400 = 0X0006,
        BAUD_256000 = 0X0007,  // radars default baud rate
        BAUD_460800 = 0X0008
    };
    enum GateType : uint32_t // for setSensitivity
    {
        GATE_0 = 0x00000000,
        GATE_1 = 0x00000001,
        GATE_2 = 0x00000002,
        GATE_3 = 0x00000003,
        GATE_4 = 0x00000004,
        GATE_5 = 0x00000005,
        GATE_6 = 0x00000006,
        GATE_7 = 0x00000007,
        GATE_8 = 0x00000008,
        GATE_ALL = 0x0000ffff
    };
    enum FrameField : uint32_t
    {
        CMD_FRAME_HEADER  = 0xFAFBFCFD,
        CMD_FRAME_TRAILER = 0x01020304,
        ACK_FRAME_HEADER = 0xFAFBFCFD,
        ACK_FRAME_TRAILER = 0x01020304,
        DAT_FRAME_HEADER = 0xF1F2F3F4,
        DAT_FRAME_TRAILER = 0xF5F6F7F8
    };
    enum FrameHeadByte : uint8_t
    {
        ACK_FRAME_HEAD_BYTE = 0xfd,
        DAT_FRAME_HEAD_BYTE = 0xf4
    };
    enum BluetoothMode : uint16_t
    {
        BluetoothOff = 0X000,
        BluetoothOn = 0X0001
    };
    enum FrameCL : bool
    {
        CURRENT_FRAME = false,
        LAST_FRAME = true
    };

#pragma pack (1)
    // frame header / trailer 
    union FrameMarkerType
    {
        uint32_t frameWord;
        uint8_t frameBytes[4];
    };
    // for identifying the frame type and getting the length
    struct FrameStart
    {
        FrameMarkerType header;
        uint16_t ifDataLength;
        uint8_t modeType; // if dataframe - 0x01 Enginnering data, 0x02 Target basic information
    };
    struct REQframeCommand
    {
        FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
        uint16_t ifDataLength;
        uint16_t commandWord;
        FrameMarkerType trailer; // 04 03 02 01 0x01020304
    };
    struct REQframeCommandWithValue
    {
        FrameMarkerType header; // FD FC FB FA == 0xfafbfcfd
        uint16_t ifDataLength; // 04 00 == 0x0004
        uint16_t commandWord; // FF 00 == 0x00ff
        uint16_t commandValue; // 01 00 == 0x0001
        FrameMarkerType trailer; // 04 03 02 01 == 0x01020304
    };
    struct ACKframeCommand
    {
        FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
        uint16_t ifDataLength; // 04 00 == 0x0004
        uint16_t commandReply;
        uint16_t ackStatus;
        FrameMarkerType trailer; // 04 03 02 01 0x01020304
    };
    struct ACKframeEnableConfig
    {
        FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
        uint16_t ifDataLength; // 08 00 == 0x0008
        uint16_t commandReply; // FF 01 == 0x01ff
        uint16_t ackStatus; // 0 / 1
        uint16_t protocolVer; // 0x0001
        uint16_t bufferSize; // 0x0040
        FrameMarkerType trailer; // 04 03 02 01 0x01020304
    };

    /// <summary>
    /// struct representing the firmware version of the radar sensor (contains the firmware type, major, minor and bugfix parts of the version)
    /// </summary>
    struct FirmwareVersion
    {
        uint16_t type;    // firmware type
        uint8_t minor;    // minor version of the radar firmware
        uint8_t major;    // major version of the radar firmware
        uint32_t bugfix;  // bug fix version of the radar firmware
    };
    struct ACKframeFirmwareVersion
    {
        FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
        uint16_t ifDataLength;
        uint16_t commandReply;
        uint16_t ackStatus;
        FirmwareVersion version;
        FrameMarkerType trailer; // 04 03 02 01 0x01020304
    };

    struct TargetData
    {
        uint16_t distance;
        uint8_t energy;
    };
    struct ReportingFrame // LD2410B uses little-endian format
    {
        FrameMarkerType header; // F4 F3 F2 F1 0xf1f2f3f4
        uint16_t ifDataLength;
        uint8_t modeType; // 0x01 Enginnering data, 0x02 Target basic information
        uint8_t ifHead; // 0xAA
        // intraframe begin - target data
        uint8_t targetState; // 0x00 No target, 0x01 Moving, 0x02 Stationary, 0x03 Moving+Stationary target
        TargetData movingTarget;
        TargetData stationaryTarget;
        uint16_t detectionDistance;
        // intraframe end
        uint8_t ifTail; // 0x55
        uint8_t ifCheck; // 0x00
        FrameMarkerType trailer; // F8 F7 F6 F5 0xf5f6f7f8
    };
    struct EngineeringFrame
    {
        FrameMarkerType header; // F4 F3 F2 F1 0xf1f2f3f4
        uint16_t ifDataLength;
        uint8_t modeType; // 0x01 Enginnering data, 0x02 Target basic information
        uint8_t ifHead; // 0xAA
        // target data
        uint8_t targetState; // 0x00 No target, 0x01 Moving, 0x02 Stationary, 0x03 Moving+Stationary target
        TargetData movingTarget;
        TargetData stationaryTarget;
        uint16_t detectionDistance;
        // engineering data
        uint8_t maxMovingDistanceGateN;
        uint8_t maxStaticDistanceGateN;
        uint8_t movementDistanceEnergy[9];
        uint8_t staticDistanceEnergy[9];
        uint16_t addInfo; // ??
        // intraframe end
        uint8_t ifTail; // 0x55
        uint8_t ifCheck; // 0x00
        FrameMarkerType trailer; // F8 F7 F6 F5 0xf5f6f7f8
    };

    /// <summary>
    /// struct representing the parameters of the radar sensor (#gates, motionSensitivity and StaticSensitivity per gate, noPersonDuration)
    /// </summary>
    struct RadarParameters
    {                                             // byte# in frame
        uint8_t maxDistanceGateN;                 //    12  = 08    == 0x08
        uint8_t maxMovingDistanceGateN;           //    13  = 08    == 0x08
        uint8_t maxStaticDistanceGateN;           //    14  = 08    == 0x08
        uint8_t motionSensitivityDistanceGate[9]; // 15-23  
        uint8_t restSensitivityDistanceGate[9];   // 24-33
        uint16_t noPersonDuration;                // 34-35
    };
    struct ACKframeParameter
    {                                             // byte#
        FrameMarkerType header;                   //  1- 4  = FD FC FB FA == 0xfafbfcfd
        uint16_t ifDataLength;                    //  5- 6  = 1c 00 == 0x001c (28) 
        uint16_t commandReply;                    //  7- 8  = 61 01 == 0x0161
        uint16_t ackStatus;                       //  9-10  = 00 01 == 0x0000 / 0x0001
        uint8_t ifHead;                           //    11  = AA    == 0xaa
        RadarParameters param;                    // 12-35
        FrameMarkerType trailer;                  // 36-39  = 04 03 02 01 == 0x01020304
    };

    struct REQframeSetParameters
    {
        FrameMarkerType header;                   //  1- 4  = FD FC FB FA == 0xfafbfcfd
        uint16_t ifDataLength;                    //  5- 6  = 14 00 == 0x0014 (20)
        uint16_t commandWord;                     //  7- 8  = 60 00 == 0x0060
        uint16_t distanceGatePar0;                //  9-10  = 00    == 0x0000
        uint32_t distanceGateVal0;                // 11-14  = xx 00 00 00 == 0x000000xx
        uint16_t distanceGatePar1;                // 15-16  = 01    == 0x0001
        uint32_t distanceGateVal1;                // 17-20  = xx 00 00 00 == 0x000000xx
        uint16_t distanceGatePar2;                // 21-22  = 02    == 0x0002
        uint32_t distanceGateVal2;                // 23-26  = xx 00 00 00 == 0x000000xx
        FrameMarkerType trailer;                  // 27-30  = 04 03 02 01 == 0x01020304
    };

#pragma pack (0)

public:
    /*
    const FrameMarkerType headerCMD = { 0xFAFBFCFD };
    const FrameMarkerType trailerCMD = { 0x01020304 };
    const FrameMarkerType headerACK = { 0xFAFBFCFD };
    const FrameMarkerType trailerACK = { 0x01020304 };
    const FrameMarkerType headerDAT = { 0xF1F2F3F4 };
    const FrameMarkerType trailerDAT = { 0xF5F6F7F8 };
    */
    
    /// <summary>
    /// Class representing the radar sensor 
    /// communicates to the radar through the UART
    /// </summary>
    HLK_LD2410();

    /// <summary>
    /// Initialize the radar sensor and associate it to UART 
    /// </summary>
    /// <param name="rStream">Stream object representing the UART</param>
    /// <remarks>HAs to be initialized to he appropriate speed before calling begin()</remarks>
    /// <returns>true on sucess</returns>
    bool begin(Stream& rStream);

    /// <summary>
    /// Sets the timeout in milliseconds
    /// </summary>
    /// <param name="timeout">Time in ms to wait for a response (ACK fame)</param>
    void setFrameTimeOut(unsigned long timeout) { frameTimeOut = timeout; }

    /// <summary>
    /// Returns the timeout in milliseconds
    /// </summary>
    /// <returns>Time in ms to wait for a response (ACK fame)</returns>
    unsigned long getFrameTimeOut() const { return frameTimeOut; }

    /// <summary>
    /// Sets the communication speed of the sensor  
    /// </summary>
    /// <remarks>Sends the command Set Serial Port Baud Rate to the sensor and waits for reply (ACK frame) from the sensor</remarks>
    /// <param name="rate">speed in Bits per second HLK_LD2410::BAUD_xxxx {
    /// <list type="bullet">
    /// <item>BAUD_9600, </item>
    /// <item>BAUD_19200, </item>
    /// <item>BAUD_38400, </item>
    /// <item>BAUD_57600, </item>
    /// <item>BAUD_115200, </item>
    /// <item>BAUD_230400, </item>
    /// <item>BAUD_256000, </item>
    /// <item>BAUD_460800</item>
    /// </list>
    /// }</param>
    /// <returns>true on sucess</returns>
    bool setBaudRate(BaudRate rate) { return sendCommand(SET_BAUDRATE, rate); }
    
    /// <summary>
    /// Enables the Enginnering mode on the radar sesnsor  
    /// </summary>
    /// <remarks>Sends the command to the sensor to enter engineering mode and waits for reply (ACK frame) from the sensor</remarks>
    /// <remarks>The sensor should start reoprting engneering data after this</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <returns>true on sucess</returns>
    bool enableEngineeringMode() { return sendCommand(ENABLE_ENGINEERING_MODE); }

    /// <summary>
    /// Disable the Enginnering mode on the radar sesnsor  
    /// </summary>
    /// <remarks>Sends the command to the sensor to exit engineering mode (goes back to basic target reporting mode) and waits for reply (ACK frame) from the sensor</remarks>
    /// <remarks>The sensor should start reoprting basic target data after this</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <returns>true on sucess</returns>
    bool disableEngineeringMode() { return sendCommand(DISABLE_ENGINEERING_MODE); }
    
    /// <summary>
    /// Resets the radar sensor - perfoms a factory reset to defaults 
    /// </summary>
    /// <remarks>Sends the reset command to the sensor and waits for reply (ACK frame) from the sensor</remarks>
    /// <remarks>Restores all configuration values of the sensor to their default values (will become active after sensor restart)</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <returns>true on sucess</returns>
    bool reset() { return sendCommand(FACTORY_RESET); } // reset device settings to defaults
    
    /// <summary>
    /// Restart the radar sensor 
    /// </summary>
    /// <remarks>Sends the restart command to the sensor and waits for reply (ACK frame) from the sensor</remarks>
    /// <returns>true on sucess</returns>
    bool restart() { return sendCommand(RESTART); } // resart the device 

    /// <summary>
    /// Returns the firmware version of the radar sensor
    /// </summary>
    /// <remarks>Sends the Read Firmware Version command to the sensor and waits for reply from the sensor</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <returns><see cref="HLK_LD2410::FirmwareVersion">FirmwareVersion</see> struct containing the firmware type, major, minor and bugfix parts of the version</returns>
    FirmwareVersion reqFirmwareVersion();

    /// <summary>
    /// Returns the parameters of the radar sensor
    /// </summary>
    /// <remarks>Sends the Read Parameter command to the sensor and waits for reply from the sensor</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <returns><see cref="HLK_LD2410::RadarParameters">RadarParameters</see> struct containing all the parameters of the sensor (#gates, motionSensitivity and StaticSensitivity per gate, noPersonDuration)</returns>
    RadarParameters reqParameters();

    /// <summary>
    /// Sets the maximum distance and no-person duration for the radar sensor
    /// </summary>
    /// <remarks>Sends the Maximum distance gate and unmanned duration parameter configuration command to the sensor and waits for reply from the sensor</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <param name="maxMovingDistanceGate">Gate number (0-8)</param>
    /// <param name="maxRestingDistanceGate">Gate number (2-8)</param>
    /// <param name="noPersonDuration">noPersonDuration (0-65535 seconds)</param>
    /// <returns>true on sucess</returns>
    bool setDistanceAndDuration(uint32_t maxMovingDistanceGate, uint32_t maxRestingDistanceGate, uint32_t noPersonDuration) {
        return setParameters(SET_MAX_DIST_AND_DUR, maxMovingDistanceGate, maxRestingDistanceGate, noPersonDuration);
    }

    /// <summary>
    /// Sets the sensitivity of the radar sensor for a specified gate (distance)
    /// </summary>
    /// <remarks>Sends the Range Gate Sensitivity Configuration command to the sensor and waits for reply from the sensor</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <param name="distanceGate">The gate number (0-8)</param>
    /// <param name="distanceGate">motion sensitivity (0-100)</param>
    /// <param name="distanceGate">resting sensitivity (0-100)</param>
    /// <returns>true on sucess</returns>
    bool setSensitivity(uint32_t distanceGate, uint32_t motionSensitivity, uint32_t restingSensitivity) {
        return setParameters(SET_GATE_SENS_CONFIG, distanceGate, motionSensitivity, restingSensitivity);
    }

    // not supported on firmware v1.07.22082218
    //bool setBluetooth(BluetoothMode state) { return sendCommand(SET_BLUETOOTH, state); } // turn on/off bluetooth
    //getMacAddress(); // todo

    /// <summary>
    /// Reads the data coming from the radar sensor
    /// </summary>
    /// <remarks>Reads the data coming from the radar about the sensed target and stores it in a buffer</remarks>
    /// <remarks>To be called repeaditly in the loop()</remarks>
    /// <returns>number of bytes read</returns>
    int read();

    /// <summary>
    /// Returns the timstamp (millis) of the current/last frame 
    /// </summary>
    /// <remarks>can select current or previous frame</remarks>
    /// <param name="frame">frame selector: true (default) for the last cmoplitely received frame / false for the current frame</param>
    /// <returns>timstamp (millis) at which the first byte of the frame has been received</returns>
    unsigned long frameStartMillis(bool frame = true) const { return frame ? lastFrameStartTS : currentFrameStartTS; } // time start of last frame by default

    /// <summary>
    /// Returns the type of the current/last frame 
    /// </summary>
    /// <remarks>can select current or previous frame</remarks>
    /// <param name="frame">frame selector: true (default) for the last cmoplitely received frame / false for the current frame</param>
    /// <returns>FrameType value { 
    /// <list type="bullet">
    /// <item>UNIDENTIFIED_FRAME, </item>
    /// <item>ENGINEERING_DATA, </item>
    /// <item>BASIC_DATA, </item>
    /// <item>UNKNOWN_DATA_FORMAT, </item>
    /// <item>ACK_FRAME, </item>
    /// <item>UNKNOWN_FRAME </item>
    /// </list>
    /// }</returns>
    FrameType frameType(bool frame = true) const; // type of the last completely received frame by default

    /// <summary>
    /// Enter the configuration mode of the radar sesnsor
    /// </summary>
    /// <remarks>Sends the command to the sensor to enable configuration mode and waits for reply (ACK frame) from the sensor</remarks>
    /// <returns>true on sucess</returns>
    bool enableConfigMode();

    /// <summary>
    /// Exit the configuration mode of the radar sesnsor
    /// </summary>
    /// <remarks>Sends the command to the sensor to end configuration mode</remarks>
    /// <returns>true on sucess</returns>
    bool disableConfigMode();

    /// <summary>
    /// Returns data about the targets
    /// </summary>
    /// <remarks>Note: returns the data from the last completely received packet (that can be up to 70ms old)</remarks>
    /// <returns>target state: encoded as movingTarget(0/1), stationaryTarget(0/1) existance in the 2 LSB</returns>
    uint8_t target() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->targetState; }

    /// <summary>
    /// Has a movement been registered by the radar sensor
    /// </summary>
    /// <remarks>Note: returns the data from the last completely received packet (that can be up to 70ms old)</remarks>
    /// <returns>moving target detected - true / false</returns>
    bool movingTarget() const { return ((reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->targetState) & 0b01; }

    /// <summary>
    /// Has a stationary target (resting person) been registered by the radar sensor
    /// </summary>
    /// <remarks>Note: returns the data from the last completely received packet (that can be up to 70ms old)</remarks>
    /// <returns>stationary target detected true / false</returns>
    bool stationaryTarget() const { return ((reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->targetState) & 0b10; }

    /// <summary>
    /// Returns the distance to the moving target registered by the radar sensor
    /// </summary>
    /// <remarks>Note: returns the data from the last completely received packet (that can be up to 70ms old)</remarks>
    /// <returns>the distance (in cm)</returns>
    uint16_t movingTargetDistance() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->movingTarget.distance; }

    /// <summary>
    /// Returns the energy of the moving target registered by the radar sensor
    /// </summary>
    /// <remarks>Note: returns the data from the last completely received packet (that can be up to 70ms old)</remarks>
    /// <returns>energy value (0-100)</returns>
    uint8_t movingTargetEnergy() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->movingTarget.energy; }

    /// <summary>
    /// Returns the distance to the staionary (sresting) target registered by the radar sensor
    /// </summary>
    /// <remarks>Note: returns the data from the last completely received packet (that can be up to 70ms old)</remarks>
    /// <returns>the distance (in cm)</returns>
    uint16_t stationaryTargetDistance() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->stationaryTarget.distance; }

    /// <summary>
    /// Returns the energy of the staionary (resting) target registered by the radar sensor
    /// </summary>
    /// <remarks>Note: returns the data from the last completely received packet (that can be up to 70ms old)</remarks>
    /// <returns>energy value (0-100)</returns>
    uint8_t stationaryTargetEnergy() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->stationaryTarget.energy; }

    /// <summary>
    /// Returns the detection distance reported by the radar sensor
    /// </summary>
    /// <remarks>Note: returns the data from the last completely received packet (that can be up to 70ms old)</remarks>
    /// <returns>distance (in cm)</returns>
    uint16_t detectionDistance() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->detectionDistance; }

    /// <summary>
    /// Returns the number of gates for movement registartion as configured in radar sensor
    /// </summary>
    /// <returns>number of configured gates</returns>
    uint8_t maxMovingDistanceGates() const { return (reinterpret_cast<const EngineeringFrame*>(bufferLastFrame))->maxMovingDistanceGateN; }

    /// <summary>
    /// Returns the number of gates for static presence registartion as configured in radar sensor
    /// </summary>
    /// <returns>number of configured gates</returns>
    uint8_t maxStaticDistanceGates() const { return (reinterpret_cast<const EngineeringFrame*>(bufferLastFrame))->maxStaticDistanceGateN; }

    /// <summary>
    /// Returns the energy of the moving target at given distance
    /// </summary>
    /// <param name="n">the number of the gate (default 75cm per gate)</param>
    /// <returns>energy (0-100)</returns>
    uint8_t movementEnergyAtDistance(int n) const { return (reinterpret_cast<const EngineeringFrame*>(bufferLastFrame))->movementDistanceEnergy[n]; }

    /// <summary>
    /// Returns the energy of the stationary (resting) target at given distance
    /// </summary>
    /// <param name="n">the number of the gate (default 75cm per gate)</param>
    /// <returns>energy (0-100)</returns>
    uint8_t restingEnergyAtDistance(int n) const { return (reinterpret_cast<const EngineeringFrame*>(bufferLastFrame))->staticDistanceEnergy[n]; }
    
//private:

    /// <summary>
    /// Sends raw data to the sensor
    /// </summary>
    /// <param name="data">pointer to the data buffer</param>
    /// <param name="size">number of butes to send</param>
    void write(const uint8_t* data, int size); //send raw data

    /// <summary>
    /// Waits for the response from the radar (ACK frame) after issuing a command to the radar
    /// </summary>
    /// <remarks>Reads the incoming data of the ACK frame and stores it in the buffer and checks its validity</remarks>
    void readAckFrame();

    /// <summary>
    /// Issues a command to the radar
    /// </summary>
    /// <remarks>Sends the given command to the radar sensor waits for the ACK frame and checks its validity</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <param name="command">command to be sent (see <see cref="enum RadarCommand">enum RadarCommand</see>)</param>
    /// <returns>true on sucess</returns>
    bool sendCommand(RadarCommand command);

    /// <summary>
    /// Issues a command with parameter to the radar
    /// </summary>
    /// <remarks>Sends the given command + parameter to the radar sensor waits for the ACK frame and checks its validity</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <param name="command">command to be sent (see <see cref="enum RadarCommand">enum RadarCommand</see>)</param>
    /// <param name="value">value fo the parameter</param>
    /// <returns>true on sucess</returns>
    bool sendCommand(RadarCommand command, uint16_t value);

    /// <summary>
    /// Waits for an accknoledgement for an issued command
    /// </summary>
    /// <remarks>Waits for an accknoledgement for an issued command to radar sensor waits for the ACK frame and checks its validity</remarks>
    /// <param name="command">command to be sent (see <see cref="enum RadarCommand">enum RadarCommand</see>)</param>
    /// <param name="remainInConfig">wherethere to remain on config mode after ACK has been received</param>
    /// <returns>true on sucess</returns>
    bool commandAccknowledged(RadarCommand command, bool remainInConfig);

    /// <summary>
    /// Setting the parameters of the radar sensor
    /// </summary>
    /// <remarks>Sends the command to the sensor for setting radar parameters + the values for the parameters and waits for reply from the sensor</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <param name="command">command to be sent (see <see cref="enum RadarCommand">enum RadarCommand</see>)</param>
    /// <param name="par1">value for the first parameter</param>
    /// <param name="par2">value for the second parameter</param>
    /// <param name="par3">value for the third parameter</param>
    /// <returns>true on sucess</returns>
    bool setParameters(RadarCommand command, uint32_t par0, uint32_t par1, uint32_t par2);


    /// <summary>
    /// DEBUGGING: Dumps the butes of the last complitely received frame 
    /// </summary>
    void _dumpLastFrame(String label = "", Stream& dumpStream = Serial) const {
        DEBUG_DUMP_FRAME(bufferLastFrame, (reinterpret_cast<const FrameStart*>(bufferLastFrame))->ifDataLength + 10, label + "[", "]", dumpStream);
    }

    /// <summary>
    /// DEBUGGING: Dumps the butes of the last current (ongoing) frame 
    /// </summary>
    void _dumpCurrentFrame(String label = "", Stream& dumpStream = Serial) const {
        DEBUG_DUMP_FRAME(bufferCurrentFrame, currentFrameIndex, label + "[", "]", dumpStream);
    }

    /// <summary>
    /// DEBUGGING: Dumps a buffer as a frame 
    /// </summary>
    static void _dumpFrame(const uint8_t* buff, int len, String pre = "", String post = "", Stream& dumpStream = Serial);

private:
    Stream* radarUART; 
    uint8_t receiveBuffer[2][RECEIVE_BUFFER_SIZE];
    uint8_t* bufferLastFrame; // pointer to the buffer where the last finished frame is stored
    uint8_t* bufferCurrentFrame; // pointer to the buffer where the next (or currently beeing recived) frame is to be stored
    unsigned long currentFrameStartTS; // the timestamp (milliseconds) when the first bute of the current frame has been recived
    unsigned long lastFrameStartTS; // the timestamp (milliseconds) when the first bute of the last finished frame has been recived
    unsigned long frameTimeOut; // user settable - timeput in milliseconds to wait for reply (ACK frame) after issuing a commnad
    int receivedFrameLen; // the actual length of the last received frame 
    int currentFrameIndex; // the index to the buffer position wher the next received byte is going to be written
    bool inConfigMode; // is tha radar sensor in config mode
    bool inFrame; // has the receiving of the frame begun
    bool frameReady; // is the frame finished (all bytes recived of the current frame)
};

#else
#error "This library is made for little endian systems only, and it seems it is beeing compiled for big endian system!" 
#endif //TEST_LITTLE_ENDIAN

#endif //_HLK_LD2410_h
