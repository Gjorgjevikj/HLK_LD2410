/**
  HLK-LD2410 library v 0.8.0
  Name: HLK_LD2410
  Purpose: Arduino library for the Hi-Link LD2410 24Ghz FMCW radar sensor

  @author Dejan Gjorgjevikj
  @version 0.8.0, 1/2024

  This sensor is a Frequency Modulated Continuous Wave radar, which makes it good for presence detection and its sensitivity at different ranges to both static and moving targets can be configured.
 
 ...todo...

Known limitations:
...

Credits:
  The code in this library is based on the manufacturer datasheet(s) https://drive.google.com/drive/folders/1p4dhbEJA3YubyIjIIC7wwVsSo8x29Fq-
  the LD2140 library from renstec https://github.com/Renstec/LD2410
  ?? initial piece of work for ESPHome https://github.com/rain931215/ESPHome-LD2410.
  ?? the ld2410 library from ncmreynolds https://github.com/ncmreynolds/ld2410

This library is distributed in the hope that it will be useful, but
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE either express or implied.
Released into the public domain.
Released under LGPL-2.1 see https://github.com/ncmreynolds/ld2410/LICENSE for full license

https://github.com/Gjorgjevikj/_tbd_.git
*/


#pragma once

#ifndef _HLK_LD2410_h
#define _HLK_LD2410_h
#include <Arduino.h>

#ifndef _LOG_LEVEL 
#define _LOG_LEVEL 2
#endif

#define LOG_FATAL(...) Serial.printf(__VA_ARGS__)
#if _LOG_LEVEL > 0
#define LOG_ERROR(...) Serial.printf(__VA_ARGS__)
#else
#define LOG_ERROR(...) { }
#endif
#if _LOG_LEVEL >= 2
#define LOG_WARN(...) Serial.printf(__VA_ARGS__)
#else
#define LOG_WARN(...) { }
#endif
#if _LOG_LEVEL >= 4
#define LOG_INFO(...) Serial.printf(__VA_ARGS__)
#else
#define LOG_INFO(...) { }
#endif
#if _LOG_LEVEL >= 6
#define LOG_DEBUG(...) Serial.printf(__VA_ARGS__)
#else
#define LOG_DEBUG(...) { }
#endif
#if _LOG_LEVEL >= 7
#define DEBUG_DUMP_FRAME(...) _dumpFrame(__VA_ARGS__)
#else
#define DEBUG_DUMP_FRAME(...) { }
#endif
#if _LOG_LEVEL > 7
#define LOG_VERBOSE(...) Serial.printf(__VA_ARGS__)
#else
#define LOG_VERBOSE(...) { }
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
    enum BaudRate : uint16_t
    {
        BAUD_9600 = 0X0001,
        BAUD_19200 = 0X0002,
        BAUD_38400 = 0X0003,
        BAUD_57600 = 0X0004,
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
    HLK_LD2410();
    bool begin(Stream& rStream);

    void setFrameTimeOut(unsigned long to) { frameTimeOut = to; }
    unsigned long getFrameTimeOut() const { return frameTimeOut; }

    bool setBaudRate(BaudRate rate) { return sendCommand(SET_BAUDRATE, rate); }
    bool enableEngineeringMode() { return sendCommand(ENABLE_ENGINEERING_MODE); }
    bool disableEngineeringMode() { return sendCommand(DISABLE_ENGINEERING_MODE); }
    bool reset() { return sendCommand(FACTORY_RESET); } // reset device settings to defaults
    bool restart() { return sendCommand(RESTART); } // resart the device 
    FirmwareVersion reqFirmwareVersion();
    RadarParameters reqParameters();
    bool setDistanceAndDuration(uint32_t maxMovingDistanceGate, uint32_t maxRestingDistanceGate, uint32_t noPersonDuration) {
        return setParameters(SET_MAX_DIST_AND_DUR, maxMovingDistanceGate, maxRestingDistanceGate, noPersonDuration);
    }
    bool setSensitivity(uint32_t distanceGate, uint32_t motionSensitivity, uint32_t restingSensitivity) {
        return setParameters(SET_GATE_SENS_CONFIG, distanceGate, motionSensitivity, restingSensitivity);
    }
    // not supported on firmware v1.07.22082218
    //bool setBluetooth(BluetoothMode state) { return sendCommand(SET_BLUETOOTH, state); } // turn on/off bluetooth
    //getMacAddress(); // todo

    int read();
    unsigned long frameStartMillis() const { return frameStartTS; }
    FrameType frameType(bool f = true) const; // type of the last complitely received frame by default

    bool enableConfigMode();
    bool disableConfigMode();

    uint8_t target() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->targetState; }
    bool movingTarget() const { return ((reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->targetState) & 0b01; }
    bool stationaryTarget() const { return ((reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->targetState) & 0b10; }
    uint16_t movingTargetDistance() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->movingTarget.distance; }
    uint8_t movingTargetEnergy() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->movingTarget.energy; }
    uint16_t stationaryTargetDistance() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->stationaryTarget.distance; }
    uint8_t stationaryTargetEnergy() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->stationaryTarget.energy; }
    uint16_t detectionDistance() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->detectionDistance; }

    uint8_t maxMovingDistanceGates() const { return (reinterpret_cast<const EngineeringFrame*>(bufferLastFrame))->maxMovingDistanceGateN; }
    uint8_t maxStaticDistanceGates() const { return (reinterpret_cast<const EngineeringFrame*>(bufferLastFrame))->maxStaticDistanceGateN; }
    uint8_t movementEnergyAtDistance(int n) const { return (reinterpret_cast<const EngineeringFrame*>(bufferLastFrame))->movementDistanceEnergy[n]; }
    uint8_t restingEnergyAtDistance(int n) const { return (reinterpret_cast<const EngineeringFrame*>(bufferLastFrame))->staticDistanceEnergy[n]; }
    
//private:
    void write(const uint8_t* data, int size); //send raw data
    void readAckFrame();

    bool sendCommand(RadarCommand);
    bool sendCommand(RadarCommand cmd, uint16_t value);
    bool commandAccknowledged(RadarCommand, bool);

    bool setParameters(RadarCommand command, uint32_t maxMovingDistanceGate, uint32_t maxRestingDistanceGate, uint32_t noPersonDuration);

    void _dumpLastFrame(String label = "", Stream& dumpStream = Serial) const {
        DEBUG_DUMP_FRAME(bufferLastFrame, (reinterpret_cast<const FrameStart*>(bufferLastFrame))->ifDataLength + 10, label + "[", "]", dumpStream);
    }
    void _dumpCurrentFrame(String label = "", Stream& dumpStream = Serial) const {
        DEBUG_DUMP_FRAME(bufferCurrentFrame, currentFrameIndex, label + "[", "]", dumpStream);
    }
    static void _dumpFrame(const uint8_t* buff, int len, String pre = "", String post = "", Stream& dumpStream = Serial);

private:
    Stream* radarUART;
    uint8_t receiveBuffer[2][RECEIVE_BUFFER_SIZE];
    uint8_t* bufferLastFrame;
    uint8_t* bufferCurrentFrame;
    unsigned long frameStartTS;
    unsigned long frameTimeOut;
    int receivedFrameLen;
    int currentFrameIndex;
    bool inConfigMode;
};

#else
#error "This library is made for little endian systems only, and it seems it is beeing compiled for big endian system!" 
#endif

#endif
