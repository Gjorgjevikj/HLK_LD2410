/*
    Name:       esp32ld2410a.ino
    Created:	13.1.2024 01:30:45
    Author:     DESKTOP-I9\Dejan
*/


#if defined(ESP32)
#if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1
#define rSerial Serial1
#define RADAR_RX_PIN 26
//#define RADAR_RX_PIN 32
#define RADAR_TX_PIN 27
//#define RADAR_TX_PIN33
#elif CONFIG_IDF_TARGET_ESP32S2
#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1
#define RADAR_RX_PIN 9
#define RADAR_TX_PIN 8
#elif CONFIG_IDF_TARGET_ESP32C3
#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1
#define RADAR_RX_PIN 4
#define RADAR_TX_PIN 5
#else 
#error Target CONFIG_IDF_TARGET is not supported
#endif
#else // ESP32 Before IDF 4.0
#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1
#define RADAR_RX_PIN 32
#define RADAR_TX_PIN 33
#endif

#include <HLK_LD2410.h>

HLK_LD2410 radar;
unsigned int last_frame_ts = 0;

void setup()
{
    Serial.begin(115200); //Feedback over Serial Monitor
    rSerial.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN); //UART for monitoring the radar
    
    Serial.println("...will start in 2s.");
    delay(2000);

    radar.begin(rSerial);
    Serial.printf("sizeof(ReportingFrame)=%d\n", sizeof(HLK_LD2410::ReportingFrame));
    Serial.printf("sizeof(EngineeringFrame)=%d\n", sizeof(HLK_LD2410::EngineeringFrame));
    delay(200);

    // out of loop reading
    //Serial.println("read one frame\n");
    //wait for current frame to end
    /*
    while (radar.frameInProgressBytesArrived()) // & timeout
    {
        radar.read();
    }
    //wait for header and length to arrive
    while (radar.frameInProgressIntraFrameLength()==0) // & timeout
    {
        radar.read();
    }
    int expLen = radar.frameInProgressIntraFrameLength() + 10; // 10 = 4 bytes header + 2 length + 4 trailer 
    while (radar.frameInProgressBytesArrived() < expLen) // & timeout
    {
        radar.read();
    }
    Serial.printf("frame type=%d captured\n", radar.frameInProgressType());
    radar.dumpCurrentFrame(Serial);
    */
    
    
    Serial.println("Enabling config mode...");
    if (radar.enableConfigMode())
        Serial.println("Config mode enabled");
    else
        Serial.println("Setting config mode failed");

/* {
        Serial.println("Waiting for 1s.");
        unsigned long t = millis();
        while (millis() - t < 1000)
        {
            radar.read();
            if (last_frame_ts < radar.frameStartMillis())
            {
                last_frame_ts = radar.frameStartMillis();
                radar.dumpLastFrame();
            }
        }
        Serial.println("==Endwait.");
    }*/
    
    Serial.println("Reading firmware version...");
    HLK_LD2410::FirmwareVersion ver = radar.reqFirmwareVersion();
    Serial.printf("FirmwareVersion %x.%02x.%08x (type%x)\n", ver.major, ver.minor, ver.bugfix, ver.type);
    delay(200);

    Serial.println("Setting parameters...");
    if(radar.setDistanceAndDuration(8, 8, 2))
        Serial.println("setDistanceAndDuration(...) succsess");
    else
        Serial.println("setDistanceAndDuration(...) failed");
    delay(200);

    Serial.println("Setting parameters...");
    if (radar.setSensitivity(8, 15, 20))
        Serial.println("setSensitivity(...) succsess");
    else
        Serial.println("setSensitivity(...) failed");
    delay(200);

    Serial.println("Reading radar parameters...");
    HLK_LD2410::RadarParameters par = radar.reqParameters();
    if (par.maxDistanceGateN)
    {
        Serial.printf("Radar parameters:\n");
        Serial.printf("maxDistanceGateN=%d\n", par.maxDistanceGateN);
        Serial.printf("maxMovingDistanceGateN=%d\n", par.maxMovingDistanceGateN);
        Serial.printf("maxStaticDistanceGateN=%d\n", par.maxStaticDistanceGateN);
        for (int i = 0; i < 9; i++)
            Serial.printf("motionSensitivityDistanceGate[%d]=%d\n", i, par.motionSensitivityDistanceGate[i]);
        for (int i = 0; i < 9; i++)
            Serial.printf("restSensitivityDistanceGate[%d]=%d\n", i, par.restSensitivityDistanceGate[i]);
        Serial.printf("noPersonDuration=%d\n", par.noPersonDuration);
    }
    else
    {
        Serial.println("reqParameters(...) failed");
    }
    delay(500);
    
    if(radar.enableEngineeringMode())
        Serial.println("Enabling EngineeringMode sucess");
    else
        Serial.println("Enabling EngineeringMode failed");
    


/*
    {
        Serial.println("Waiting for 1s.");
        unsigned long t = millis();
        while (millis() - t < 1000)
        {
            radar.read();
            if (last_frame_ts < radar.frameStartMillis())
            {
                last_frame_ts = radar.frameStartMillis();
                radar.dumpLastFrame();
            }
        }
        Serial.println("==Endwait.");
    }
    */

    Serial.println("Disabling config mode...");
    if (radar.disableConfigMode())
        Serial.println("Config mode disabled");
    else
        Serial.println("Disabling config mode failed");

    /*
    {
        Serial.println("Waiting for 1s.");
        unsigned long t = millis();
        while (millis() - t < 1000)
        {
            radar.read();
            if (last_frame_ts < radar.frameStartMillis())
            {
                last_frame_ts = radar.frameStartMillis();
                radar.dumpLastFrame();
            }
        }
        Serial.println("==Endwait.");

        //Serial.println("Try to get version while not in command mode...");
        //HLK_LD2410::FirmwareVersion vver = radar.reqFirmwareVersion();
        //Serial.printf("Firmware(t%x) Version %x.%02x.%08x\n", vver.type, vver.major, vver.minor, vver.bugFix);
    }

    Serial.println("Set Bluetooth OFF...");
    if (radar.setBluetooth(HLK_LD2410::BluetoothOff))
        Serial.println("Bluetooth turnde off");
    else
        Serial.println("Turning off Bluetooth failed");
    delay(200);
    Serial.println("Set Bluetooth ON...");
    if (radar.setBluetooth(HLK_LD2410::BluetoothOn))
        Serial.println("Bluetooth turnde on");
    else
        Serial.println("Turning on Bluetooth failed");
    delay(200);

    Serial.println("Set Baud rate...");
    if(radar.setBaudRate(HLK_LD2410::BAUD_256000))
        Serial.println("setBaudRate(256000) succsess");
    else
        Serial.println("setBaudRate(256000) failed");
    
    delay(200);

    Serial.println("Reset...");
    if (radar.reset())
        Serial.println("reset succsess");
    else
        Serial.println("reset failed");

    delay(200);

    Serial.println("restart...");
    if (radar.restart())
        Serial.println("restart succsess");
    else
        Serial.println("restart failed");

    delay(500);
    */
    Serial.println("start\n");
    
    //Serial.println("M, S, MTd, MTe, STd, Ste, dD");

    Serial.print("M, S, MdE0, MdE1, MdE2, MdE3, MdE4, MdE5, MdE6, MdE7, MdE8,");
    Serial.println(" SdE2, SdE3, SdE4, SdE5, SdE6, SdE7, SdE8");
}

void loop()
{
    radar.read();
    if (last_frame_ts < radar.frameStartMillis())
    {
        last_frame_ts = radar.frameStartMillis();
        radar._dumpLastFrame();

        //if (radar.radarMode() == HLK_LD2410::BASIC_DATA) 
        if (radar.frameType() == HLK_LD2410::BASIC_DATA)
        {
            Serial.printf("%3d, %3d, %3d, %3d, %3d, %3d, %3d, 1000\n",
              //Serial.printf("%3d, %3d, %3d, %3d, %3d, 1000\n",
                    radar.movingTarget() ? 10 : 0,
                radar.stationaryTarget() ? 10 : 0,
                radar.movingTargetDistance(),
                radar.movingTargetEnergy() * 10,
                radar.stationaryTargetDistance(),
                radar.stationaryTargetEnergy() * 10,
                radar.detectionDistance());
        }
        //else if (radar.radarMode() == HLK_LD2410::ENGINEERING_DATA) 
        else if (radar.frameType() == HLK_LD2410::ENGINEERING_DATA)
        {
            //Serial.printf("%3d, %3d, %3d, %3d, %3d, %3d, %3d, 1000\n",
            //    radar.movingTarget() ? 10 : 0,
            //    radar.stationaryTarget() ? 10 : 0,
            //    radar.movingTargetDistance(),
            //    radar.movingTargetEnergy() * 10,
            //    radar.stationaryTargetDistance(),
            //    radar.stationaryTargetEnergy() * 10,
            //    radar.detectionDistance());

            Serial.printf("%3d, %3d",
                radar.movingTarget() ? 90 : 50,
                radar.stationaryTarget() ? 40 : 0);
            int offset = 100;
            for (int i = 0; i < 9; i++, offset += 100)
                Serial.printf(", %3d", offset + radar.movementEnergyAtDistance(i));
            for (int i = 2; i < 9; i++, offset += 100)
                Serial.printf(", %3d", offset + radar.restingEnergyAtDistance(i));
            Serial.println(", 1700");
        }
    }
    yield();
}


/*

#define BUF_SIZE 80

// globas - to go i class
unsigned long lastReadFrame_ts = 0;
char rBuff[2][BUF_SIZE] = { {0},{0} };
char *rBuffLast = rBuff[1];
char *rBuffCurrent = rBuff[0];
int rBuffLastLen = 0;
int rBuffPos = 0;
bool wholeFrameReady = false;

// Reporting data frame format
#pragma pack (1)
struct TargetData
{
    uint16_t Distance;
    uint8_t Energy;
};

struct ld2410ReportingFrame // LD2410B uses little-endian format
{
    uint32_t frameHeader; // F4 F3 F2 F1 0xf1f2f3f4
    uint16_t intraFrameDataLength;
    uint8_t modeType; // 0x01 Enginnering data, 0x02 Target basic information
    uint8_t ifHead; // 0xAA

    uint8_t targetState; // 0x00 No target, 0x01 Moving, 0x02 Stationary, 0x03 Moving+Stationary target
    TargetData movingTarget;
    TargetData stationaryTarget;
    uint16_t detectionDistance;

    uint8_t ifTail; // 0x55
    uint8_t ifCheck; // 0x00
    uint32_t frameTrailer; // F8 F7 F6 F5 0xf5f6f7f8
};

struct ld2410EngineeringFrame 
{
    uint32_t frameHeader; // F4 F3 F2 F1 0xf1f2f3f4
    uint16_t intraFrameDataLength;
    uint8_t modeType; // 0x01 Enginnering data, 0x02 Target basic information
    uint8_t ifHead; // 0xAA

    uint8_t targetState; // 0x00 No target, 0x01 Moving, 0x02 Stationary, 0x03 Moving+Stationary target
    TargetData movingTarget;
    TargetData stationaryTarget;
    uint16_t detectionDistance;

    uint8_t maxMovingDistance;
    uint8_t maxStaticDistance;
    uint8_t movmentDistanceEnergyGate[9];
    uint8_t staticDistanceEnergyGate[9];
    uint8_t aditionlaInfo[2];

    uint8_t ifTail; // 0x55
    uint8_t ifCheck; // 0x00
    uint32_t frameTrailer; // F8 F7 F6 F5 0xf5f6f7f8
};

#pragma pack (0)

class HLK_LD2410
{
    union FrameMarkerType
    {
        uint32_t frameHeader;
        uint8_t fHeader[4];
    };

    struct TargetData
    {
        uint16_t Distance;
        uint8_t Energy;
    };

    struct ReportingFrame // LD2410B uses little-endian format
    {
        FrameMarkerType header; // F4 F3 F2 F1 0xf1f2f3f4
        uint16_t intraFrameDataLength;
        uint8_t modeType; // 0x01 Enginnering data, 0x02 Target basic information
        uint8_t ifHead; // 0xAA

        uint8_t targetState; // 0x00 No target, 0x01 Moving, 0x02 Stationary, 0x03 Moving+Stationary target
        TargetData movingTarget;
        TargetData stationaryTarget;
        uint16_t detectionDistance;

        uint8_t ifTail; // 0x55
        uint8_t ifCheck; // 0x00
        FrameMarkerType trailer; // F8 F7 F6 F5 0xf5f6f7f8
    };


public:
    const FrameMarkerType headerACK = { 0xFAFBFCFD };
    const FrameMarkerType trailerACK = { 0x01020304 };
    const FrameMarkerType headerDAT = { 0xF1F2F3F4 };
    const FrameMarkerType trailerDAT = { 0xF5F6F7F8 };
    
    RadarLD2410x()
    {
        rBuffLast = rBuff[1]; rBuffCurrent = rBuff[0]; rBuffLastLen = rBuffPos = 0; radar_uart = nullptr;
    }
    bool begin(Stream& rStream)
    {
        radar_uart = &rStream;
        return true;
    }
    unsigned long frameStartMillis() const { return lastFrame_ts;  }

    void read();
    void dumpLastFrame(Stream& s = Serial) const;
    int lastFrameType() const;

    uint8_t target() const { return (reinterpret_cast<const ReportingFrame*>(rBuffLast))->targetState; }
    bool movingTarget() const { return ((reinterpret_cast<const ReportingFrame*>(rBuffLast))->targetState) & 0b01; }
    bool stationaryTarget() const { return ((reinterpret_cast<const ReportingFrame*>(rBuffLast))->targetState) & 0b10; }
    uint16_t movingTargetDistance() const { return (reinterpret_cast<const ReportingFrame*>(rBuffLast))->movingTarget.Distance; }
    uint8_t movingTargetEnergy() const { return (reinterpret_cast<const ReportingFrame*>(rBuffLast))->movingTarget.Energy; }
    uint16_t stationaryTargetDistance() const { return (reinterpret_cast<const ReportingFrame*>(rBuffLast))->stationaryTarget.Distance; }
    uint8_t stationaryTargetEnergy() const { return (reinterpret_cast<const ReportingFrame*>(rBuffLast))->stationaryTarget.Energy; }
    uint16_t detectionDistance() const { return (reinterpret_cast<const ReportingFrame*>(rBuffLast))->detectionDistance; }

private:
    Stream* radar_uart;
    char rBuff[2][BUF_SIZE];
    char* rBuffLast;
    char* rBuffCurrent;
    int rBuffLastLen;
    int rBuffPos;
    //unsigned long currentFrame_ts;
    unsigned long lastFrame_ts;
};

void RadarLD2410x::read()
{
    if (rSerial.available())
    {
        char c = rSerial.read();
        if (c == 0xfd || c == 0xf4) // new frame begining
        {
            //swap reading buffers
            rBuffLast = rBuffCurrent;
            rBuffLastLen = rBuffPos;
            rBuffCurrent = rBuffLast;
            rBuffPos = 0;
            //lastFrame_ts = currentFrame_ts;
            //currentFrame_ts = millis();
            lastFrame_ts = millis();
            wholeFrameReady = true;
        }
        if (rBuffPos >= BUF_SIZE)
        {
            Serial.println("ERROR: Serial reading buffer overrun!");
            rBuffPos = 0;
        }
        rBuffCurrent[rBuffPos++] = c;
    }
}

void RadarLD2410x::dumpLastFrame(Stream & dumpStream) const 
{
    for (int i = 0; i < rBuffLastLen; i++)
        dumpStream.printf("%02x", rBuffLast[i]);
    dumpStream.println();

}

int RadarLD2410x::lastFrameType() const
{
    const FrameMarkerType* fheader = reinterpret_cast<const FrameMarkerType*>(rBuffLast);
    if(fheader->frameHeader == headerDAT.frameHeader)
    {
        return 1;
    }
    if (fheader->frameHeader == headerACK.frameHeader)
    {
        return 0;
    }
    return -1; // unknown
}



//////////////////////////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(115200); //Feedback over Serial Monitor
    rSerial.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN); //UART for monitoring the radar

    Serial.printf("sizeof(ld2410ReportingFrame)=%d\n", sizeof(ld2410ReportingFrame));
    Serial.printf("sizeof(ld2410EngineeringFrame)=%d\n", sizeof(ld2410EngineeringFrame));
    


    Serial.println("strat\n");
    Serial.println("M, S, MTd, MTe, STd, Ste, dD");


}

void loop()
{
    if (rSerial.available())
    {
        char c = rSerial.read();
        if (c == 0xfd || c == 0xf4) // new frame begining
        {
            //swap reading buffers
            rBuffLast = rBuffCurrent;
            rBuffLastLen = rBuffPos;
            rBuffCurrent = rBuffLast;
            rBuffPos = 0;
            wholeFrameReady = true;
        }
        if (rBuffPos >= BUF_SIZE)
        {
            Serial.println("ERROR: Serial reading buffer overrun!");
            rBuffPos = 0;
        }
        rBuffCurrent[rBuffPos++] = c;
    }
    if (wholeFrameReady)
    {
        
        for (int i = 0; i < rBuffLastLen; i++)
            Serial.printf("%02x", rBuffLast[i]);
        Serial.println();
        

        //ld2410ReportingFrame rData = *(reinterpret_cast<ld2410ReportingFrame*>(rBuffLast));
        const ld2410ReportingFrame * rData = reinterpret_cast<const ld2410ReportingFrame *>(rBuffLast);
        if (rData->modeType == 0x02)
        {
            Serial.printf("%3d, %3d, %3d, %3d, %3d, %3d, %3d, 1000\n",
            //Serial.printf("T:%c%c (%4x,%2x) (%4x,%2x) %d\n",
                //((rData->targetState & 0b01) ? 'M' : '.'),
                //((rData->targetState & 0b10) ? 'S' : '.'),
                ((rData->targetState & 0b01) ? 10 : 0),
                ((rData->targetState & 0b10) ? 10 : 0),
                rData->movingTarget.Distance,
                rData->movingTarget.Energy * 10,
                rData->stationaryTarget.Distance,
                rData->stationaryTarget.Energy * 10,
                rData->detectionDistance);
                
            //Serial.printf("%08x %02x ... %02x %02x %08x\n",
            //    rData->frameHeader,
            //    rData->ifHead,
            //    rData->ifTail,
            //    rData->ifCheck,
            //    rData->frameTrailer);
                
        }
        wholeFrameReady = false;
    }
    yield();
}
*/