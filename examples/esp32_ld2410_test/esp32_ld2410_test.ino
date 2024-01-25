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

#define PLOT_ADPT

#include <HLK_LD2410.h>

HLK_LD2410 radar;
unsigned int last_frame_ts = 0;
const char* dr[] = { "0.75m", "0.2m", "???" };

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

    Serial.println("Reading firmware version...");
    HLK_LD2410::FirmwareVersion ver = radar.reqFirmwareVersion();
    Serial.printf("FirmwareVersion %x.%02x.%08x (type%x)\n", ver.major, ver.minor, ver.bugfix, ver.type);
    delay(200);

    Serial.println("Turning off Bluetooth...");
    if (radar.setBluetooth(HLK_LD2410::BluetoothOff))
        Serial.println("Bluetoot turned off.");
    else
        Serial.println("Turning off Bluetooth failed.");
    delay(200);

    Serial.println("Reading distance resolution...");
    HLK_LD2410::DistanceResolution d = radar.reqDistanceResolution();
    Serial.printf("DistanceResolution : %04x (%s)\n", d, (d < 2 ? dr[d] : dr[2]));
    delay(200);

    Serial.println("Setting distance resolution to 0.2m");
    if (radar.setDistanceResolution(HLK_LD2410::DR_0_2))
        Serial.println("Distance resolution set to 0.2");
    else
        Serial.println("Setting distance resolution failed");
    delay(200);

    Serial.println("Reading distance resolution...");
    HLK_LD2410::DistanceResolution d2 = radar.reqDistanceResolution();
    Serial.printf("DistanceResolution now : %04x (%s)\n", d2, (d2 < 2 ? dr[d2] : dr[d2]));
    delay(200);

    Serial.println("Setting distance resolution back");
    if (radar.setDistanceResolution(d))
        Serial.println("Distance resolution set back to previous value");
    else
        Serial.println("Setting distance resolution failed");
    delay(200);

    Serial.println("Reading distance resolution...");
    d = radar.reqDistanceResolution();
    Serial.printf("DistanceResolution now : %04x (%s)\n", d, (d < 2 ? dr[d] : dr[2]));
    delay(200);
    
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
    ver = radar.reqFirmwareVersion();
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

    delay(200);
    Serial.println("Turning on Bluetooth...");
    if (radar.setBluetooth(HLK_LD2410::BluetoothOn))
        Serial.println("Bluetoot turned on.");
    else
        Serial.println("Turning Bluetooth on failed.");

    delay(200);
    HLK_LD2410::MACaddress maca = radar.reqMacAddress();
    Serial.print("MAC address: ");
    char macStr[20];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
        maca.mac[0], maca.mac[1], maca.mac[2], maca.mac[3], maca.mac[4], maca.mac[5]);
    Serial.println(macStr);

    delay(500);
    Serial.println("Restoring radar factory defaults...");
    if (radar.reset())
        Serial.println("Reseted to factory deafuls (effective after restart)");
    else
        Serial.println("Reset failed");

    delay(500);
    Serial.println("Setting distance resolution to 0.2m");
    if (radar.setDistanceResolution(HLK_LD2410::DR_0_2))
        Serial.println("Distance resolution set to 0.2");
    else
        Serial.println("Setting distance resolution failed");
    delay(200);
    
    Serial.println("Restarting radar...");
    if (!radar.restart())
        Serial.println("Restart failed");

    delay(1500);
    Serial.println("Reading distance resolution...");
    d2 = radar.reqDistanceResolution();
    Serial.printf("DistanceResolution now : %04x (%s)\n", d2, (d2 < 2 ? dr[d2] : dr[d2]));
    delay(200);

    Serial.println("Enabling Engineering mode...");
    if (radar.enableEngineeringMode())
        Serial.println("Enabling EngineeringMode sucess");
    else
        Serial.println("Enabling EngineeringMode failed");

    Serial.println("start\n");
    
#ifdef PLOT_ADPT
    //Serial.print("\nM, S, MtD, MtE, StD, StE");
    Serial.print("\nM, S");
    for (int i = 0; i < 9; i++)
        Serial.printf(", MdE%d", i);
    for (int i = 2; i < 9; i++)
        Serial.printf(", SdE%d", i);
#else
    Serial.print("\nMS, MtD, MtE, StD, StE, G, g");
    for (int i = 0; i < 9; i++)
        Serial.printf(", Me%d", i);
    for (int i = 2; i < 9; i++)
        Serial.printf(", Se%d", i);
    Serial.print(", SE0, SE1");
#endif
    Serial.println();

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
#ifdef PLOT_ADPT
            if (radar.target() > 3)
                radar._dumpCurrentFrame();
            Serial.printf("%3d, %3d",
                radar.movingTarget() ? 90 : 50,
                radar.stationaryTarget() ? 40 : 0);
            int offset = 100;
            for (int i = 0; i < 9; i++, offset += 100)
                Serial.printf(", %3d", offset + radar.movementEnergyAtDistance(i));
            for (int i = 2; i < 9; i++, offset += 100)
                Serial.printf(", %3d", offset + radar.restingEnergyAtDistance(i));
            Serial.println(", 1700");
#else
            Serial.printf("%2d, %3d, %3d, %3d, %3d, %d, %d",
                radar.target() & 0x0f,
                radar.movingTargetDistance(),
                radar.movingTargetEnergy(),
                radar.stationaryTargetDistance(),
                radar.stationaryTargetEnergy(),
                radar.maxMovingDistanceGates(), // ??
                radar.maxStaticDistanceGates()  // ?? 
            );
            for (int i = 0; i < 9; i++)
                Serial.printf(", %3d", radar.movementEnergyAtDistance(i));
            for (int i = 2; i < 9; i++)
                Serial.printf(", %3d", radar.restingEnergyAtDistance(i));
            Serial.printf(", %3d, %3d", radar.restingEnergyAtDistance(0), radar.restingEnergyAtDistance(1));
            Serial.println();
#endif
        }
    }
    //yield();
    delay(55+random(20));

}


