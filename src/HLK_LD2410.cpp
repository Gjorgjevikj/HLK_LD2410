#include "HLK_LD2410.h"

// This library is made for little endian systems only!
#ifdef TEST_LITTLE_ENDIAN

HLK_LD2410::HLK_LD2410() : radarUART(nullptr), frameTimeOut(120), interCommandDelay(0)
{
    init();
}

void HLK_LD2410::init()
{
    inConfigMode = inFrame = frameReady = false;
    receivedFrameLen = currentFrameIndex = 0;
    bufferLastFrame = receiveBuffer[1];
    bufferCurrentFrame = receiveBuffer[0];
}

bool HLK_LD2410::begin(Stream& rStream)
{
    radarUART = &rStream;
    return true;
}

// reads incoming bytes on the serial and stores them in buffer
int HLK_LD2410::read()
{ 
    int rc = 0; // number of bytes read in this call
    while (radarUART->available())
    {
        uint8_t c = radarUART->read();
        rc++;
        if (!inFrame)
        {
            if (c == ACK_FRAME_HEAD_BYTE || c == DAT_FRAME_HEAD_BYTE) // new frame begining (c>>4 == 0x0f)
            {
                inFrame = true;
                frameReady = false;
                //swap reading buffers
                bufferLastFrame = bufferCurrentFrame;
                receivedFrameLen = currentFrameIndex;
                lastFrameStartTS = currentFrameStartTS;
                bufferCurrentFrame = bufferLastFrame;
                currentFrameIndex = 0;
                currentFrameStartTS = millis();
            }
        }
        else
        {
            if (currentFrameIndex > 6) // the frame type and length can be determined 
            {
                if (frameType(CURRENT_FRAME) == UNIDENTIFIED_FRAME) 
                {
                    inFrame = frameReady = false;
                    currentFrameIndex = 0;
                    R_LOG_WARN("Sync!\n");
                }
                else if(currentFrameIndex >= (reinterpret_cast<const FrameStart*>(bufferCurrentFrame))->ifDataLength+7)
                {
                    frameReady = true; // the rest are trailer bytes, so the frame can be processed 
                }
            }
            if (currentFrameIndex >= RECEIVE_BUFFER_SIZE)
            {
                R_LOG_ERROR("ERROR: Receive buffer overrun!\n");
                currentFrameIndex = 0;
            }
        }
        bufferCurrentFrame[currentFrameIndex++] = c;
        if (currentFrameIndex >= (reinterpret_cast<const FrameStart*>(bufferCurrentFrame))->ifDataLength + 10)
        {
            inFrame = false;
            //return rc;
        }
    }
    return rc;
}

void HLK_LD2410::write(const uint8_t* data, int size) //send raw data
{
    static unsigned long lastCommandIssuedAt = 0UL;
    DEBUG_DUMP_FRAME(data, size, ">>[","]");
    if (interCommandDelay > 0)
    {
        unsigned long sinceLast = millis() - lastCommandIssuedAt;
        if(sinceLast < interCommandDelay)
            delay(interCommandDelay - sinceLast);
    }
    radarUART->write(data, size);
    radarUART->flush();
}

// check the type of the frame
// frame=true -> last completed frame, frame=false -> current frame
HLK_LD2410::FrameType HLK_LD2410::frameType(bool frame) const
{
    if (frame || currentFrameIndex > 6) // the basic type can be determined by the first byte only
    {
        const FrameStart* fheadid = reinterpret_cast<const FrameStart*>(frame ? bufferLastFrame : bufferCurrentFrame);
        switch (fheadid->header.frameWord)
        {
        case DAT_FRAME_HEADER:
            return static_cast<FrameType>(fheadid->modeType);
            break;
        case ACK_FRAME_HEADER:
            return ACK_FRAME;
        default:
            return UNKNOWN_FRAME;
        }
    }
    return UNIDENTIFIED_FRAME; // not yet identified
}

bool HLK_LD2410::readAckFrame() 
{
    unsigned long now = millis();

    // wait for something to arrive / frame to start
    while (!read() && millis() - now < frameTimeOut)
        delay(1);
        //yield();
    if (millis() - now >= frameTimeOut)
    {
        R_LOG_WARN("# TimeOut while waiting for reply - no response\n");
        return false;
    }

    // waiting for ACK frame, skip the other (if any) frames that were in the buffer
    now = millis();
    while (frameType(CURRENT_FRAME) != ACK_FRAME && millis() - now < frameTimeOut)
    {
        //while (!frameReady && millis() - now < frameTimeOut)
        {
            if (!read())
                delayMicroseconds(50);
        }
    }
    if (millis() - now >= frameTimeOut)
    {
        DEBUG_DUMP_FRAME(bufferCurrentFrame, currentFrameIndex, "##[", "]");
        R_LOG_WARN("# TimeOut while waiting for ACK frame\n");
        //yield();
        return false;
    }
    if (frameType(CURRENT_FRAME) == ACK_FRAME)
    {
        int expLen = (reinterpret_cast<const FrameStart*>(bufferCurrentFrame))->ifDataLength + 10;
        now = millis();
        while (currentFrameIndex < expLen && millis() - now < frameTimeOut)
        {
            if (!read())
                delayMicroseconds(50);
        }
        DEBUG_DUMP_FRAME(bufferCurrentFrame, currentFrameIndex, "==[", "]");
        if (millis() - now >= frameTimeOut)
        {
            R_LOG_ERROR("# TimeOut reading ACK frame\n");
            //yield();
            return false;
        }
    }
    else
    {
        R_LOG_ERROR("# TimeOut while waiting for ACK frame\n");
        return false;
    }
    return true;
}

bool HLK_LD2410::enableConfigMode() 
{
    // when entering config mode (from reporting mode) several reporting frames meight be already in the buffer
    // a longer timeout is needed to ignore them
    // or ... another apprach would be to clean up the incoming buffer of butes waiting to be read before issuing the command.
    while (radarUART->available())
        radarUART->read();
    const static REQframeCommandWithValue cmdEnableConfMode = { CMD_FRAME_HEADER, 0x0004, ENABLE_CONFIG_MODE, 0x0001, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmdEnableConfMode), sizeof(REQframeCommandWithValue));
    //unsigned long prevTO = frameTimeOut;
    //frameTimeOut = 250;
    //readAckFrame();
    inConfigMode = commandAccknowledged(ENABLE_CONFIG_MODE, readAckFrame(), true);
    //frameTimeOut = prevTO;
    return inConfigMode;
}

bool HLK_LD2410::disableConfigMode() 
{
    const static REQframeCommand cmdEndConfMode = { CMD_FRAME_HEADER, 0x0002, DISABLE_CONFIG_MODE, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t *>(&cmdEndConfMode), sizeof(REQframeCommand));
    //readAckFrame();
    inConfigMode = !commandAccknowledged(DISABLE_CONFIG_MODE, readAckFrame(), true);
    return !inConfigMode;
}

HLK_LD2410::FirmwareVersion HLK_LD2410::reqFirmwareVersion()
{
    bool wasInConfigMode = inConfigMode;
    FirmwareVersion ver; // = { 0, 0, 0, 0 };
    if (!inConfigMode)
        enableConfigMode();
    const static REQframeCommand cmd = { CMD_FRAME_HEADER, 0x0002, READ_FIRMWARE_VERSION, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmd), sizeof(REQframeCommand));
    bool ackValid = readAckFrame();
    const ACKframeFirmwareVersion* ack = reinterpret_cast<const ACKframeFirmwareVersion*>(bufferCurrentFrame);
    ver = ack->version;
    return commandAccknowledged(READ_FIRMWARE_VERSION, ackValid, wasInConfigMode) ? ver : ((struct FirmwareVersion) { 0, 0, 0, 0 });
}

HLK_LD2410::RadarParameters HLK_LD2410::reqParameters()
{
    bool wasInConfigMode = inConfigMode;
    RadarParameters par;
    if (!inConfigMode)
        enableConfigMode();
    const static REQframeCommand cmd = { CMD_FRAME_HEADER, 0x0002, READ_PARAMETER, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmd), sizeof(REQframeCommand));
    bool ackValid=readAckFrame();
    const ACKframeParameter* ack = reinterpret_cast<const ACKframeParameter*>(bufferCurrentFrame);
    par = ack->param;
    return commandAccknowledged(READ_PARAMETER, ackValid, wasInConfigMode) ? par : ((struct RadarParameters) { 0, 0, 0 });
}

HLK_LD2410::MACaddress HLK_LD2410::reqMacAddress()
{
    bool wasInConfigMode = inConfigMode;
    MACaddress mac = { 0, 0, 0, 0, 0, 0 };
    if (!inConfigMode)
        enableConfigMode();
    const static REQframeCommandWithValue cmdReadMacAaddress = { CMD_FRAME_HEADER, 0x0004, READ_MAC_ADDRESS, 0x0001, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmdReadMacAaddress), sizeof(REQframeCommandWithValue));
    bool ackValid = readAckFrame();
    const ACKframeMacAddress* ack = reinterpret_cast<const ACKframeMacAddress*>(bufferCurrentFrame);
    mac = ack->macAddress;
    return commandAccknowledged(READ_MAC_ADDRESS, ackValid, wasInConfigMode) ? mac : ((struct MACaddress) { 0, 0, 0, 0, 0, 0 });
}

bool HLK_LD2410::setParameters(RadarCommand cmd, uint32_t distanceGatePar0, uint32_t distanceGatePar1, uint32_t distanceGatePar2)
{
    bool wasInConfigMode = inConfigMode;
    if (!inConfigMode)
        enableConfigMode();
    REQframeSetParameters cmdFrame = {
        CMD_FRAME_HEADER,
        0x0014, // (20)
        cmd,
        0x0000,
        distanceGatePar0,
        0x0001,
        distanceGatePar1,
        0x0002,
        distanceGatePar2,
        CMD_FRAME_TRAILER
    };

    write(reinterpret_cast<const uint8_t*>(&cmdFrame), sizeof(REQframeSetParameters));
    //readAckFrame();
    return commandAccknowledged(cmd, readAckFrame(),wasInConfigMode);
}

HLK_LD2410::DistanceResolution HLK_LD2410::reqDistanceResolution()
{
    bool wasInConfigMode = inConfigMode;
    if (!inConfigMode)
        enableConfigMode();
    const static REQframeCommand cmd = { CMD_FRAME_HEADER, 0x0002, READ_DISTANCE_RESOLUTION, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmd), sizeof(REQframeCommand));
    bool ackValid = readAckFrame();
    const ACKframeDistanceResolution* ack = reinterpret_cast<const ACKframeDistanceResolution*>(bufferCurrentFrame);
    DistanceResolution deRet = ack->distanceResolution;
    return commandAccknowledged(READ_DISTANCE_RESOLUTION, ackValid , wasInConfigMode) ? deRet : DR_ERROR;
}

bool HLK_LD2410::sendCommand(RadarCommand cmd)
{
    bool wasInConfigMode = inConfigMode;
    if (!inConfigMode)
        enableConfigMode();
    REQframeCommand cmdFrame = { CMD_FRAME_HEADER, 0x0002, cmd, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmdFrame), sizeof(REQframeCommand));
    bool ackValid = readAckFrame();
    // do not try to return from config mode after restart 
    // the radar will restart in reporting mode immidiately after sending ACK 
    if (cmd == RESTART && ackValid)
    {
        init();
        return true;
    }
        //wasInConfigMode = inConfigMode = false; // ?? restoring all other class variables to initial vlaues 
    return commandAccknowledged(cmd, ackValid, wasInConfigMode);
}

bool HLK_LD2410::sendCommand(RadarCommand cmd, uint16_t value)
{
    bool wasInConfigMode = inConfigMode;
    if (!inConfigMode)
        enableConfigMode();
    REQframeCommandWithValue cmdFrame = { CMD_FRAME_HEADER, 0x0004, cmd, value, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmdFrame), sizeof(REQframeCommandWithValue));
    //readAckFrame();
    return commandAccknowledged(cmd, readAckFrame(), wasInConfigMode);
}

bool HLK_LD2410::commandAccknowledged(RadarCommand cmd, bool AckFrameReceived, bool remainInConfig)
{
    const ACKframeCommand* ack = reinterpret_cast<const ACKframeCommand*>(bufferCurrentFrame);
    DEBUG_DUMP_FRAME(bufferCurrentFrame, ack->ifDataLength + 10, "<<[", "]");
    bool validAck = (AckFrameReceived
        && ack->header.frameWord == ACK_FRAME_HEADER)
        && (ack->commandReply == (cmd | 0x0100))
        && (ack->ackStatus == 0);
    if (!remainInConfig)
    {
        delay(100);
        disableConfigMode();
    }
    return validAck;
}

void HLK_LD2410::_dumpFrame(const uint8_t* buff, int len, String pre, String post, Stream& dumpStream)
{
    dumpStream.print(pre);
    for (int i = 0; i < len; i++)
        dumpStream.printf("%02x", buff[i]);
    dumpStream.println(post);
}

#endif
