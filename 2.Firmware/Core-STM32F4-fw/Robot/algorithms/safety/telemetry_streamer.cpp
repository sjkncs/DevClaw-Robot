#include "telemetry_streamer.h"
#include <cstring>
#include <cstdio>


TelemetryStreamer::TelemetryStreamer()
{
    memset(&config, 0, sizeof(Config_t));
    memset(&data, 0, sizeof(TelemetryData_t));
    seqNumber = 0;
    sampleCounter = 0;
    frameReady = false;
    config.decimation = 10;
    config.binaryMode = true;
}


void TelemetryStreamer::Init(uint16_t _channels, uint16_t _decimation)
{
    config.enabledChannels = _channels;
    config.decimation = (_decimation > 0) ? _decimation : 1;
    config.binaryMode = true;
    seqNumber = 0;
    sampleCounter = 0;
    frameReady = false;
}


void TelemetryStreamer::SetChannels(uint16_t _channelMask)
{
    config.enabledChannels = _channelMask;
}


void TelemetryStreamer::SetDecimation(uint16_t _decimation)
{
    config.decimation = (_decimation > 0) ? _decimation : 1;
}


void TelemetryStreamer::SetBinaryMode(bool _binary)
{
    config.binaryMode = _binary;
}


bool TelemetryStreamer::Update(const TelemetryData_t &_data)
{
    memcpy(&data, &_data, sizeof(TelemetryData_t));
    sampleCounter++;

    if (sampleCounter >= config.decimation)
    {
        sampleCounter = 0;
        frameReady = true;
        return true;
    }

    frameReady = false;
    return false;
}


int TelemetryStreamer::AppendFloats(uint8_t *_buf, int _offset,
                                      const float *_data, int _count)
{
    memcpy(_buf + _offset, _data, _count * sizeof(float));
    return _offset + _count * (int)sizeof(float);
}


int TelemetryStreamer::AppendUint32s(uint8_t *_buf, int _offset,
                                       const uint32_t *_data, int _count)
{
    memcpy(_buf + _offset, _data, _count * sizeof(uint32_t));
    return _offset + _count * (int)sizeof(uint32_t);
}


uint16_t TelemetryStreamer::ComputeCRC16(const uint8_t *_data, int _len)
{
    // CRC-16-CCITT (polynomial 0x1021, init 0xFFFF)
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < _len; i++)
    {
        crc ^= (uint16_t)_data[i] << 8;
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}


int TelemetryStreamer::BuildBinaryFrame(uint8_t *_buf) const
{
    if (!frameReady) return 0;

    int offset = 0;

    // Header
    _buf[offset++] = SYNC1;
    _buf[offset++] = SYNC2;
    _buf[offset++] = seqNumber;

    // Timestamp
    memcpy(_buf + offset, &data.timestampMs, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    // Channel mask
    memcpy(_buf + offset, &config.enabledChannels, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    uint16_t ch = config.enabledChannels;

    // Payload
    if (ch & CH_JOINT_POS)
        offset = AppendFloats(_buf, offset, data.jointPos, NUM_JOINTS);
    if (ch & CH_JOINT_VEL)
        offset = AppendFloats(_buf, offset, data.jointVel, NUM_JOINTS);
    if (ch & CH_JOINT_ACC)
        offset = AppendFloats(_buf, offset, data.jointAcc, NUM_JOINTS);
    if (ch & CH_MOTOR_CURRENT)
        offset = AppendFloats(_buf, offset, data.motorCurrent, NUM_JOINTS);
    if (ch & CH_CART_POSE)
        offset = AppendFloats(_buf, offset, data.cartPose, 6);
    if (ch & CH_EXT_FORCE)
        offset = AppendFloats(_buf, offset, data.extForce, 6);
    if (ch & CH_TEMPERATURE)
        offset = AppendFloats(_buf, offset, data.temperature, NUM_JOINTS);
    if (ch & CH_SAFETY_STATUS)
    {
        uint32_t safety[4] = {data.safetyLevel, data.safetyViolation,
                               (uint32_t)data.safetyJoint, data.safetyCount};
        offset = AppendUint32s(_buf, offset, safety, 4);
    }
    if (ch & CH_STATE_MACHINE)
    {
        uint32_t sm[2] = {data.stateCurrent, data.statePrevious};
        offset = AppendUint32s(_buf, offset, sm, 2);
    }
    if (ch & CH_TIMING)
    {
        float timing[2] = {data.loopTimeMs, data.jitterMs};
        offset = AppendFloats(_buf, offset, timing, 2);
    }

    // CRC over everything before CRC
    uint16_t crc = ComputeCRC16(_buf, offset);
    memcpy(_buf + offset, &crc, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    // Increment sequence (cast away const for this mutable counter)
    const_cast<TelemetryStreamer*>(this)->seqNumber++;

    return offset;
}


int TelemetryStreamer::BuildASCIIFrame(char *_buf) const
{
    if (!frameReady) return 0;

    int len = 0;
    uint16_t ch = config.enabledChannels;

    len += snprintf(_buf + len, MAX_FRAME_SIZE - len, "%lu", (unsigned long)data.timestampMs);

    if (ch & CH_JOINT_POS)
        for (int i = 0; i < NUM_JOINTS; i++)
            len += snprintf(_buf + len, MAX_FRAME_SIZE - len, ",%.2f", (double)data.jointPos[i]);

    if (ch & CH_JOINT_VEL)
        for (int i = 0; i < NUM_JOINTS; i++)
            len += snprintf(_buf + len, MAX_FRAME_SIZE - len, ",%.2f", (double)data.jointVel[i]);

    if (ch & CH_EXT_FORCE)
        for (int i = 0; i < 6; i++)
            len += snprintf(_buf + len, MAX_FRAME_SIZE - len, ",%.3f", (double)data.extForce[i]);

    if (ch & CH_SAFETY_STATUS)
        len += snprintf(_buf + len, MAX_FRAME_SIZE - len, ",%lu,%lu",
                        (unsigned long)data.safetyLevel, (unsigned long)data.safetyViolation);

    if (ch & CH_TIMING)
        len += snprintf(_buf + len, MAX_FRAME_SIZE - len, ",%.2f", (double)data.loopTimeMs);

    len += snprintf(_buf + len, MAX_FRAME_SIZE - len, "\n");

    const_cast<TelemetryStreamer*>(this)->seqNumber++;

    return len;
}
