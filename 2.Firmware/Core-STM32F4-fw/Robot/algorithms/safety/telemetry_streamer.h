#ifndef TELEMETRY_STREAMER_H
#define TELEMETRY_STREAMER_H

#include <cstdint>

/**
 * @brief Real-Time Telemetry Streamer
 *
 * Collects and packages robot state data for real-time streaming to host PC.
 * Supports configurable channels, decimation, and binary/ASCII formats.
 *
 * === Data Channels ===
 *   CH_JOINT_POS       : 6x float — joint positions (deg)
 *   CH_JOINT_VEL       : 6x float — joint velocities (deg/s)
 *   CH_JOINT_ACC       : 6x float — joint accelerations (deg/s^2)
 *   CH_MOTOR_CURRENT   : 6x float — motor currents (A)
 *   CH_CART_POSE       : 6x float — Cartesian pose [x,y,z,a,b,c]
 *   CH_EXT_FORCE       : 6x float — external force/torque [Fx,Fy,Fz,Tx,Ty,Tz]
 *   CH_TEMPERATURE     : 6x float — estimated motor temperatures (°C)
 *   CH_SAFETY_STATUS   : 4x uint32 — safety level, violation, joint, count
 *   CH_STATE_MACHINE   : 2x uint32 — current state, previous state
 *   CH_TIMING          : 2x float — loop time (ms), jitter (ms)
 *
 * === Binary Frame Format ===
 *   [SYNC1][SYNC2][SEQ][TIMESTAMP_MS][CHANNEL_MASK][DATA...][CRC16]
 *   SYNC = 0xAA 0x55
 *   SEQ  = incrementing sequence number (uint8)
 *   TIMESTAMP = uint32 ms
 *   CHANNEL_MASK = uint16 bitmask of enabled channels
 *   DATA = concatenated float/uint32 arrays for enabled channels
 *   CRC16 = CRC-16-CCITT over entire frame
 *
 * === Decimation ===
 *   Control loop at 1kHz, but streaming at 10-100Hz is sufficient.
 *   Decimation factor N: send every N-th sample.
 */
class TelemetryStreamer
{
public:
    static constexpr int NUM_JOINTS = 6;
    static constexpr int MAX_FRAME_SIZE = 512;
    static constexpr uint8_t SYNC1 = 0xAA;
    static constexpr uint8_t SYNC2 = 0x55;

    enum Channel_t : uint16_t
    {
        CH_JOINT_POS      = (1 << 0),
        CH_JOINT_VEL      = (1 << 1),
        CH_JOINT_ACC      = (1 << 2),
        CH_MOTOR_CURRENT  = (1 << 3),
        CH_CART_POSE      = (1 << 4),
        CH_EXT_FORCE      = (1 << 5),
        CH_TEMPERATURE    = (1 << 6),
        CH_SAFETY_STATUS  = (1 << 7),
        CH_STATE_MACHINE  = (1 << 8),
        CH_TIMING         = (1 << 9),
        CH_ALL            = 0x03FF
    };

    struct Config_t
    {
        uint16_t enabledChannels;    // Bitmask of enabled channels
        uint16_t decimation;         // Send every N-th sample (1 = every sample)
        bool binaryMode;             // true = binary, false = ASCII CSV
    };

    struct TelemetryData_t
    {
        float jointPos[NUM_JOINTS];
        float jointVel[NUM_JOINTS];
        float jointAcc[NUM_JOINTS];
        float motorCurrent[NUM_JOINTS];
        float cartPose[6];           // [x,y,z,a,b,c]
        float extForce[6];           // [Fx,Fy,Fz,Tx,Ty,Tz]
        float temperature[NUM_JOINTS];
        uint32_t safetyLevel;
        uint32_t safetyViolation;
        int32_t safetyJoint;
        uint32_t safetyCount;
        uint32_t stateCurrent;
        uint32_t statePrevious;
        float loopTimeMs;
        float jitterMs;
        uint32_t timestampMs;
    };

    TelemetryStreamer();

    /**
     * @brief Initialize with default configuration
     */
    void Init(uint16_t _channels = CH_JOINT_POS | CH_JOINT_VEL | CH_EXT_FORCE,
              uint16_t _decimation = 10);

    /**
     * @brief Set enabled channels
     */
    void SetChannels(uint16_t _channelMask);

    /**
     * @brief Set decimation factor
     */
    void SetDecimation(uint16_t _decimation);

    /**
     * @brief Enable/disable binary mode (vs ASCII)
     */
    void SetBinaryMode(bool _binary);

    /**
     * @brief Update telemetry data (call every control cycle)
     * @return true if a frame is ready to send (after decimation)
     */
    bool Update(const TelemetryData_t &_data);

    /**
     * @brief Build binary frame into buffer
     * @param _buf    Output buffer (must be >= MAX_FRAME_SIZE)
     * @return Number of bytes written
     */
    int BuildBinaryFrame(uint8_t *_buf) const;

    /**
     * @brief Build ASCII CSV line into buffer
     * @param _buf    Output buffer (must be >= MAX_FRAME_SIZE)
     * @return Number of characters written
     */
    int BuildASCIIFrame(char *_buf) const;

    /**
     * @brief Get frame ready flag
     */
    bool IsFrameReady() const { return frameReady; }

    /**
     * @brief Get current data snapshot
     */
    const TelemetryData_t &GetData() const { return data; }

private:
    Config_t config;
    TelemetryData_t data;
    uint8_t seqNumber;
    uint16_t sampleCounter;
    bool frameReady;

    /**
     * @brief Append float array to binary buffer
     */
    static int AppendFloats(uint8_t *_buf, int _offset,
                              const float *_data, int _count);

    /**
     * @brief Append uint32 array to binary buffer
     */
    static int AppendUint32s(uint8_t *_buf, int _offset,
                               const uint32_t *_data, int _count);

    /**
     * @brief CRC-16-CCITT
     */
    static uint16_t ComputeCRC16(const uint8_t *_data, int _len);
};

#endif // TELEMETRY_STREAMER_H
