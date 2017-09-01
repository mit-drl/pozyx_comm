
#include <Pozyx.h>
#include <Pozyx_definitions.h>

typedef enum sensor_type_t {
	RANGE, GPS, CONTROL, CONSENSUS, LIDAR_POSE
} sensor_type;

typedef struct dq_range_t {
    uint16_t id;
    uint32_t dist;
} dq_range;

typedef struct dq_control_t {
    float steering_angle;
    float velocity;
    // float x;
    // float y;
    // float qx;
    // float qy;
    // float qz;
    // float qw;
} dq_control;

typedef struct dq_header_t {
    uint8_t num_meas;
    uint16_t id;
} dq_header;

typedef struct dq_gps_t {
    uint8_t status;
    float lat;
    float lon;
    float alt;
} dq_gps;

template <int NUM_DIM>
struct dq_lidar_pose_t {
    float x;
    float y;
    float theta;
    uint8_t id;
    float cov[NUM_DIM * NUM_DIM];
};

typedef struct dq_lidar_pose_without_cov_t {
    float x;
    float y;
    float theta;
    uint8_t id;
} dq_lidar_pose_without_cov;

template <int NUM_CARS, int NUM_DIM>
struct dq_consensus_t {
    uint16_t id;
    float confidences[NUM_CARS * NUM_DIM * NUM_CARS * NUM_DIM];
    float states[NUM_CARS * NUM_DIM];
};

template <typename type>
size_t read_msg(type *msg, uint8_t *&cur)
{
    memcpy(msg, cur, sizeof(type));
    cur += sizeof(type);
    return sizeof(type);
}

template <typename type>
size_t write_msg(uint8_t *&cur, type *msg)
{
    memcpy(cur, msg, sizeof(type));
    cur += sizeof(type);
    return sizeof(type);
}

void setup_uwb()
{
    UWB_settings_t uwb_settings;
    Pozyx.getUWBSettings(&uwb_settings);
    uwb_settings.bitrate = 2;
    uwb_settings.plen = 0x08;
    /* uwb_settings.channel = 1; */
    Pozyx.setUWBSettings(&uwb_settings);
}
