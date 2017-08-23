
typedef enum sensor_type_t {
	RANGE, GPS, CONTROL, CONSENSUS
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
    uint8_t num_measurements;
    sensor_type *measurements_type; // of length num_measurements
} dq_header;

typedef struct dq_gps_t {
    uint8_t status;
    float lat;
    float lon;
    float alt;
} dq_gps;

template <int NUM_CARS, int NUM_DIM>
struct dq_consensus_t {
    uint16_t id;
    float confidences[NUM_CARS * NUM_DIM * NUM_CARS * NUM_DIM];
    float states[NUM_CARS * NUM_DIM];
};
