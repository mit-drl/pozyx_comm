
typedef struct dq_range_t {
    uint16_t id;
    float dist;
} dq_range;

typedef struct dq_header_t {
    uint8_t sensor_type;
    uint16_t num_data;
} dq_header;
