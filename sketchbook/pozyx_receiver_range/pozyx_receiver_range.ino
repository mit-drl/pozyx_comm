#include <ros.h>
#include <ArduinoHardware.h>
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <multi_car_msgs/UWBRange.h>
#include <multi_car_msgs/PozyxDebug.h>
#include <common.h>

ros::NodeHandle nh;
dq_range rng;
multi_car_msgs::UWBRange range_msg;
multi_car_msgs::PozyxDebug debug_msg;

const int num_dim = 3;

ros::Publisher pub_debug("/debug/receiver", &debug_msg);
ros::Publisher pub_range("ranges", &range_msg);

int this_car;
uint16_t source_id;
const char *receivers[3] = {"6802","6806","6827"};
const int num_cars = sizeof(receivers)/2;
const char *senders[num_cars] = {"6835","6867","685b"};

void setup(){
    Serial.begin(115200);
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub_debug);
    nh.advertise(pub_range);

    // initialize Pozyx
    if(!Pozyx.begin(false, MODE_INTERRUPT, POZYX_INT_MASK_RX_DATA, 0)){
        Serial.println("ERROR: Unable to connect to POZYX shield");
        Serial.println("Reset required");
        abort();
    }
    Pozyx.regRead(POZYX_NETWORK_ID, (uint8_t*)&source_id, 2);
    for (int i = 0;i < num_cars;i++) {
      if (String(source_id, HEX) == receivers[i]) {
        this_car = i;
        break;
      }
    }
    setup_uwb();
}

void loop(){
    publish_messages();
    nh.spinOnce();
}

size_t get_exp_msg_size(sensor_type *meas_types, uint8_t num_meas)
{
    size_t exp_msg_size =
        sizeof(dq_header) +
        num_meas * sizeof(sensor_type);

    for (size_t i = 0; i < num_meas; i++)
    {
        exp_msg_size += sizeof(dq_range);
    }

    return exp_msg_size;
}

void parse_data(uint16_t sender_id, uint8_t *data, uint8_t length)
{

    uint8_t *cur = data;
    dq_header header;
    read_msg<dq_header>(&header, cur);
    uint8_t num_meas = header.num_meas;

    sensor_type meas_types[num_meas];
    memcpy(meas_types, cur, num_meas * sizeof(sensor_type));
    cur += num_meas * sizeof(sensor_type);

    size_t exp_msg_size = get_exp_msg_size(meas_types, num_meas);
    if (length != exp_msg_size or sender_id != header.id)
    {
        return;
    }

    debug_msg.header.stamp = nh.now();
    debug_msg.sender_id = sender_id;
    debug_msg.receiver_id = source_id;
    debug_msg.num_meas = num_meas;
    debug_msg.meas_types = (uint8_t *) meas_types;
    debug_msg.meas_types_length = num_meas;
    pub_debug.publish(&debug_msg);

    for (size_t i = 0; i < num_meas; i++)
    {
        memcpy(&rng, cur, sizeof(dq_range));
        cur += sizeof(dq_range);

        if (rng.dist > 0)
        {
            range_msg.distance = rng.dist / 1000.0;
            range_msg.from_id = sender_id;
            range_msg.to_id = rng.id;
            range_msg.header.stamp = nh.now();
            pub_range.publish(&range_msg);
        }
    }
}

void publish_messages()
{
    if (Pozyx.waitForFlag(POZYX_INT_STATUS_RX_DATA, 100))
    {
        uint16_t sender_id = 0x00;
        uint8_t length = 0;
        delay(1);
        int res_last_id = Pozyx.getLastNetworkId(&sender_id);
        int res_last_length = Pozyx.getLastDataLength(&length);
        if (length > 0 and res_last_id == POZYX_SUCCESS
            and res_last_length == POZYX_SUCCESS)
        {
            uint8_t data[length];
            Pozyx.readRXBufferData(data, length);
            parse_data(sender_id, data, length);
        }
    }
}
