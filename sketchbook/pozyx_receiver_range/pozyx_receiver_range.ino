#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <ros.h>
/* #include <sensor_msgs/Range.h> */
#include <multi_car_msgs/GPS.h>
#include <multi_car_msgs/CarMeasurement.h>
#include <multi_car_msgs/UWBRange.h>
#include <common.h>

ros::NodeHandle  nh;
/* sensor_msgs::Range range_msg; */
multi_car_msgs::CarMeasurement meas;
multi_car_msgs::UWBRange range;
multi_car_msgs::GPS gps;

ros::Publisher pub_meas("measurements", &meas);
ros::Publisher pub_range("ranges", &range);
ros::Publisher pub_gps("fixes", &gps);

const char *car_ids[6] = {"6802","6835","6806","6867","6827","685b"}; //every car id (Receiver, Sender)
const int num_ids = sizeof(car_ids)/2; //amount of total ids
const char *car_num[num_ids] = {"car0","car0","car1","car1","car2","car2"}; //assign car number to each id
int repeat[] = {0,1};
String my_id;

void setup_uwb()
{
    UWB_settings_t uwb_settings;
    Pozyx.getUWBSettings(&uwb_settings);
    uwb_settings.bitrate = 2;
    uwb_settings.plen = 0x24;
    Pozyx.setUWBSettings(&uwb_settings);
}

void setup(){
    Serial.begin(57600);
    nh.initNode();
    /* nh.advertise(pub_meas); */
    nh.advertise(pub_range);
    nh.advertise(pub_gps);
    // initialize Pozyx
    if(!Pozyx.begin(false, MODE_INTERRUPT, POZYX_INT_MASK_RX_DATA, 0)){
        Serial.println("ERROR: Unable to connect to POZYX shield");
        Serial.println("Reset required");
        abort();
    }
    //setup_uwb();
}

void loop(){
    publish_messages();
}

void parse_data(uint16_t sender_id, uint8_t *data)
{
    uint8_t *cur = data;
    uint8_t num_meas;
    memcpy(&num_meas, cur, sizeof(uint8_t));
    cur += sizeof(uint8_t);

    sensor_type meas_types[num_meas];
    memcpy(meas_types, cur, num_meas * sizeof(sensor_type));
    cur += num_meas * sizeof(sensor_type);

    for (size_t i = 0; i < num_meas; i++)
    {
        switch (meas_types[i])
        {
            case RANGE:
                dq_range rng;
                memcpy(&rng, cur, sizeof(dq_range));
                cur += sizeof(dq_range);

                if (rng.dist > 0)
                {
                    range.distance = rng.dist;
                    range.from_id = sender_id;
                    range.to_id = rng.id;
                    range.header.stamp = nh.now();
                    pub_range.publish(&range);
                }
                break;
            case GPS:
                dq_gps nmea;
                memcpy(&nmea, cur, sizeof(dq_gps));
                cur += sizeof(dq_gps);
                gps.header.stamp = nh.now();
                gps.car_id = sender_id;
                gps.fix.header.stamp = nh.now();
                gps.fix.header.frame_id = "world";
                gps.fix.status.status = nmea.status;
                gps.fix.latitude = nmea.lat;
                gps.fix.longitude = nmea.lon;
                gps.fix.altitude = nmea.alt;
                pub_gps.publish(&gps);
                break;
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
        Pozyx.getLastNetworkId(&sender_id);
        Pozyx.getLastDataLength(&length);

        if (length > 0)
        {
            uint8_t data[length];
            Pozyx.readRXBufferData(data, length);
            parse_data(sender_id, data);
        }
    }

    nh.spinOnce();
}

