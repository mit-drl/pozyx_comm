#include <ros.h>
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <multi_car_msgs/CarControl.h>
#include <multi_car_msgs/GPS.h>
#include <multi_car_msgs/UWBRange.h>
#include <multi_car_msgs/ConsensusMsg.h>
#include <multi_car_msgs/PozyxDebug.h>
#include <common.h>

ros::NodeHandle  nh;
multi_car_msgs::UWBRange range;
multi_car_msgs::GPS gps;
multi_car_msgs::CarControl control;
multi_car_msgs::ConsensusMsg consensus;
multi_car_msgs::PozyxDebug debug_msg;

ros::Publisher pub_debug("/receiver/debug", &debug_msg);
ros::Publisher pub_range("ranges", &range);
ros::Publisher pub_gps("fixes", &gps);
ros::Publisher pub_control("controls", &control);
ros::Publisher pub_consensus("consensus", &consensus);

void setup_uwb()
{
    UWB_settings_t uwb_settings;
    Pozyx.getUWBSettings(&uwb_settings);
    uwb_settings.bitrate = 2;
    uwb_settings.plen = 0x08;
    Pozyx.setUWBSettings(&uwb_settings);
}

int this_car;
uint16_t source_id;                 // the network id of this device
const char *receivers[3] = {"6802","6806","6827"}; //every car id (Receiver, Sender)
const int num_cars = sizeof(receivers)/2;
const char *senders[num_cars] = {"6835","6867","685b"}; //every car id (Receiver, Sender)

using dq_consensus = dq_consensus_t<2, 3>;

void setup(){
    Serial.begin(57600);
    nh.initNode();
    nh.advertise(pub_debug);
    nh.advertise(pub_range);
    nh.advertise(pub_gps);
    nh.advertise(pub_control);
    nh.advertise(pub_consensus);

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

void parse_data(uint16_t sender_id, uint8_t *data, uint8_t length)
{

    uint8_t *cur = data;
    dq_header header;
    memcpy(&header, cur, sizeof(dq_header));
    cur += sizeof(dq_header);
    /* read_msg<dq_header>(&header, cur); */
    uint8_t num_meas = header.num_meas;

    sensor_type meas_types[num_meas];
    memcpy(meas_types, cur, num_meas * sizeof(sensor_type));
    cur += num_meas * sizeof(sensor_type);
    size_t exp_msg_size =
        sizeof(dq_header) +
        num_meas * sizeof(sensor_type);

    for (size_t i = 0; i < num_meas; i++)
    {
        if (meas_types[i] == RANGE)
        {
            exp_msg_size += sizeof(dq_range);
        }
        if (meas_types[i] == CONTROL)
        {
            exp_msg_size += sizeof(dq_control);
        }
        if (meas_types[i] == GPS)
        {
            exp_msg_size += sizeof(dq_gps);
        }
        if (meas_types[i] == CONSENSUS)
        {
            exp_msg_size += sizeof(dq_consensus);
        }
    }

    char buf[50];
    sprintf(buf, "act: %d, exp: %d, act_id: %d, exp_id: %d",
        length, exp_msg_size, sender_id, header.id);
    if (length != exp_msg_size or sender_id != header.id)
    {
        /* nh.loginfo(buf); */
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

        if (meas_types[i] == RANGE)
        {
            /* nh.loginfo("RANGE"); */
            dq_range rng;
            memcpy(&rng, cur, sizeof(dq_range));
            cur += sizeof(dq_range);

            if (rng.dist > 0)
            {
                range.distance = rng.dist / 1000.0;
                range.from_id = sender_id;
                range.to_id = rng.id;
                range.header.stamp = nh.now();
                pub_range.publish(&range);
            }
        }

        if (meas_types[i] == CONTROL)
        {
            /* nh.loginfo("CONTROL"); */
            if (sender_id != 26715)
            {
                char buf[50];
                sprintf(buf, "%d -- %d -- %d", sender_id, length,
                    meas_types[i]);
                /* nh.loginfo(buf); */
            }
            dq_control con;
            memcpy(&con, cur, sizeof(dq_control));
            cur += sizeof(dq_control);
            control.header.stamp = nh.now();
            control.car_id = sender_id;
            control.steering_angle = con.steering_angle;
            control.velocity = con.velocity;
            pub_control.publish(&control);
        }

        if (meas_types[i] == GPS)
        {
            dq_gps nmea;
            memcpy(&nmea, cur, sizeof(dq_gps));
            cur += sizeof(dq_gps);
            gps.header.stamp = nh.now();
            gps.car_id = sender_id;
            gps.fix.header.stamp = nh.now();
            //gps.fix.header.frame_id = "world";
            gps.fix.status.status = nmea.status;
            gps.fix.latitude = nmea.lat;
            gps.fix.longitude = nmea.lon;
            gps.fix.altitude = nmea.alt;
            pub_gps.publish(&gps);
        }

        if (meas_types[i] == CONSENSUS)
        {
            dq_consensus cons;
            memcpy(&cons, cur, sizeof(dq_consensus));
            cur += sizeof(dq_consensus);
            consensus.header.stamp = nh.now();
            consensus.confidences = cons.confidences;
            consensus.confidences_length = 2 * 3 * 2 * 3;
            consensus.states = cons.states;
            consensus.states_length = 2 * 3;
            consensus.car_id = cons.id;
            pub_consensus.publish(&consensus);
        }
    }
}

void publish_messages()
{
    if (Pozyx.waitForFlag(POZYX_INT_STATUS_RX_DATA, 50))
    {
        bool other_car = true;
        uint16_t sender_id = 0x00;
        uint8_t length = 0;
        delay(1);
        int res_last_id = Pozyx.getLastNetworkId(&sender_id);
        /* if (sender_id == senders[this_car]) { */
        /*   other_car = false; */
        /* } */

        int res_last_length = Pozyx.getLastDataLength(&length);
        if (length > 0 and res_last_id == POZYX_SUCCESS
            and res_last_length == POZYX_SUCCESS)
        {
            char buf[50];
            sprintf(buf, "%d", length);
            /* nh.loginfo(buf); */
            uint8_t data[length];
            Pozyx.readRXBufferData(data, length);
            parse_data(sender_id, data, length);
        }
    }
}
