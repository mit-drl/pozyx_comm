#include <ros.h>
#include <ArduinoHardware.h>
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <multi_car_msgs/CarControl.h>
#include <multi_car_msgs/GPS.h>
#include <multi_car_msgs/UWBRange.h>
#include <multi_car_msgs/ConsensusMsg.h>
#include <multi_car_msgs/PozyxDebug.h>
#include <multi_car_msgs/LidarPose.h>
#include <common.h>

/* ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048> nh; */
ros::NodeHandle nh;
multi_car_msgs::UWBRange range_msg;
multi_car_msgs::GPS gps_msg;
multi_car_msgs::CarControl control_msg;
multi_car_msgs::ConsensusMsg consensus_msg;
multi_car_msgs::PozyxDebug debug_msg;
multi_car_msgs::LidarPose lidar_pose_msg;

const int num_dim = 3;
using dq_lidar_pose = dq_lidar_pose_t<num_dim>;

dq_range rng;
dq_control control;
dq_gps gps;
dq_lidar_pose lidar_pose;

ros::Publisher pub_debug("/debug/receiver", &debug_msg);
ros::Publisher pub_range("ranges", &range_msg);
ros::Publisher pub_gps("fixes", &gps_msg);
ros::Publisher pub_control("controls", &control_msg);
ros::Publisher pub_consensus("consensus", &consensus_msg);
ros::Publisher pub_lidar_pose("lidar_poses", &lidar_pose_msg);

void setup_uwb()
{
    UWB_settings_t uwb_settings;
    Pozyx.getUWBSettings(&uwb_settings);
    uwb_settings.bitrate = 0;
    uwb_settings.plen = 0x08;
    /* uwb_settings.channel = 1; */
    Pozyx.setUWBSettings(&uwb_settings);
}

int this_car;
uint16_t source_id;                 // the network id of this device
const char *receivers[3] = {"6802","6806","6827"}; //every car id (Receiver, Sender)
const int num_cars = sizeof(receivers)/2;
const char *senders[num_cars] = {"6835","6867","685b"}; //every car id (Receiver, Sender)

using dq_consensus = dq_consensus_t<2, 3>;

void setup(){
    Serial.begin(115200);
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub_debug);
    nh.advertise(pub_range);
    /* nh.advertise(pub_gps); */
    /* nh.advertise(pub_control); */
    /* nh.advertise(pub_consensus); */
    /* nh.advertise(pub_lidar_pose); */

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
        if (meas_types[i] == LIDAR_POSE)
        {
            exp_msg_size += sizeof(dq_lidar_pose);
        }
    }

    return exp_msg_size;
}

void parse_data(uint16_t sender_id, uint8_t *data, uint8_t length)
{

    uint8_t *cur = data;
    dq_header header;
    /* memcpy(&header, cur, sizeof(dq_header)); */
    /* cur += sizeof(dq_header); */
    /* read_msg<dq_header>(&header, cur); */
    read_msg<dq_header>(&header, cur);
    uint8_t num_meas = header.num_meas;

    sensor_type meas_types[num_meas];
    memcpy(meas_types, cur, num_meas * sizeof(sensor_type));
    cur += num_meas * sizeof(sensor_type);

    size_t exp_msg_size = get_exp_msg_size(meas_types, num_meas);

    char buf[50];
    sprintf(buf, "act: %d, exp: %d, act_id: %d, exp_id: %d",
        length, exp_msg_size, sender_id, header.id);
    if (length != exp_msg_size or sender_id != header.id)
    {
        /* nh.loginfo(buf); */
        /* nh.loginfo("not okay"); */
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

        if (meas_types[i] == CONTROL)
        {
            memcpy(&control, cur, sizeof(dq_control));
            cur += sizeof(dq_control);
            control_msg.header.stamp = nh.now();
            control_msg.car_id = sender_id;
            control_msg.steering_angle = control.steering_angle;
            control_msg.velocity = control.velocity;
            pub_control.publish(&control_msg);
        }

        if (meas_types[i] == GPS)
        {
            memcpy(&gps, cur, sizeof(dq_gps));
            cur += sizeof(dq_gps);
            gps_msg.header.stamp = nh.now();
            gps_msg.car_id = sender_id;
            gps_msg.fix.header.stamp = nh.now();
            //gps.fix.header.frame_id = "world";
            gps_msg.fix.status.status = gps.status;
            gps_msg.fix.latitude = gps.lat;
            gps_msg.fix.longitude = gps.lon;
            gps_msg.fix.altitude = gps.alt;
            pub_gps.publish(&gps_msg);
        }

        if (meas_types[i] == CONSENSUS)
        {
            dq_consensus cons;
            memcpy(&cons, cur, sizeof(dq_consensus));
            cur += sizeof(dq_consensus);
            consensus_msg.header.stamp = nh.now();
            consensus_msg.confidences = cons.confidences;
            consensus_msg.confidences_length = 2 * 3 * 2 * 3;
            consensus_msg.states = cons.states;
            consensus_msg.states_length = 2 * 3;
            consensus_msg.car_id = cons.id;
            pub_consensus.publish(&consensus_msg);
        }

        if (meas_types[i] == LIDAR_POSE)
        {
            dq_lidar_pose lp;
            read_msg<dq_lidar_pose>(&lp, cur);
            lidar_pose_msg.header.stamp = nh.now();
            lidar_pose_msg.x = lp.x;
            lidar_pose_msg.y = lp.y;
            lidar_pose_msg.theta = lp.theta;
            lidar_pose_msg.car_id = lp.id;
            lidar_pose_msg.cov = lp.cov;
            lidar_pose_msg.cov_length = num_dim * num_dim;
            pub_lidar_pose.publish(&lidar_pose_msg);
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
