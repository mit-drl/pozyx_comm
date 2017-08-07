#include <ros.h>
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <common.h>
#include <NMEAGPS.h>
#include <Time.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <multi_car_msgs/CarControl.h>
#include <multi_car_msgs/ConsensusMsg.h>

//GPSBAUD 4800
#define gpsPort Serial1

NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

uint16_t source_id;                 // the network id of this device
uint16_t chat_id = 0;             //Broadcast the message
int status;
time_t last_time = 0;
const int offset = -5;  // Eastern Standard Time (USA)

uint8_t ranging_protocol = POZYX_RANGE_PROTOCOL_PRECISION; // ranging protocol of the Pozyx.
const int num_cars = 1; //Amount of other cars
uint16_t car_ids[num_cars] = {0x6806}; //Default is car0 sender so only range car1,car2 receivers

ros::NodeHandle nh;

sensor_msgs::NavSatFix gps_msg;
ros::Publisher pub_gps("fix", &gps_msg);

geometry_msgs::Twist control;
geometry_msgs::Pose odom;
multi_car_msgs::ConsensusMsg consensus;
bool new_control = false, new_consensus = false, new_odom = false;

void car_control_cb(const geometry_msgs::Twist msg)
{
    control = msg;
    new_control = true;
}

void car_odom_cb(const geometry_msgs::Pose &msg)
{
    odom = msg;
    new_odom = true;
}

void consensus_cb(const multi_car_msgs::ConsensusMsg &msg)
{
    consensus = msg;
    new_consensus = true;
}

ros::Subscriber<geometry_msgs::Twist>
car_control_sub("/cmd_vel_mux/input/teleop", &car_control_cb);

ros::Subscriber<geometry_msgs::Pose>
car_odom_sub("/pose", &car_odom_cb);

ros::Subscriber<multi_car_msgs::ConsensusMsg>
consensus_sub("consensus", &consensus_cb);


void setup_uwb()
{
    UWB_settings_t uwb_settings;
    Pozyx.getUWBSettings(&uwb_settings);
    uwb_settings.bitrate = 2;
    uwb_settings.plen = 0x24;
    Pozyx.setUWBSettings(&uwb_settings);
}

void setup()
{
    Serial.begin(57600);
    /* gpsPort.begin(9600); */
    gpsPort.begin(4800);
    nh.initNode();
    nh.advertise(pub_gps);
    nh.subscribe(car_control_sub);
    nh.subscribe(car_odom_sub);
    nh.subscribe(consensus_sub);

    // initialize Pozyx
    if(Pozyx.begin() == POZYX_FAILURE)
    {
        Serial.println("ERROR: Unable to connect to POZYX shield");
        Serial.println("Reset required");
        delay(100);
        abort();
    }

    Pozyx.regRead(POZYX_NETWORK_ID, (uint8_t*)&source_id, 2);

    if (String(source_id,HEX) == "6867") //This is car1 sender
    {
        car_ids[0] = 0x6802; //so only range car0 receiver
        /* car_ids[1] = 0x6827; //and car2 reciever */
    }
    else if (String(source_id,HEX) == "685b") //This is car2 sender
    {
         //car_ids[0] = 0x6802; //so only range car0
        car_ids[0] = 0x6806; //and car1
    }
}

void discover()
{
    Pozyx.clearDevices();
    int status = Pozyx.doDiscovery(POZYX_DISCOVERY_ANCHORS_ONLY);
    if (status == POZYX_SUCCESS)
    {
        uint8_t n_devs = 0;
        status = 0;
        status = Pozyx.getDeviceListSize(&n_devs);
        if (n_devs > 0)
        {
            uint16_t devices[n_devs];
            Pozyx.getDeviceIds(devices, n_devs);
            for (int i = 0; i < n_devs; i++)
            {
                //Serial.println(String(devices[i], HEX));
            }
        }
    }
    else
    {
    }
}

void loop()
{
    /* discover(); */
    send_message();
    nh.spinOnce();
}

void send_message()
{
    size_t max_cars = 20;
    size_t max_buffer_size =
        // number of msgs
        sizeof(uint8_t)
        // array of sensor msg types
        + max_cars * sizeof(sensor_type)
        // array of ranges
        + max_cars * sizeof(dq_range)
        // gps msg
        + sizeof(dq_gps)
        // control msg
        + sizeof(dq_control)
        // consensus msg
        + sizeof(dq_consensus);

    size_t max_msg_size =
        max_buffer_size
        // without header
        - sizeof(uint8_t)
        - max_cars * sizeof(sensor_type);

    uint8_t buffer[max_buffer_size];
    uint8_t msg[max_msg_size];
    sensor_type meas_types[max_cars * sizeof(sensor_type)];
    uint8_t *cur = msg;
    size_t msg_size = 0;
    uint8_t meas_counter = 0;
    fix = gps.read();

    for (int i = 0; i < num_cars; i++) {
        if (String(source_id,HEX) != String(car_ids[i],HEX)) {
            device_range_t range;
            status = Pozyx.doRanging(car_ids[i], &range);
            if (status == POZYX_SUCCESS and range.distance > 0) {
                dq_range rng = {car_ids[i], range.distance};
                memcpy(cur, &rng, sizeof(dq_range));
                cur += sizeof(dq_range);
                msg_size += sizeof(dq_range);
                meas_types[meas_counter++] = RANGE;
                break;
            }
        }
    }

    if (fix.valid.location)
    {
        unsigned char gps_status = fix.status - 2;
        float lat = fix.latitude();
        float lon = fix.longitude();
        float alt = fix.altitude();
        dq_gps nmea = {gps_status,lat,lon,alt};
        memcpy(cur, &nmea, sizeof(dq_gps));
        cur += sizeof(dq_gps);
        msg_size += sizeof(dq_gps);
        meas_types[meas_counter++] = GPS;
        gps_msg.header.stamp = nh.now();
        gps_msg.header.frame_id = "world";
        gps_msg.status.status = gps_status;
        gps_msg.latitude = lat;
        gps_msg.longitude = lon;
        gps_msg.altitude = alt;
        pub_gps.publish(&gps_msg);
    }

    if (new_control)
    {
        dq_control con = {
            control.angular.z,
            control.linear.x,
            odom.position.x,
            odom.position.y,
            odom.orientation.x,
            odom.orientation.y,
            odom.orientation.z,
            odom.orientation.w
        };
        memcpy(cur, &con, sizeof(dq_control));
        cur += sizeof(dq_control);
        msg_size += sizeof(dq_control);
        meas_types[meas_counter++] = CONTROL;
        new_control = false;
        new_odom = false;
    }

    if (new_consensus)
    {
    }

    //Serial.println(meas_counter);
    memcpy(buffer, &meas_counter, sizeof(uint8_t));
    memcpy(buffer + sizeof(uint8_t), meas_types,
        sizeof(sensor_type) * meas_counter);
    memcpy(buffer + sizeof(sensor_type) * meas_counter + sizeof(uint8_t),
        msg, msg_size);
    Pozyx.writeTXBufferData(buffer, sizeof(uint8_t) +
        sizeof(sensor_type) * meas_counter + msg_size);
    status = Pozyx.sendTXBufferData(chat_id);
    delay(1);
}
