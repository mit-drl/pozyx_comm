#include <ros.h>
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <common.h>
#include <Adafruit_GPS.h>
#include <AltSoftSerial.h>
#include <multi_car_msgs/GPS.h>
#include <multi_car_msgs/PozyxDebug.h>
#include <multi_car_msgs/UWBRange.h>

AltSoftSerial mySerial;
Adafruit_GPS gpsPort(&mySerial);

uint16_t source_id;
uint16_t chat_id = 0;
int status;
const int offset = -5;  // Eastern Standard Time (USA)
uint8_t ranging_protocol = POZYX_RANGE_PROTOCOL_PRECISION;
const int num_cars = 2;
const int num_dim = 3;
uint16_t car_ids[num_cars] = {0x6806,0x6827};
unsigned char gps_frame_id[6] = "earth";

ros::NodeHandle nh;

multi_car_msgs::GPS gps_msg;
ros::Publisher pub_gps("fix", &gps_msg);

multi_car_msgs::PozyxDebug debug_msg;
ros::Publisher pub_debug("/debug/sender", &debug_msg);

multi_car_msgs::UWBRange range_msg;
ros::Publisher pub_range("ranges", &range_msg);

void setup()
{
    Serial.begin(115200);
    gpsPort.begin(9600);

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub_gps);
    nh.advertise(pub_debug);
    nh.advertise(pub_range);

    gpsPort.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    gpsPort.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    gpsPort.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
    //gpsPort.sendCommand(PGCMD_ANTENNA);
    gpsPort.sendCommand(PMTK_SET_BAUD_57600);
    delay(1000);
    gpsPort.begin(57600);
    delay(1500);

    // initialize Pozyx
    if (Pozyx.begin() == POZYX_FAILURE)
    {
        Serial.println("ERROR: Unable to connect to POZYX shield");
        Serial.println("Reset required");
        delay(100);
        abort();
    }

    Pozyx.regRead(POZYX_NETWORK_ID, (uint8_t*)&source_id, 2);

    if (String(source_id, HEX) == "6867") //This is car1 sender
    {
        car_ids[0] = 0x6802; //so only range car0 receiver
        car_ids[1] = 0x6827; //and car2 reciever
    }
    else if (String(source_id, HEX) == "685b") //This is car2 sender
    {
        car_ids[0] = 0x6802; //so only range car0
        car_ids[1] = 0x6806; //and car1
    }

    setup_uwb();
}


void loop()
{
    send_message();
    nh.spinOnce();
}

void send_message()
{
    size_t max_cars = 20;
    size_t max_buffer_size =
        // header
        sizeof(dq_header)
        // array of sensor msg types
        + max_cars * sizeof(sensor_type)
        // array of ranges
        + max_cars * sizeof(dq_range);

    size_t max_msg_size =
        max_buffer_size
        // without header
        - sizeof(dq_header)
        - max_cars * sizeof(sensor_type);

    uint8_t buffer[max_buffer_size];
    uint8_t msg[max_msg_size];
    sensor_type meas_types[max_cars * sizeof(sensor_type)];
    uint8_t *cur = msg;
    size_t msg_size = 0;
    uint8_t meas_counter = 0;

    for (int i = 0; i < num_cars; i++) {
        if (String(source_id, HEX) != String(car_ids[i], HEX)) {
            device_range_t range;
            status = Pozyx.doRanging(car_ids[i], &range);
            if (status == POZYX_SUCCESS and range.distance > 0) {
                dq_range rng = {car_ids[i], range.distance};
                msg_size += write_msg<dq_range>(cur, &rng);
                meas_types[meas_counter++] = RANGE;
                range_msg.distance = rng.dist / 1000.0;
                range_msg.from_id = source_id;
                range_msg.to_id = car_ids[i];
                range_msg.header.stamp = nh.now();
                pub_range.publish(&range_msg);
            }
        }
    }

    /* while (!gpsPort.newNMEAreceived()) */
    /* { */
    /*     char c = gpsPort.read(); */
    /* } */

    if (gpsPort.parse(gpsPort.lastNMEA()))
    {       // this also sets the newNMEAreceived() flag to false
        unsigned char gps_status = gpsPort.fixquality;
        float lat = gpsPort.latitudeDegrees;
        float lon = gpsPort.longitudeDegrees;
        float alt = 0.0;
        gps_msg.fix.header.stamp = nh.now();
        gps_msg.fix.header.frame_id = gps_frame_id;
        gps_msg.fix.status.status = gps_status;
        gps_msg.fix.latitude = lat;
        gps_msg.fix.longitude = lon;
        gps_msg.fix.altitude = alt;
        gps_msg.car_id = source_id;
        gps_msg.header.stamp = nh.now();
        pub_gps.publish(&gps_msg);
    }

    dq_header header = {meas_counter, source_id};
    memcpy(buffer, &header, sizeof(dq_header));
    memcpy(buffer + sizeof(dq_header), meas_types,
                 sizeof(sensor_type) * meas_counter);
    memcpy(buffer + sizeof(dq_header) + sizeof(sensor_type) * meas_counter,
                 msg, msg_size);

    debug_msg.header.stamp = nh.now();
    debug_msg.sender_id = source_id;
    debug_msg.num_meas = meas_counter;
    debug_msg.meas_types = (uint8_t *) meas_types;
    debug_msg.meas_types_length = meas_counter;
    pub_debug.publish(&debug_msg);

    if (meas_counter > 0)
    {
        for (size_t i = 0; i < num_cars; i++)
        {
            status = Pozyx.sendData(car_ids[i], buffer, sizeof(dq_header) +
                sizeof(sensor_type) * meas_counter + msg_size);
        }
    }

    delay(1);
}
