#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <common.h>
#include <NMEAGPS.h>
#include <Time.h>
#include <sensor_msgs/NavSatFix.h>

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
/* const int num_cars = 2; //Amount of other cars */
const int num_cars = 1; //Amount of other cars
/* uint16_t car_ids[num_cars] = {0x6806,0x6827}; //Default is car0 sender so only range car1,car2 receivers */
uint16_t car_ids[num_cars] = {0x6806}; //Default is car0 sender so only range car1,car2 receivers

ros::NodeHandle nh;

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
    gpsPort.begin(9600);

    // initialize Pozyx
    if(Pozyx.begin() == POZYX_FAILURE)
    {
        Serial.println("ERROR: Unable to connect to POZYX shield");
        Serial.println("Reset required");
        delay(100);
        abort();
    }

    /* delay(1000); */
    //setup_uwb();
    // read the network id of this device
    /* Pozyx.setOperationMode(POZYX_ANCHOR_MODE); */
    delay(1000);
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
    if (gps.available( gpsPort )) {
        send_message();
    }
}

void send_message()
{
    bool got_fix = false;
    int car_data_size = sizeof(dq_range);
    int cars_ranged = 0;
    size_t max_cars = 20;
    size_t max_msg_size = sizeof(uint8_t) + max_cars * sizeof(sensor_type)
        + max_cars * sizeof(dq_range) + sizeof(dq_gps) + sizeof(dq_control);
    size_t total_car_data = sizeof(dq_header) + num_cars * car_data_size + sizeof(dq_gps);
    uint8_t buffer[max_msg_size];
    uint8_t msg[max_msg_size - sizeof(uint8_t) - max_cars * sizeof(sensor_type)];
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
                cars_ranged += 1;
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
        got_fix = true;
        memcpy(cur, &nmea, sizeof(dq_gps));
        cur += sizeof(dq_gps);
        msg_size += sizeof(dq_gps);
        meas_types[meas_counter++] = GPS;
    }
    //Serial.println(meas_counter);
    memcpy(buffer, &meas_counter, sizeof(uint8_t));
    memcpy(buffer + sizeof(uint8_t), meas_types,
        sizeof(sensor_type) * meas_counter);
    memcpy(buffer + sizeof(sensor_type) * meas_counter + sizeof(uint8_t),
        msg, msg_size);
    Pozyx.writeTXBufferData(buffer, sizeof(uint8_t) + sizeof(sensor_type) * meas_counter + msg_size);
    status = Pozyx.sendTXBufferData(chat_id);
    delay(1);
}
