#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <common.h>
#include <NMEAGPS.h>
#include <Time.h>

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
    gpsPort.begin(4800);
    
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
    //Serial.println(
    discover();
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
    size_t total_car_data = sizeof(dq_header) + num_cars * car_data_size + sizeof(dq_gps);
    uint8_t buffer[total_car_data];
    
    fix = gps.read();
    sensor_type sensor = RANGE;
    dq_header header = {1, {GPS,RANGE}};
    // writes message header
    
    for (int i = 0; i < num_cars; i++) {
        if (String(source_id,HEX) != String(car_ids[i],HEX)) {
            device_range_t range;
            status = 0;
            while (status != POZYX_SUCCESS) {
                status = Pozyx.doRanging(car_ids[i], &range);
                if (status == POZYX_SUCCESS) {
                    float dist = range.distance / 1000.0;
                    dq_range rng = {car_ids[i], dist};
                    memcpy(buffer + i * car_data_size + sizeof(dq_header),
                    &rng, sizeof(dq_range));
                    cars_ranged += 1;
                    break;
                }
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
        header = {GPS, num_cars};
        got_fix = true;
        memcpy(buffer + sizeof(dq_header) + cars_ranged * car_data_size, &nmea, sizeof(dq_gps));
    }
    
    memcpy(&buffer[0], &header, sizeof(dq_header));
    //Serial.println(header.sensor_type);
    
    if (got_fix) 
    {
        status = Pozyx.writeTXBufferData(buffer, total_car_data);
    }
    else 
    {
        status = Pozyx.writeTXBufferData(buffer, total_car_data - sizeof(dq_gps));
    }
    
    // broadcast the contents of the TX buffer
    status = Pozyx.sendTXBufferData(chat_id);
    delay(1);
}
