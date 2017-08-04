#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <multi_car_msgs/CarMeasurement.h>
#include <common.h>

ros::NodeHandle  nh;
/* sensor_msgs::Range range_msg; */
multi_car_msgs::CarMeasurement meas;
/* ros::Publisher pub_range( "range_data", &range_msg); */
ros::Publisher pub_meas("measurements", &meas);

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
  nh.advertise(pub_meas);
  // initialize Pozyx
  if(! Pozyx.begin(false, MODE_INTERRUPT, POZYX_INT_MASK_RX_DATA, 0)){
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    abort();
  }
  Pozyx.setOperationMode(POZYX_TAG_MODE);
  //setup_uwb();
  //Serial.flush();
}

void loop(){
  print_message();
}

void parse_data(uint8_t *data)
{
    dq_header header;
    memcpy(&header, data, sizeof(dq_header));

    // if range data
    if (header.sensor_type == 0)
    {
        dq_range rng;
        /* Serial.println(header.num_data); */
        for (int i = 0; i < header.num_data; i++)
        {
            memcpy(&rng, data + sizeof(dq_header) + i * sizeof(dq_range),
                sizeof(dq_range));
            /* meas.header.frame_id = rng.id; */
            /* String(rng.id, HEX).toCharArray(meas.header.frame_id, sizeof(uint16_t)); */
            if (rng.dist > 0)
            {
                meas.control.steering_angle = rng.dist;
                meas.header.stamp = nh.now();
                /* Serial.println(rng.id); */
                pub_meas.publish(&meas);
            }
        }
    }
}

void print_message() {
  if(Pozyx.waitForFlag(POZYX_INT_STATUS_RX_DATA,100)){ //wait up to 50ms for a message
    // we have received a message!
    // who sent the message?
    uint16_t messenger = 0x00;
    uint8_t length = 0;
    delay(1);
    Pozyx.getLastNetworkId(&messenger);
    Pozyx.getLastDataLength(&length);
    /* char data[length]; */
    String your_id = String(messenger,HEX); //sender id
    // read the contents of the receive (RX) buffer, this is the msesage that was sent to this device

    if (length > 0)
    {
        uint8_t data[length];
        Pozyx.readRXBufferData(data, length);
        /* Serial.println(length); */
        parse_data(data);
    }
    /* String my_id = String(data).substring(0,4); //sender id */
    /* String meters = String(data).substring(5); //distance */
    //Did we receive data, and is it reasonable?
  /*   if (sizeof(data) > 0 and meters.toFloat() < 100){ */
  /*     //What car are we? */
  /*     //Serial.println("here"); */
  /*     for (int a = 0;a < num_ids; a++) { */
  /*       if (my_id == car_ids[a]) { */
  /*         range_msg.header.frame_id = car_num[a]; */
  /*       } */
  /*       //what car are you? */
  /*       else if (your_id == car_ids[a]) { */
  /*         range_msg.field_of_view = String(car_num[a][3]).toFloat(); */
  /*       } */
  /*     } */
  /*     //Zero meters is a misread */
  /*     if (meters.toFloat() != 0) { */
  /*       range_msg.range = meters.toFloat(); */
  /*       pub_range.publish(&range_msg); */
  /*       meas.gps.latitude = meters.toFloat(); */
  /*       pub_meas.publish(&meas); */
  /*     } */
  /*   } */
  /* } */
  }
    nh.spinOnce();
}

