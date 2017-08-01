#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);

const char *car_ids[6] = {"6802","6835","6806","6867","6827","685b"}; //every car id (Receiver, Sender)
const int num_ids = sizeof(car_ids)/2; //amount of total ids
const char *car_num[num_ids] = {"car0","car0","car1","car1","car2","car2"}; //assign car number to each id
int repeat[] = {0,1};
String my_id;

  
void setup(){
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(pub_range);
  // initialize Pozyx
  if(! Pozyx.begin(false, MODE_INTERRUPT, POZYX_INT_MASK_RX_DATA, 0)){
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    abort();
  }
}

void loop(){
  print_message();
}

void print_message() { 
  if(Pozyx.waitForFlag(POZYX_INT_STATUS_RX_DATA,100)){ //wait up to 50ms for a message
    // we have received a message!
    // who sent the message?
    uint16_t messenger = 0x00;
    delay(1);
    Pozyx.getLastNetworkId(&messenger);
    char data[11];
    String your_id = String(messenger,HEX); //sender id
    // read the contents of the receive (RX) buffer, this is the msesage that was sent to this device
    Pozyx.readRXBufferData((uint8_t *) data, 11); 
    String my_id = String(data).substring(0,4); //sender id
    String meters = String(data).substring(5); //distance
    //Did we receive data, and is it reasonable?
    if (sizeof(data) > 0 and meters.toFloat() < 100){ 
      //What car are we?
      for (int a = 0;a < num_ids; a++) {
        if (my_id == car_ids[a]) { 
          range_msg.header.frame_id = car_num[a];
        }
        //what car are you? 
        else if (your_id == car_ids[a]) {
          range_msg.field_of_view = String(car_num[a][3]).toFloat();
        }
      }
      //Zero meters is a misread
      if (meters.toFloat() != 0) { 
        range_msg.range = meters.toFloat();
        pub_range.publish(&range_msg);
        nh.spinOnce();
      }
    }
  }
}

