/**
  The pozyx chat demo
  please check out https://www.pozyx.io/Documentation/Tutorials/getting_started

  This demo requires at least two pozyx shields and an equal number of Arduino's.
  It demonstrates the wireless messaging capabilities of the pozyx device.

  This demo creates a chat room. Text written in the Serial monitor will be broadcasted to all other pozyx devices
  within range. They will see your message appear in their Serial monitor.
*/

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

uint16_t source_id;                 // the network id of this device
uint16_t chat_id = 0;             //Broadcast the message
String distance = "";            // a string to hold incoming data

uint8_t ranging_protocol = POZYX_RANGE_PROTOCOL_PRECISION; // ranging protocol of the Pozyx.
const int num_cars = 2; //Amount of other cars
uint16_t car_ids[num_cars] = {0x6806,0x6827}; //Default is car0 sender so only range car1,car2 receivers

void setup(){
  Serial.begin(57600);
  // initialize Pozyx
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    delay(100);
    abort();
  }
  // read the network id of this device
  Pozyx.regRead(POZYX_NETWORK_ID, (uint8_t*)&source_id, 2);
  if (String(source_id,HEX) == "6867"){ //This is car1 sender 
    car_ids[0] = 0x6802; //so only range car0 receiver
    car_ids[1] = 0x6827; //and car2 reciever
  }
  else if (String(source_id,HEX) == "685b"){ //This is car2 sender
    car_ids[0] = 0x6802; //so only range car0
    car_ids[1] = 0x6806; //and car1
  }
}

void loop(){
  send_message();
}

void send_message() {
  for (int i = 0;i < num_cars;i++) { 
    if (String(source_id,HEX) != String(car_ids[i],HEX)) {
      device_range_t range;
      int status = 0;
      while (status != POZYX_SUCCESS){
        status = Pozyx.doRanging(car_ids[i], &range);
        if (status == POZYX_SUCCESS) {
          distance = String(car_ids[i], HEX) + "," + String(range.distance/1000.0,3);
          int length = String(distance).length()+1;
          uint8_t buffer[length];
          String(distance).getBytes(buffer, length);   
          // write the message to the transmit (TX) buffer
          status = Pozyx.writeTXBufferData(buffer, length);
          // broadcast the contents of the TX buffer
          status = Pozyx.sendTXBufferData(chat_id);
          break;
        }
      }
    }
  }
}


