#include "sbus.h"
#include <stdio.h>

// Reciever Test
// This is to read from the SBUS reciever (FrSky L9R)
// And print it to the monitor.
// Thanks to library Sbus https://github.com/bolderflight/sbus 



#include <SoftwareSerial.h>
// SBUS object on hardware serial port 1 
bfs::SbusRx l9r(1);
bfs::SbusData data;
SoftwareSerial mySerial (10, 11);
void setup() {
  Serial.begin(115200);
  // Serial.println("Spinning up...");
  while (!Serial) {}
  //begin SBUS Communication
  l9r.Begin();


}

void loop() {
  // Start the SBUS Communication
   Serial.begin(115200);
   // Get the Recieved data
  char buffer[data.NUM_CH];
  if(l9r.Read()){
    data = l9r.data();
    for (int8_t i = 0; i < data.NUM_CH; i++) {
      buffer[i] = data.ch[i];
      mySerial.print(data.ch[i]);        
      mySerial.print("\t");  
    }
    // End SBUS Communication
    
  }

}
