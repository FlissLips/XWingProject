#include "sbus.h"

// Reciever Test
// This is to read from the SBUS reciever (FrSky L9R)
// And print it to the monitor.
// Thanks to library Sbus https://github.com/bolderflight/sbus 



// SBUS object on hardware serial port 1 
bfs::SbusRx l9r(1);
bfs::SbusData data;
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  //begin SBUS Communication
  l9r.Begin();

}

void loop() {
   // Get the Recieved data
  if(l9r.Read()){
    data = l9r.data();
    for (int8_t i = 0; i < data.NUM_CH; i++) {
      Serial.print(data.ch[i]);
      Serial.print("\t");
    }

  }

}
