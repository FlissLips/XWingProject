#include "sbus.h"

const bool ENABLE_LOGGING = true;

/** object to represent data recieved from receiver */
class ReceiverData {
public:
    int16_t
        _leftJoystickX,
        _leftJoystickY,
        _rightJoystickX,
        _rightJoystickY;
    ReceiverData() = default;
    ~ReceiverData() = default;
    void readFromSbusData(const bfs::SbusData& data);
    void printData() const;
};

void ReceiverData::readFromSbusData(const bfs::SbusData& data) {
    _leftJoystickY = data.ch[0];
    _leftJoystickX = data.ch[1];
    _rightJoystickY = data.ch[2];
    _rightJoystickX = data.ch[3];
    // TODO
}

void ReceiverData::printData() const {
    const int16_t min = 171, max = 1811;
    const int progressWidth = 16;
    const int interval = (max-min)/progressWidth + 1;
    const int numDashes = 15;
    for (int i = 0; i < numDashes; i++)
        Serial.print('-');
    Serial.println("\nReceiver Data:");
    Serial.print("Left Joystick (X): \t");
    int currentIndex = (this->_leftJoystickX - min) / interval;
    Serial.print("<");
    for (int i = 0; i < progressWidth; i++) {
        if (i != currentIndex)
            Serial.print("-");
        else
            Serial.print("0");
    }
    Serial.println(">");
    Serial.print("Right Joystick (X): \t");
    currentIndex = (this->_rightJoystickX - min) / interval;
    Serial.print("<");
    for (int i = 0; i < progressWidth; i++) {
        if (i != currentIndex)
            Serial.print("-");
        else
            Serial.print("0");
    }
    Serial.println(">");
    Serial.print("Left Joystick (Y): \t");
    currentIndex = (this->_leftJoystickY - min) / interval;
    Serial.print("<");
    for (int i = 0; i < progressWidth; i++) {
        if (i != currentIndex)
            Serial.print("-");
        else
            Serial.print("0");
    }
    Serial.println(">");
    Serial.print("Right Joystick (Y): \t");
    currentIndex = (this->_rightJoystickY - min) / interval;
    Serial.print("<");
    for (int i = 0; i < progressWidth; i++) {
        if (i != currentIndex)
            Serial.print("-");
        else
            Serial.print("0");
    }
    Serial.println(">");
}


class  
/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial3);
ReceiverData latestReceiverData;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();

}

void handleReceiverSbus() {
 if (sbus_rx.Read()) {
    /* Grab the received data */
    latestReceiverData.readFromSbusData(sbus_rx.data());
    latestReceiverData.printData();
    // /* Display the received data */
    // for (int8_t i = 0; i < data.NUM_CH; i++) {
    //   Serial.print(data.ch[i]);
    //   Serial.print("\t");
    // }
    // Serial.println();
  }
}

void loop () {
    handleReceiverSbus();
}
