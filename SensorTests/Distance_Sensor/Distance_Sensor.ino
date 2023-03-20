// Distance Sensor code without using the library.
#define pinNo 17

#define C1 1125.0
#define C2 137500.0

void setup() {
  // put your setup code here, to run once:
  pinMode(pinNo,INPUT);
  Serial.begin(9600);
}

#define BUF_SIZE 30
uint16_t buffer[BUF_SIZE] = {0};
int counter = 0;
void loop() {
  // put your main code here, to run repeatedly:
  uint16_t reading = analogRead(pinNo);
  for(int i = 1; i < BUF_SIZE; i++)
    buffer[i] = buffer[i-1];
  buffer[0] = reading;

  int avg = 0;
  for(int i = 0; i < BUF_SIZE; i++)
    avg += buffer[i];
  avg /= BUF_SIZE;

  double readingMillivolts = 1000.0 * avg*(5.0/1024.0);
  double distanceCentimetres = 1.0 / ( (readingMillivolts - C1) / C2 );
  distanceCentimetres += 50;
  delay(1);
  if(counter++ >= 1000)
  {
    counter = 0;
    Serial.println(distanceCentimetres);

  }
}