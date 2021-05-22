// THIS IS SLAVE_CTRL : FINGER CONTROLLER
// It controls : Finger motor & Wrist Servo
// It reads : 3x FSR from finger
// It communicates : Bidirectionnaly with Maser over i2c

#include <Arduino.h>
// Library for I2C communication
#include <Wire.h>
// Library for Servo montor control
#include <Servo.h>
// Library to control RGB led
#include <Adafruit_NeoPixel.h>

// Initialize pins
#define PrxFSR A2  // Proximal phalange Force Sensor 12bit
#define MidFSR A1  // Middle phalange Force Sensor 12bit
#define DisFSR A0  // Distal phalange Force Sensor 12bit

#define MotCW 6  // CW Finger motor rotation
#define MotCCW 7  // CCW Finger motor rotation
#define WristSERVO 8  // Wirst servo motor for PWM

#define ARR_SIZE(a) (sizeof(a) / sizeof(a[0]))

// Initialize objects
Servo WristServo;

// create a pixel strand with 1 pixel on PIN_NEOPIXEL
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

// Initialize variables
const byte MSaddress_EMG = 0x70;
const byte SLaddress_Ctrl = 0x71;
const byte SLaddress_Fdbck = 0x72;
int DataFromMaster[] = { 0, 0 };  // DataFromMaster[0] = 1=FLX / 2=EXT / 0=NON - DataFromMaster[1] = 1=ABD / 2=ADD / 0=NON
const int FSRpins[] = {PrxFSR, MidFSR, DisFSR};
int FSRVal[] = {0, 0, 0};
int FSRerror[] = {0, 0, 0};
int ErrorItr = 5; // Window size for error average
int ErrorMrgn = 100; // Arror margin to avoid negative values
const int ServoMin = 100;
const int ServoMax = 150;
const int ServoStep = 1;
int ServoVal = 125;

// Declare functions
void receiveFromMaster(int howMany);
void requestToMaster();
void WristControl(int cmd);
void FingerControl(int cmd);

void setup() {
  pixels.begin();  // initialize the pixel
  pixels.setPixelColor(0, pixels.Color(0, 36, 4));
  pixels.show();

  analogReadResolution(12);
  pinMode(MotCW, OUTPUT);
  pinMode(MotCCW, OUTPUT);
  digitalWrite(MotCW, LOW);
  digitalWrite(MotCCW, LOW);
  WristServo.attach(WristSERVO);

  // FSR callibration with mean of first "ErrorItr" values
  for (int i = 0; i < ARR_SIZE(FSRpins); i++) {
    for (int j = 0; j < ErrorItr; j++) {
      FSRerror[i] += analogRead(FSRpins[i]);
      delay(2);
    }
    FSRerror[i] = (FSRerror[i] / ErrorItr) + ErrorMrgn;
  }

  Wire.begin(SLaddress_Ctrl);
  Wire.onReceive(receiveFromMaster);
  Wire.onRequest(requestToMaster);
  Serial.begin(115200);
}

void loop() {
  // - Update FSR values
  for (int i = 0; i < ARR_SIZE(FSRpins); i++) {
    FSRVal[i] = analogRead(FSRpins[i]) - FSRerror[i];
  }
  // - Update Finger motor
  FingerControl(DataFromMaster[0]);
  // - Update Wrist Servo
  WristControl(DataFromMaster[1]);
  delay(10);
}

void FingerControl(int cmd) {
  // cmd = 1=FLX / 2=EXT / 0=NON
  if (cmd == 1) {
    digitalWrite(MotCW, HIGH);
    digitalWrite(MotCCW, LOW);
  }
  else if (cmd == 2) {
    digitalWrite(MotCW, LOW);
    digitalWrite(MotCCW, HIGH);
  }
  else if (cmd == 0) {
    digitalWrite(MotCW, LOW);
    digitalWrite(MotCCW, LOW);
  }
  else {
    digitalWrite(MotCW, LOW);
    digitalWrite(MotCCW, LOW);
  }
}

void WristControl(int cmd) {
  // cmd = 1=ABD / 2=ADD / 0=NON
  if (cmd == 1) {
    ServoVal += ServoStep;
  }
  else if (cmd == 2) {
    ServoVal -= ServoStep;
  }
  else { // Do nothing }
    if ( ServoVal < ServoMin ) {
      ServoVal = ServoMin;
    }
    if ( ServoVal > ServoMax ) {
      ServoVal = ServoMax;
    }
    WristServo.write(ServoVal);
  }
}

void requestToMaster()
{
  // Send 3x FSR analog values
  int tmpFSRVal[ARR_SIZE(FSRVal)];
  for (int i = 0; i < ARR_SIZE(FSRVal); i++) {
    tmpFSRVal[i] = map(FSRVal[i], 0, 4095, 0, 255);
  }
  Wire.write(tmpFSRVal, ARR_SIZE(tmpFSRVal));
}

// function that executes whenever data is received from master
void receiveFromMaster(int howMany) {
  // DataFromMaster[0] = 1=FLX / 2=EXT / 0=NON - DataFromMaster[1] = 1=ABD / 2=ADD / 0=NON
  int i = 0;
  while (Wire.available()) {
    DataFromMaster[i] = Wire.read();
    i++;
  }
  Serial.println("Recieved data :");
  for (int j = 0; j < ARR_SIZE(DataFromMaster); j++) {
    Serial.print(DataFromMaster[j]);
    Serial.print(" ");
  }
  Serial.println("");
}