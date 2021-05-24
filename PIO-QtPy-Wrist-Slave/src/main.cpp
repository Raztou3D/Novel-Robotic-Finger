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
int ErrorItr = 10; // Window size for error average
int ErrorMrgn = 100; // Arror margin to avoid negative values
const int ServoMin = 30;
const int ServoMax = 150;
const int ServoStep = 1;
int ServoVal = 90;

// Declare functions
void receiveFromMaster(int howMany);
void requestToMaster();
void WristControl(int cmd);
void FingerControl(int cmd);

void setup() {
  pixels.begin();  // initialize the pixel
  pixels.setPixelColor(0, pixels.Color(0, 10, 0));
  pixels.show();

  analogReadResolution(12);
  pinMode(MotCW, OUTPUT);
  pinMode(MotCCW, OUTPUT);
  digitalWrite(MotCW, LOW);
  digitalWrite(MotCCW, LOW);

  WristServo.attach(WristSERVO);

  // FSR callibration with mean of first "ErrorItr" values
  for (unsigned int i = 0; i < ARR_SIZE(FSRpins); i++) {
    for (int j = 0; j < ErrorItr; j++) {
      FSRerror[i] += analogRead(FSRpins[i]);
      delay(2);
    }
    FSRerror[i] = (FSRerror[i] / ErrorItr) - ErrorMrgn;
  }

  Wire.begin(SLaddress_Ctrl);
  Wire.onReceive(receiveFromMaster);
  Wire.onRequest(requestToMaster);
  Serial.begin(115200);
}

void loop() {
  pixels.begin();  // initialize the pixel
  pixels.setPixelColor(0, pixels.Color(0, 10, 0));
  pixels.show();

  // Serial.print("FSRError : ");
  // for (unsigned int i = 0; i < ARR_SIZE(FSRpins); i++) {
  //   Serial.print(FSRerror[i]);
  //   Serial.print(" ");
  // }
  // Serial.println("");

  // - Update FSR values
  for (unsigned int i = 0; i < ARR_SIZE(FSRpins); i++) {
    FSRVal[i] = analogRead(FSRpins[i]) - FSRerror[i];
  }

  // Serial.print("FSRVal : ");
  // for (unsigned int i = 0; i < ARR_SIZE(FSRpins); i++) {
  //   Serial.print(FSRVal[i]);
  //   Serial.print(" ");
  // }
  // Serial.println("");


  // - Update Finger motor
  FingerControl(DataFromMaster[0]);
  // - Update Wrist Servo
  WristControl(DataFromMaster[1]);
  WristServo.write(ServoVal);
  // Serial.println(ServoVal);
  delay(100); // 100Hz loop
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
  else { 
    // Do nothing 
  }
  ServoVal = constrain(ServoVal, ServoMin, ServoMax);  // limits range of motion
}

void requestToMaster()
{
  pixels.begin();  // initialize the pixel
  pixels.setPixelColor(0, pixels.Color(0, 0, 10));
  pixels.show();

  // Send 3x FSR analog values
  int tmpFSRVal[ARR_SIZE(FSRVal)];
  for (unsigned int i = 0; i < ARR_SIZE(FSRVal); i++) {
    tmpFSRVal[i] = map(FSRVal[i], 0, 4095, 0, 255);
    Wire.write(tmpFSRVal[i]);
  }
}

// function that executes whenever data is received from master
void receiveFromMaster(int howMany) {
  pixels.begin();  // initialize the pixel
  pixels.setPixelColor(0, pixels.Color(10, 0, 0));
  pixels.show();

  // DataFromMaster[0] = 1=FLX / 2=EXT / 0=NON - DataFromMaster[1] = 1=ABD / 2=ADD / 0=NON
   for (int i=0 ; i<howMany ; i++) {
    DataFromMaster[i] = Wire.read();
  }   

  // Serial.println("Recieved data :");
  // for (unsigned int j = 0; j < ARR_SIZE(DataFromMaster); j++) {
  //   Serial.print(DataFromMaster[j]);
  //   Serial.print(" ");
  // }
  // Serial.println("");
}