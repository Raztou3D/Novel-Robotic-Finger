// THIS IS SLAVE_FDBCK : The force and contact feeback armband
// It controls : Servo pusher & 3x Haptic motors
// It reads : nothing
// It communicates : Unidirectionnaly with Maser over i2c

#include <Arduino.h>
// Library for I2C communication
#include <Wire.h>
// Library for Servo montor control
#include <Servo.h>
// Library to control RGB led
#include <Adafruit_NeoPixel.h>

// Initialize pins
#define PrxVib 7        // Proximal phalange Force Sensor 12bit
#define MidVib 8        // Middle phalange Force Sensor 12bit
#define DisVib 9        // Distal phalange Force Sensor 12bit
#define PusherSERVO 10  // Wirst servo motor for PWM

#define ARR_SIZE(a) (sizeof(a) / sizeof(a[0]))

// Initialize objects
Servo FdbckServo;

// create a pixel strand with 1 pixel on PIN_NEOPIXEL
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

// Initialize variables
const byte MSaddress_EMG = 0x70;
const byte SLaddress_Ctrl = 0x71;
const byte SLaddress_Fdbck = 0x72;
const int HapticPins[] = { PrxVib, MidVib, DisVib };
int DataFromMaster[] = { 0, 0, 0, 90 };  // DataFromMaster[0,1,2] = PrxVib,MidVib,DisVib [0:novib - 1:vibrate] / DataFromMaster[3] = Servo angle [0-180]
const long VibTime = 250;                // Haptic motos vibration time in milliseconds [ms]
long previousMillis[] = { 0, 0, 0 };     // Will store last time Haptic motors were updated, per motor
int ServoVal = 30;

// Declare fonctions
void receiveFromMaster(int howMany);

void setup() {
  pixels.begin();  // initialize the pixel
  pixels.setPixelColor(0, pixels.Color(0, 10, 0));
  pixels.show();

  for (unsigned int i = 0; i < ARR_SIZE(HapticPins); i++) {
    pinMode(HapticPins[i], OUTPUT);
  }
  FdbckServo.attach(PusherSERVO);

  Wire.begin(SLaddress_Fdbck);
  Wire.onReceive(receiveFromMaster);
  Serial.begin(115200);
}

void loop() {
  // - Update Haptic motors
  for (unsigned int i = 0; i < ARR_SIZE(HapticPins); i++) {
    unsigned long currentMillis = millis();
    if (DataFromMaster[i] == 1) {
      previousMillis[i] = currentMillis;
      analogWrite(HapticPins[i], 255);
    }
    if (currentMillis - previousMillis[i] >= VibTime) {
      analogWrite(HapticPins[i], 0);
      DataFromMaster[i] = 0;
    }
  }
  // - Update Wrist Servo
  FdbckServo.write(DataFromMaster[3]);
  delay(100);
  
  Serial.println("SAMD21 - Loop");
  for (unsigned int j = 0; j < ARR_SIZE(DataFromMaster); j++) {
    Serial.print(DataFromMaster[j]);
    Serial.print(" ");
  }
  Serial.println("");

  pixels.begin();  // initialize the pixel
  pixels.setPixelColor(0, pixels.Color(0, 0, 10));
  pixels.show();
}

// function that executes whenever data is received from master
void receiveFromMaster(int howMany) {
  // DataFromMaster[0,1,2] = PrxVib,MidVib,DisVib [0:novib - 1:vibrate] / DataFromMaster[3] = Servo angle [0-180]
  for (int i=0 ; i<howMany ; i++) {
    DataFromMaster[i] = Wire.read();
  }   
  pixels.begin();  // initialize the pixel
  pixels.setPixelColor(0, pixels.Color(10, 0, 0));
  pixels.show();
}