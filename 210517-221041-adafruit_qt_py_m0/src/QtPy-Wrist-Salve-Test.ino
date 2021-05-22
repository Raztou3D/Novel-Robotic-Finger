// Library for Servo montor control
#include <Servo.h>

// Initialize pins
#define PrxFSR A2  // Proximal phalange Force Sensor 12bit
#define MidFSR A1  // Middle phalange Force Sensor 12bit
#define DisFSR A0  // Distal phalange Force Sensor 12bit

#define MotCW 6  // CW Finger motor rotation
#define MotCCW 7  // CCW Finger motor rotation
#define WristSERVO 8  // Wirst servo motor for PWM

// Initialize objects
Servo WristServo;

// Initialize variables
const byte MSaddress_EMG = 3;
const byte SLaddress_Ctrl = 4;
const byte SLadDress_Fdbck = 5;
int FSRVal[] = {0,0,0};
int ServoVal = 140;

void setup() {
  analogReadResolution(12);
  pinMode(MotCW, OUTPUT);
  pinMode(MotCCW, OUTPUT);
  digitalWrite(MotCW, LOW);
  digitalWrite(MotCCW, LOW);
  //WristServo.attach(WristSERVO);
  //WristServo.write(ServoVal);
  
  Serial.begin(115200);
}

void loop() {
  FSRVal[0] = analogRead(PrxFSR);
  FSRVal[1] = analogRead(MidFSR);
  FSRVal[2] = analogRead(DisFSR);
  Serial.print(FSRVal[0]);
  Serial.print(" ");
  Serial.print(FSRVal[1]);
  Serial.print(" ");
  Serial.println(FSRVal[2]);
  //WristServo.write(ServoVal);

  
  digitalWrite(MotCW, LOW);
  digitalWrite(MotCCW, LOW);

  String input = "";
  while (Serial.available() > 0)
  {
      input = Serial.readString();
      Serial.println(input);
  }
  if(input.toInt() == 0){
    Serial.println("Do nothing");
    digitalWrite(MotCW, LOW);
    digitalWrite(MotCCW, LOW);
  }
  if(input.toInt() == 10){
    Serial.println("Do CW");
    digitalWrite(MotCW, HIGH);
    digitalWrite(MotCCW, LOW);
  }
  if(input.toInt() == 1){
    Serial.println("Do CCW");
    digitalWrite(MotCW, LOW);
    digitalWrite(MotCCW, HIGH);
  }
  if(input.toInt() == 11){
    Serial.println("Do nothing");
    digitalWrite(MotCW, LOW);
    digitalWrite(MotCCW, LOW);
  }
  else{
    Serial.println("invlid input");
  }
  
  delay(2000);
}
