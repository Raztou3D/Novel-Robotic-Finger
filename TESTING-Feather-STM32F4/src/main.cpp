// THIS IS EMG MASTER : EMG read - AI Gesture - General slave management
// It controls : OLED display
// It reads : 5x sEMG analog value & Battery level
// It communicates : Bidirectionnaly with Wrist-Slave & Unidirectionnaly with Feedback-Salve, both over i2c

// comment to test github autocommit.

// Library from OYMOTION - EMGFilters
#include "Arduino.h"

// Library for I2C communication
#include "Wire.h"

// Initialize pins

#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5

// Initialize variables
const byte MSaddress_EMG = 0x70;
const byte SLaddress_Ctrl = 0x71;
const byte SLaddress_Fdbck = 0x72;
int LastButtonValue = 0;
int DataToCtrlSlave[] = { 0, 0 };  // DataToCtrlSlave[0] = 1=FLX / 2=EXT / 0=NON - DataFromMaster[1] = 1=ABD / 2=ADD / 0=NON
int DataToFdbckSlave[] = { 0, 0, 0, 0 };     // DataToFdbckSlave[0,1,2] = PrxVib,MidVib,DisVib [0:novib - 1:vibrate] / DataToFdbckSlave[3] = Servo angle [0-180]
int FSRVal[] = { 0, 0, 0 }; // FSR values from 0 to 255
int lastFSRVal[] = { 0, 0, 0 }; // Stored FSR values
int FSRthreshold = 20;
int FdbckServoMin = 10;
int FdbckServoMax = 170;

// Initialize EMG Filter functions
#define ARR_SIZE(a) (sizeof(a) / sizeof(a[0]))

void setup() {
  // Initialize display
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  if (!digitalRead(BUTTON_A)) {
    LastButtonValue = 1;
    FSRVal[0] = 0;
    FSRVal[1] = 0;
    FSRVal[2] = 0;
    DataToCtrlSlave[0] = 0;
    DataToCtrlSlave[1] = 0;
    Serial.println("Button A pressed.");
  }
  if (!digitalRead(BUTTON_B)) {
    LastButtonValue = 2;
    FSRVal[0] = 500;
    FSRVal[1] = 0;
    FSRVal[2] = 0;
    DataToCtrlSlave[0] = 1;
    DataToCtrlSlave[1] = 0;
    Serial.println("Button B pressed.");
  }
  if (!digitalRead(BUTTON_C)) {
    LastButtonValue = 3;
    FSRVal[0] = 500;
    FSRVal[1] = 200;
    FSRVal[2] = 100;
    DataToCtrlSlave[0] = 2;
    DataToCtrlSlave[1] = 0;
    Serial.println("Button C pressed.");
  }

  // Compute necessary vibrations
  for (unsigned int i = 0; i < ARR_SIZE(FSRVal); i++) {
    if ((FSRVal[i] >= FSRthreshold) && (lastFSRVal[i] < FSRthreshold)) {
      DataToFdbckSlave[i] = 1;
    }
    else if ((FSRVal[i] < FSRthreshold) && (lastFSRVal[i] >= FSRthreshold)) {
      DataToFdbckSlave[i] = 1;
    }
    else {
      DataToFdbckSlave[i] = 0;
    }
    lastFSRVal[i] = FSRVal[i];
  }

  // Compute pusher servo value
  DataToFdbckSlave[3] = 0;
  for (unsigned int i = 0; i < ARR_SIZE(FSRVal); i++) {
    if (FSRVal[i] >= FSRthreshold) {
      DataToFdbckSlave[3] += map(FSRVal[i], 0, 255, FdbckServoMin, FdbckServoMax);
    }
    else {
      DataToFdbckSlave[3] += FdbckServoMin;
    }
  }
  DataToFdbckSlave[3] = DataToFdbckSlave[3] / ARR_SIZE(FSRVal);

  // - Send commands to Wrist-Slave 
  //   - DataToCtrlSlave[] = { 0, 0 };
  //   - DataToCtrlSlave[0] = 1=FLX / 2=EXT 0=NON
  //   - DataFromMaster[1] = 1=ABD / 2=ADD / 0=NON
  Serial.println("Sending data from to wrist slave :");
  Wire.beginTransmission(SLaddress_Ctrl); // transmit to SLAVE
  for (unsigned int i = 0; i < ARR_SIZE(DataToCtrlSlave); i++) {
    Wire.write(DataToCtrlSlave[i]);  // write command to buffer
  }
  Wire.endTransmission();               // transmit buffer

  // - Request 3x FSR values from Wrist-Slave (target 100-1000Hz)
  //   - [4095,4095,4095] (Prx,Mid,Dist analog reads)
  Wire.requestFrom(SLaddress_Ctrl, ARR_SIZE(FSRVal));    // request 3x FSR values from CTRL Slave
  for (unsigned int i = 0; i < ARR_SIZE(FSRVal); i++) {
    FSRVal[i] = Wire.read();
  }
  Serial.println("Recieved data from wirst slave :");
  for (unsigned int j = 0; j < ARR_SIZE(FSRVal); j++) {
    Serial.print(FSRVal[j]);
    Serial.print(" ");
  }
  Serial.println("");

  Serial.println("FSRVal : ");

  for (unsigned int i = 0; i < ARR_SIZE(FSRVal); i++) {
    Serial.print(FSRVal[i]);
    Serial.print(" ");
  }
  Serial.println("");

  // - Send feedback to Feedback_Slave (target 100-1000Hz)
  //   - DataToFdbckSlave[] = {0, 0, 0, 90};
  //     - DataToFdbckSlave[0,1,2] = PrxVib,MidVib,DisVib [0:novib - 1:vibrate]
  //     - DataToFdbckSlave[3] = Servo angle [FdbckServoMin -FdbckServoMax]
  Serial.println("Sending to Feedback Slave ...");
  Wire.beginTransmission(SLaddress_Fdbck); // transmit to SLAVE
  // Wire.write((const uint8_t *)DataToFdbckSlave,(int)ARR_SIZE(DataToFdbckSlave));
  for (unsigned int i = 0; i < ARR_SIZE(DataToFdbckSlave); i++) {
    Wire.write(DataToFdbckSlave[i]);  // write command to buffer
    Serial.print(DataToFdbckSlave[i]);
    Serial.print(" ");
  }
  Serial.println("");
  Wire.endTransmission();  // transmit buffer
  Serial.println("Sent to Feedback Slave.");

  delay(100);
}
