// THIS IS EMG MASTER : EMG read - AI Gesture - General slave management
// It controls : OLED display
// It reads : 5x sEMG analog value & Battery level
// It communicates : Bidirectionnaly with Wrist-Slave & Unidirectionnaly with Feedback-Salve, both over i2c

// Library from OYMOTION - EMGFilters
#include "Arduino.h"

#include "EMGFilters.h"
#define SerialToEI Serial

#define EIDSP_USE_CMSIS_DSP             1
#define EIDSP_LOAD_CMSIS_DSP_SOURCES    1
#include <semg-band_inference.h>
#if defined(EI_CLASSIFIER_FREQUENCY)
#define FREQUENCY_HZ EI_CLASSIFIER_FREQUENCY
#else
#define FREQUENCY_HZ 1000
#endif
#define INTERVAL_MS (1000 / (FREQUENCY_HZ + 1))

// features is 1 frame of data to classify
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
// keeping track of where we are in the feature array
size_t feature_ix = 0;

// Library for I2C communication
#include <Wire.h>

// Library to control RGB led
#include <Adafruit_NeoPixel.h>

// Library for OLED display
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);

// Initialize pins
#define EMG0 A1  // DFRobot Gravity EMG sensor analog value
#define EMG1 A2
#define EMG2 A3
#define EMG3 A4
#define EMG4 A5
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5

// create a pixel strand with 1 pixel on PIN_NEOPIXEL
Adafruit_NeoPixel pixels(1, PC0);

// Initialize variables
const byte MSaddress_EMG = 0x70;
const byte SLaddress_Ctrl = 0x71;
const byte SLaddress_Fdbck = 0x72;
const int EMGPins[] = { EMG0, EMG1, EMG2, EMG3, EMG4 };
int LastButtonValue = 0;
int DataToCtrlSlave[] = { 0, 0 };  // DataToCtrlSlave[0] = 1=FLX / 2=EXT / 0=NON - DataFromMaster[1] = 1=ABD / 2=ADD / 0=NON
int DataToFdbckSlave[] = { 0, 0, 0, 90 };     // DataToFdbckSlave[0,1,2] = PrxVib,MidVib,DisVib [0:novib - 1:vibrate] / DataToFdbckSlave[3] = Servo angle [0-180]
int lastVib[] = { 0, 0, 0 }; // Stored FSR values
int FSRVal[] = { 0, 0, 0 }; // FSR values from 0 to 255
int FSRthreshold = 25;
int FdbckServoMin = 10;
int FdbckServoMax = 170;
unsigned long start_interval_ms = 0;
unsigned long interval_ms = 0;

// Initialize EMG Filter functions
#define ARR_SIZE(a) (sizeof(a) / sizeof(a[0]))
// Cycle buffer
typedef struct
{
  uint8_t index;
  uint16_t buf[64]; /* Buffer for rectified AC value */
  uint32_t sum;     /* Sum for fast caculation of mean value */
} CycleBuf_t;
// Append to cycle buffer
#define CYCLE_BUF_ADD(cb, val) \
  { \
    cb.sum -= cb.buf[cb.index]; \
    cb.sum += (val); \
    cb.buf[cb.index] = (val); \
    cb.index = (cb.index + 1) % ARR_SIZE(cb.buf); \
  }
// Get mean value of cycle buffer
#define CYCLE_BUF_MEAN(cb) (cb.sum / ARR_SIZE(cb.buf))
CycleBuf_t rectifiedAcBuf[ARR_SIZE(EMGPins)];
EMGFilters myFilter[ARR_SIZE(EMGPins)];
// The filters work only with fixed sample frequency of
// "SAMPLE_FREQ_500HZ" or "SAMPLE_FREQ_1000HZ".
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_500HZ;
// Time interval for processing the input signal.
unsigned long long interval = 1000000ul / sampleRate;
// Set the frequency of power line hum to filter out.
// "NOTCH_FREQ_50HZ" or "NOTCH_FREQ_60HZ"
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;

void setup() {
  pixels.begin();  // initialize the pixel
  pixels.setPixelColor(0, pixels.Color(4, 36, 4));
  pixels.show();

  // EMGFilter initialization
  for (int i = 0; i < ARR_SIZE(EMGPins); i++) {
    myFilter[i].init(sampleRate, humFreq, true, true, true);

    rectifiedAcBuf[i].sum = 0;
    rectifiedAcBuf[i].index = 0;

    for (int j = 0; j < ARR_SIZE(rectifiedAcBuf[i].buf); j++) {
      rectifiedAcBuf[i].buf[j] = 0;
    }
  }

  // Initialize display
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  display.begin(0x3C, true);  // Address 0x3C default
  display.clearDisplay();
  display.setRotation(1);
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 0);
  display.println("Loading ...");
  display.display();

  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  if (!digitalRead(BUTTON_A)) {
    LastButtonValue = 1;
    // Reset screen
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("A : Stream Data");
    display.println("Streaming to serial");
    display.println("Run in PowerShell");
    display.println("edge-impulse-data-forwarder");
    display.display();
  }
  if (!digitalRead(BUTTON_B)) {
    LastButtonValue = 2;
    // Reset screen
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("B : Test AI model");
    display.display();
  }
  if (!digitalRead(BUTTON_C)) {
    LastButtonValue = 3;
    // Reset screen
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("C : Start prog.");
    display.display();
  }

  switch (LastButtonValue) {
    case 1:
      {
        // - Get EMG values
        // - Filter EMG values
        // - Stream EMG values to Serial
        static unsigned long last_interval_ms = 0;
        if (millis() > last_interval_ms + INTERVAL_MS) {
          last_interval_ms = millis();
          // Note: `micros()` will overflow and reset every about 70 minutes.
          unsigned long long timeStamp = micros();
          // Filter processing
          int data = 0, dataAfterFilter = 0;
          for (int i = 0; i < ARR_SIZE(EMGPins); i++) {
            data = analogRead(EMGPins[i]);
            dataAfterFilter = myFilter[i].update(data);
            // Rectification
            CYCLE_BUF_ADD(rectifiedAcBuf[i], abs(dataAfterFilter));
            // Simple envelope calculation
            uint16_t envelope = CYCLE_BUF_MEAN(rectifiedAcBuf[i]);
            //SerialToEI.print(128 + dataAfterFilter); // Draw offset = 128
            SerialToEI.print(envelope);
            SerialToEI.print('\t');
          }
          SerialToEI.println();
        }
      }
      break;
    case 2:
      {
        // - Get EMG values
        // - Filter EMG values
        // - Detect current gesture from EMG signal
        static unsigned long last_interval_ms = 0;
        if (millis() > last_interval_ms + INTERVAL_MS) {
          last_interval_ms = millis();

          // Filter processing
          int data = 0, dataAfterFilter = 0;
          for (int i = 0; i < ARR_SIZE(EMGPins); i++) {
            data = analogRead(EMGPins[i]);
            dataAfterFilter = myFilter[i].update(data);
            // Rectification
            CYCLE_BUF_ADD(rectifiedAcBuf[i], abs(dataAfterFilter));
            // Simple envelope calculation
            uint16_t envelope = CYCLE_BUF_MEAN(rectifiedAcBuf[i]);
            features[feature_ix++] = envelope;
          }
        }

        // features buffer full? then classify!
        if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
          ei_impulse_result_t result;

          // create signal from features frame
          signal_t signal;
          numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

          // run classifier
          EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
          ei_printf("run_classifier returned: %d\n", res);
          if (res != 0) return;

          // print predictions
          ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                    result.timing.dsp, result.timing.classification, result.timing.anomaly);

          // print the predictions
          for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("%s:\t%d\n", result.classification[ix].label, static_cast<int>(result.classification[ix].value * 100));
          }
          #if EI_CLASSIFIER_HAS_ANOMALY == 1
                    ei_printf("anomaly:\t%d\n", static_cast<int>(result.anomaly));
          #endif

          // reset features frame
          feature_ix = 0;

          // - Display current gesture on OLED screen
          display.clearDisplay();
          display.setCursor(0, 0);
          display.println("B : Test AI model");                                                   // LINE 1
          for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) { 
            display.print(result.classification[ix].label);
            display.print(": ");
            display.println(String(static_cast<int>(result.classification[ix].value * 100)));     // LINE 2-3-4-5-6
          }
          int AItime = result.timing.dsp + result.timing.classification + result.timing.anomaly;
          display.print("AI time [ms]: ");
          display.println(String(AItime));                                                        // LINE 7
          display.print("Anomaly ?: ");                        
          display.println(String(EI_CLASSIFIER_HAS_ANOMALY));                                     // LINE 8
          display.display();
        }
      }
      break;
    case 3:
      {
        // - Get EMG values
        // - Filter EMG values
        // - Detect current gesture from EMG signal
        static unsigned long last_interval_ms = 0;
        if (millis() > last_interval_ms + INTERVAL_MS) {
          last_interval_ms = millis();

          // Filter processing
          int data = 0, dataAfterFilter = 0;
          for (int i = 0; i < ARR_SIZE(EMGPins); i++) {
            data = analogRead(EMGPins[i]);
            dataAfterFilter = myFilter[i].update(data);
            // Rectification
            CYCLE_BUF_ADD(rectifiedAcBuf[i], abs(dataAfterFilter));
            // Simple envelope calculation
            uint16_t envelope = CYCLE_BUF_MEAN(rectifiedAcBuf[i]);
            features[feature_ix++] = envelope;
          }
        }

        // features buffer full? then classify!
        if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
          ei_impulse_result_t result;

          // create signal from features frame
          signal_t signal;
          numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

          // run classifier
          EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
          ei_printf("run_classifier returned: %d\n", res);
          if (res != 0) return;

          // print predictions
          ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                    result.timing.dsp, result.timing.classification, result.timing.anomaly);

          // print the predictions
          for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("%s:\t%d\n", result.classification[ix].label, static_cast<int>(result.classification[ix].value * 100));
          }
          #if EI_CLASSIFIER_HAS_ANOMALY == 1
                    ei_printf("anomaly:\t%d\n", static_cast<int>(result.anomaly));
          #endif

          // Find highest probabilitiy gesture in predictions
          unsigned ix_max=0; 
          unsigned arr_max=0; 
          for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (static_cast<int>(result.classification[ix].value > arr_max)) {
              arr_max = static_cast<int>(result.classification[ix].value);
              ix_max = ix;
            }
          }

          // DataToCtrlSlave[] = { 0, 0 };  
          // - DataToCtrlSlave[0] = 1=FLX / 2=EXT 0=NON
          // - DataFromMaster[1] = 1=ABD / 2=ADD / 0=NON
          if(result.classification[ix_max].label == "FIST"){ DataToCtrlSlave[0] = 1; DataToCtrlSlave[1] = 0; } // FLX
          if(result.classification[ix_max].label == "PALM"){ DataToCtrlSlave[0] = 2; DataToCtrlSlave[1] = 0; } // EXT
          if(result.classification[ix_max].label == "TWOF"){ DataToCtrlSlave[0] = 0; DataToCtrlSlave[1] = 1; } // ABD
          if(result.classification[ix_max].label == "THRF"){ DataToCtrlSlave[0] = 0; DataToCtrlSlave[1] = 2; } // ADD
          else { DataToCtrlSlave[0] = 0; DataToCtrlSlave[1] = 0; } // NON

          // reset features frame
          feature_ix = 0;

          // - Display current gesture on OLED screen
          display.clearDisplay();
          display.setCursor(0, 0);
          display.println("B : Test AI model"); // LINE 1
          for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) { 
            display.print(result.classification[ix].label);
            display.print(": ");
            display.println(String(static_cast<int>(result.classification[ix].value * 100))); // LINE 2-3-4-5-6
          }
          int AItime = result.timing.dsp + result.timing.classification + result.timing.anomaly;
          display.print("AI time [ms]: ");
          display.println(String(AItime)); // LINE 7
          display.display();
        }

        // - Send commands to Wrist-Slave
        //   - DataToCtrlSlave[] = { 0, 0 };  
        //   - DataToCtrlSlave[0] = 1=FLX / 2=EXT 0=NON
        //   - DataFromMaster[1] = 1=ABD / 2=ADD / 0=NON
        Wire.beginTransmission(SLaddress_Ctrl); // transmit to SLAVE
        // Wire.write((const uint8_t *)DataToCtrlSlave,(int)ARR_SIZE(DataToFdbckSlave));
        for(unsigned int i=0; i < ARR_SIZE(DataToCtrlSlave); i++) {
          Wire.write(DataToCtrlSlave[i]);  // write command to buffer
        }
        Wire.endTransmission();               // transmit buffer

        // - Request 3x FSR values from Wrist-Slave (target 100-1000Hz)
        //   - [4095,4095,4095] (Prx,Mid,Dist analog reads)
        Wire.requestFrom(SLaddress_Ctrl, ARR_SIZE(FSRVal));    // request 3x FSR values from CTRL Slave
        for (int i = 0; i < ARR_SIZE(FSRVal); i++) {
          FSRVal[i] = Wire.read();
        }
        // SerialToEI.println("Recieved data from wirst slave :");
        // for (int j = 0; j < ARR_SIZE(FSRVal); j++) {
        //   SerialToEI.print(FSRVal[j]);
        //   SerialToEI.print(" ");
        // }
        // SerialToEI.println("");

        // Compute necessary vibrations
        for (unsigned int i = 0; i < ARR_SIZE(FSRVal); i++) {
          if ((FSRVal[i] >= FSRthreshold) && (lastVib[i] == 0)) {
            DataToFdbckSlave[i] = 1;
          }
          else if ((FSRVal[i] < FSRthreshold) && (lastVib[i] == 1)) {
            DataToFdbckSlave[i] = 1;
          }
          else {
            DataToFdbckSlave[i] = 0;
          }
          lastVib[i] = DataToFdbckSlave[i];
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

        // - Send feedback to Feedback_Slave (target 100-1000Hz)
        //   - DataToFdbckSlave[] = {0, 0, 0, 90}; 
        //     - DataToFdbckSlave[0,1,2] = PrxVib,MidVib,DisVib [0:novib - 1:vibrate] 
        //     - DataToFdbckSlave[3] = Servo angle [FdbckServoMin -FdbckServoMax]
        Wire.beginTransmission(SLaddress_Fdbck); // transmit to SLAVE
        for (unsigned int i = 0; i < ARR_SIZE(DataToFdbckSlave); i++) {
          Wire.write(DataToFdbckSlave[i]);  // write command to buffer
        //   Serial.print(DataToFdbckSlave[i]);
        //   Serial.print(" ");
        }
        // Serial.println("");
        Wire.endTransmission();               // transmit buffer

        // calculate loop time
        if(start_interval_ms != 0) { unsigned long interval_ms = start_interval_ms - millis(); }
          
        // Start timer for loop time
        unsigned long start_interval_ms = millis();

        // display loop time
        display.print("Loop time : ");                        
        display.println(String(interval_ms));  // LINE 8
        display.display();
      }
      break;
    default:
      {
        // Display options to user
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Thumb-Xtra v0.1");
        display.println("A : Stream data");
        display.println("B : Test AI model");
        display.println("C : Start Programm");
        display.println("");
        display.println("");
        display.println("");
        display.println("by Reza SN");
        display.display();
      }
      break;
  }
}

void ei_printf(const char *format, ...) {
    static char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.write(print_buf);
    }
}