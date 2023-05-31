#include <Arduino.h>

//  Variables
unsigned long previousMillisGetHR = 0;
unsigned long previousMillisHR = 0;

const long intervalGetHR = 20;
const long intervalHR = 10000;

int PulseSensorHRWire = 32; // Pulse Sensor WIRE connected to ANALOG PIN 32
int LED = LED_BUILTIN;      //  The on-board ESP32 LED (=2)

// int Signal;           // holds the incoming raw data. Signal value can range from 0-4095
int Threshold = 2970; // Determine which Signal to "count as a beat", and which to ingore.

int cntHB = 0;
boolean ThresholdStat = true;
int BPMval = 0;

void GetHeartRate();

void setup()
{
  Serial.begin(115200);
  delay(500);

  pinMode(LED, OUTPUT); // pin that will blink to your heartbeat!

  Serial.println();
  Serial.println("Please wait 10 seconds to get the BPM Value");
}

// The Main Loop Function
void loop()
{
  GetHeartRate();

  // int PulseSensorHRVal = analogRead(PulseSensorHRWire);
  // Serial.println(PulseSensorHRVal);

  // delay(20);
}

void GetHeartRate()
{
  // Process of reading heart rate.
  unsigned long currentMillisGetHR = millis();

  if (currentMillisGetHR - previousMillisGetHR >= intervalGetHR)
  {
    previousMillisGetHR = currentMillisGetHR;

    int PulseSensorHRVal = analogRead(PulseSensorHRWire);

    if (PulseSensorHRVal > Threshold && ThresholdStat == true)
    {
      cntHB++;
      ThresholdStat = false;
      digitalWrite(LED, HIGH);
    }

    if (PulseSensorHRVal < Threshold)
    {
      ThresholdStat = true;
      digitalWrite(LED, LOW);
    }
  }

  // The process for getting the BPM value.
  unsigned long currentMillisHR = millis();

  if (currentMillisHR - previousMillisHR >= intervalHR)
  {
    previousMillisHR = currentMillisHR;

    BPMval = cntHB * 6;
    Serial.print("BPM : ");
    Serial.println(BPMval);

    cntHB = 0;
  }
}