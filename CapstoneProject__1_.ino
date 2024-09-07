#include <Servo.h>
#include <TomIBT2.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SSD1306.h>
#include "LIDARLite_v3HP.h"
#include <stdint.h>

Adafruit_INA219 ina219;

#define MonitorPin            5
#define TriggerPin            6
#define MOTOR_PIN_R_EN        7
#define MOTOR_PIN_L_EN        8
#define MOTOR_PIN_RPWM        10  // PWM 490.20Hz
#define MOTOR_PIN_LPWM        9   // PWM 490.20Hz
#define encoderPinA 3
#define encoderPinB 4

Servo servo;

uint32_t distance;
uint32_t maxDistance;
uint32_t startTime;
uint32_t endTime;
bool     newDistance = false;
bool     measuring   = false;
volatile long encoderCount = 0;
volatile float current_mA = 0;
volatile long Pulses = 0;

TomIBT2 motor(MOTOR_PIN_R_EN, MOTOR_PIN_L_EN, MOTOR_PIN_RPWM, MOTOR_PIN_LPWM);

byte CurrentState=0;
const byte MeasureDistance = 0;
const byte LowerXDistance =  1;
const byte Retract     = 2; 
const byte KillSwitch    = 3;
//const byte Retra  = 4;

unsigned long previousMillis = 0UL;
unsigned long interval = 1000UL;

void setup() {
  Serial.begin(115200);

  servo.attach(11);
  //servo.writeMicroseconds(2000); // give the actuator a 2ms pulse to retract the arm (1000us = 1ms)
  //delay(100); // delay a few seconds to give the arm time to retract

  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }

  motor.begin();

  Serial.println("Hello!");

  if(! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  //ina219.setCalibration_16V_400mA();

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);

  pinMode(MonitorPin, INPUT);
  pinMode(TriggerPin, OUTPUT);
  digitalWrite(TriggerPin, LOW); // Set trigger LOW for continuous read

  startTime = micros();
  endTime   = startTime;

  CurrentState = MeasureDistance;
}

void handleEncoder() {
  if (digitalRead(encoderPinA) > digitalRead(encoderPinB)){
    encoderCount++;
  }
  else{
    encoderCount--;
  }
}

void States(){
  switch (CurrentState){
    case MeasureDistance:

      unsigned long start = millis();
      previousMillis = start;
      while(previousMillis - start < 2000){
        
        if (digitalRead(MonitorPin))
        {
          if (measuring == false)
          {
          startTime   = micros();
          measuring   = true;
          }
        }
        else
        {
          if (measuring == true)
          {
            endTime     = micros();
            measuring   = false;
            newDistance = true;
          }
        }

        // If we get a new reading, print it
        if (newDistance == true)
        {
          distance = (endTime - startTime) / 10; // 10usec = 1 cm of distance
          Serial.println(distance); // Print measured distance
          if(distance > maxDistance){
            maxDistance = distance;
          }
          newDistance = false;
        }
        previousMillis = millis();
      }
      Pulses = maxDistance*18;
      CurrentState = LowerXDistance;
      Serial.print("Max Distance: ");Serial.println(maxDistance);Serial.println("");
      //break;
    
    case LowerXDistance:
      while(encoderCount < Pulses){
        motor.rotate(64,TomIBT2::CW);
        float shuntvoltage = 0;
        float busvoltage = 0;
        float current_mA = 0;
        float loadvoltage = 0;
        float power_mW = 0;

        shuntvoltage = ina219.getShuntVoltage_mV();
        busvoltage = ina219.getBusVoltage_V();
        current_mA = ina219.getCurrent_mA();
        power_mW = ina219.getPower_mW();
        loadvoltage = busvoltage + (shuntvoltage / 1000);
        current_mA = ina219.getCurrent_mA();
        Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
        Serial.print("Encoder Count: "); Serial.println(encoderCount);
        Serial.println("");
        //delay(2000);
        //break;
      }
      motor.stop();
      delay(2000);
      CurrentState = Retract;

    case Retract:
      while(encoderCount > 0){
        motor.rotate(64,TomIBT2::CCW);
        current_mA = ina219.getCurrent_mA();
        Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
        Serial.print("Encoder Count: "); Serial.println(encoderCount);
        Serial.println("");
        //delay(2000);
        //break;
      }
      motor.stop();
      CurrentState = KillSwitch;

    case KillSwitch:
      Serial.println("cutting");
      servo.writeMicroseconds(1000); // 1ms pulse to extend the arm
      delay(500);
      servo.writeMicroseconds(2000); // 1ms pulse to retract the arm
      delay(500);


  //case ReleasePayload:
    //Serial.println("rdy");


    default:
      Serial.println(maxDistance);
      Serial.println("done");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  States();
  //Serial.println("loop");
  //Serial.println(CurrentState);
}
