#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"
#include <Servo.h> 

Servo myServo; 
const int trigPin = 13; // trig
const int echoPin = 12; // echo
bool isStop = false;


void setup()
{

  Application_FunctionSet.ApplicationFunctionSet_Init();

  myServo.attach(10);
  myServo.write(90); 

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
}


int TrackingDetection_S = 300; // alt siyah
int TrackingDetection_E = 400; // üst beyaz

long readUltrasonicDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    long distanceCm = duration * 0.034 / 2;
    return distanceCm;
}

void loop() {
    long distance = readUltrasonicDistance();
    if (distance<8) {
      isStop = true;
      myServo.write(0);
    }
    else{
      isStop = false;
      myServo.write(90);
      delay(500);
    }
    Application_FunctionSet.ApplicationFunctionSet_Tracking(isStop);
    Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
    delay(20);

    if (isStop) delay(700);
  
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim(); 
        int spaceIndex = input.indexOf(' ');
        if (spaceIndex > 0) {
            int newS = input.substring(0, spaceIndex).toInt();
            int newE = input.substring(spaceIndex + 1).toInt();
            if (newS < newE) {
                TrackingDetection_S = newS;
                TrackingDetection_E = newE;
                Serial.print("Updated thresholds -> S: ");
                Serial.print(TrackingDetection_S);
                Serial.print(" | E: ");
                Serial.println(TrackingDetection_E);
            } else {
                Serial.println("Invalid input: S must be < E");
            }
        } else {
            Serial.println("Invalid format. Use: S E");
        }
    }
}
