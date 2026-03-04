// By: Robbie Leslie

#include <Arduino.h>
#include "motor_functions.h"

// #define L1 3
// #define L2 2
// #define R1 14
// #define R2 15

long prev_time;
bool mode = false;
int motor_speed = -20;

void setup(){
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  Serial.println("Starting PWM test");

  for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }

  // analogWrite(L1, 50); // ~19% duty cycle

  // analogWrite(L2, 100); // ~39%

  // analogWrite(R1, 150); // ~59%

  // analogWrite(R2, 200); // ~78%
    
  

  delay(5000); //wait 5 seconds


  // somewhere between 10 and 15 is minimum to start motor
  setMotor(RIGHT, motor_speed);
  setMotor(LEFT, motor_speed);
}


void loop(){
  if(millis() - prev_time > 3000){
    if (mode==false) {
      Serial.println("Timer triggered!");
      // motor_speed = -motor_speed;
      // setMotor(RIGHT, motor_speed);
      // setMotor(RIGHT, motor_speed);
      // setMotor(LEFT, motor_speed);
      setBothMotors(motor_speed, motor_speed);
      mode = true;
    } else {
      stopBothMotors();
      mode = false;
    }
    prev_time = millis();
  }

}