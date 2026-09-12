#include <Arduino.h>
#include <ESP32Servo.h>

#include "arm_receiver.h"

Servo shoulderServo;
Servo elbowServo;
const int shoulderPin = GPIO_NUM_25; // pins available for PWM signals
const int elbowPin = GPIO_NUM_26;

float* armptr;

void setup() {
  Serial.begin(115200);

  // ESP32 explicit PWM timer allocation, needed to run more than one servo simultaneously
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  shoulderServo.setPeriodHertz(50);  // standard 50Hz servo signal
  elbowServo.setPeriodHertz(50);

  // 500-2500us pulse range as specified in datasheet
  shoulderServo.attach(shoulderPin, 500, 2500);
  elbowServo.attach(elbowPin, 500, 2500);

  receiver_setup();
}

void loop() {
  // pointer to array of L-shoulder, L-elbow, R-shoulder, R-elbow
  armptr = receiver_loop();

  shoulderServo.write(armptr[2]);  // rightShoulderAngle: [0,180]

  // rightElbowAngle is signed [-180,180]
  // Servo::write() only accepts [0,180], so remap is necessary
  int elbowServoAngle = map((long)armptr[3], -180, 180, 0, 180);
  elbowServo.write(elbowServoAngle);
}