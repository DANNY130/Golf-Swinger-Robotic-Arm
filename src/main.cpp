#include <Arduino.h>

#include <Wire.h>                     //to use i2c
#include <Adafruit_PWMServoDriver.h>  // Using Adafruit driver library
#include "IK.h" // Include the IK header file
#include <stdio.h>                                        // Include for sscanf
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  //Default address 0x40

#define SERVO_MIN 125  // Corresponds to ~500µs pulse
#define SERVO_MAX 575  // Corresponds to ~2500µs pulse

#define D1 91.22 // Distance from ground to q1 in mm
#define A2 113.12 // Distance from q1 to q2 in mm
#define D4 97.05 // Distance from q2 to wrist in mm
#define D6 55.0 // Distance from wrist to gripper in mm
// Need to go and fix these constants in mm

void motorActivator(uint8_t servoNum, float angle) {
  int pulseLength = map(round(angle), 0,  180, SERVO_MIN, SERVO_MAX); // Need to round up and map to integer for PWM control
  pwm.setPWM(servoNum, 0, pulseLength); //actuate the servos
}

void setup() {
  Serial.begin(9600);  // Start serial communication for debugging purposes
  pwm.begin();         // Initialize the PCA9685 module.
  pwm.setPWMFreq(60);  // Set the PWM frequency suitable for servos.
}


void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove any leading/trailing whitespace or newline
    float x, y, z;
    int firstComma = input.indexOf(',');
    int secondComma = input.indexOf(',', firstComma + 1);

    if (firstComma != -1 && secondComma != -1) {
      String xStr = input.substring(0, firstComma);
      String yStr = input.substring(firstComma + 1, secondComma);
      String zStr = input.substring(secondComma + 1);

      x = xStr.toDouble();
      y = yStr.toDouble();
      z = zStr.toDouble();

      Serial.print("Parsed coordinates -> X: ");
      Serial.print(x);
      Serial.print(" | Y: ");
      Serial.print(y);
      Serial.print(" | Z: ");
      Serial.println(z);
    } else {
      Serial.println("Invalid input. Please enter x, y, z, separated by spaces.");
    }

    double w[3][3]= {{0,1,0},    //the target rotation matrix
                 {0,0,1},
                 {1,0,0}};
    double angles[6] = {0, 0, 0, 0, 0, 0}; // Array to hold the calculated joint angles

    // Compute inverse kinematics
    IK::IK_DATA ikData = {D1, A2, D4, D6}; // Initialize the IK data structure
    IK ik(ikData); // Create an instance of the IK class
    ik.inverseKinematics(x, y, z, w, angles, false); // Perform inverse kinematics

    Serial.print("Calculated angles: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(angles[i], 4); // Print each angle with 4 decimal places
      if (i < 5) Serial.print(", ");
    }
    Serial.println();
    delay(1000); // Delay for a second before the next loop iteration

    // Convert angles to servo positions
    for (int i = 0; i < 6; i++) {
      float servoAngle = IK::toDeg(angles[i]); // Convert radians to degrees
      motorActivator(i, servoAngle); // Activate the servo with the calculated angle
      Serial.print("Servo ");
      Serial.print(i);
      Serial.print(" set to angle: ");
      Serial.println(servoAngle);
    }

    //Double Check by performing forward kinematics
    double jointPos[6][3] = {0}; // Array to hold the joint positions
    ik.forwardKinematics(angles, jointPos); // Perform forward kinematics
    Serial.println("Joint positions after forward kinematics:");
    for (int i = 0; i < 6; i++) {
      Serial.print("Joint ");
      Serial.print(i);
      Serial.print(": X: ");
      Serial.print(jointPos[i][0], 4);
      Serial.print(" | Y: ");
      Serial.print(jointPos[i][1], 4);
      Serial.print(" | Z: ");
      Serial.println(jointPos[i][2], 4);
    }
  }
}
