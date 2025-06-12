#include <Wire.h>                     //to use i2c
#include <Adafruit_PWMServoDriver.h>  // Using Adafruit driver library
#include <math.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  //Default address 0x40

#define SERVO_MIN 125  // Corresponds to ~500µs pulse
#define SERVO_MAX 575  // Corresponds to ~2500µs pulse


// Need to go and fix these constants in mm
const float base_offset = 50.87;
const float base_link_upper1 = 40.35;             //Probably need to sum this with base_offset
const float h0 = base_offset + base_link_upper1;  // From the ground to the first shoulder joint
const float l1 = 113.12;
const float l2 = 125;
const float l3 = 93;

//Helper function for printing end effector position and orientation.
float endEffectorPosOri[4] = { 0, 0, 0, 0 };
float jointLinkAngles[6] = { 0, 0, 0, 0, 0, 0};
void printEndEffectorPosOri();
//Helper function for printing joint angles
void printJointLinkAngles();

// Function Definitions:
void motorActivator(uint8_t servorNum, float angle);

void calc_FK(float jointAngle[]);  // Calculating forward kinematics

void calc_IK(float* PosOri);

/************ Vadim's sensor function **************/
bool PhotoGolf();        // Check whether there is golf or not then send position of the golf ball for robot to hit


void setup() {
  Serial.begin(9600);  // Start serial communication for debugging purposes
  pwm.begin();         // Initialize the PCA9685 module.
  pwm.setPWMFreq(60);  // Set the PWM frequency suitable for servos.
  Serial.println("Select which mode you wanna do - Type 2 (Forward Kinematics) or 1 (Inverse Kinematics): ");
}


void loop() {

  /********* Debugging code
  // Serial.println("******** Results ***********");
  // Serial.println();

  // Serial.println("Solving FK for:");
  // printJointLinkAngles();
  // calc_FK(jointLinkAngles);
  // printEndEffectorPosOri();
  // Serial.println();
  */


  /************* Robot Arm activation **************/
  if (Serial.available() > 0) {
    int input = Serial.parseInt();
    delay(500);
    switch (input) {
      case 2:
        Serial.println("Enter joint angles (e.g. theta0,theta1,theta2,theta3) then Enter:");
        if (Serial.available() > 0) {
          String Input = Serial.readStringUntil('\n');
          Input.trim();  // Remove any leading/trailing whitespace or newline
          float theta0, theta1, theta2, theta3, theta4, theta5;
          int firstComma = Input.indexOf(',');
          int secondComma = Input.indexOf(',', firstComma + 1);
          int thirdComma = Input.indexOf(',', secondComma + 1);
          int fourthComma = Input.indexOf(',', thirdComma + 1);
          int fifthComma = Input.indexOf(',', fourthComma + 1);

          if (firstComma != -1 && secondComma != -1 && thirdComma != -1 && fourthComma != -1 && fifthComma != -1) {
            theta0 = Input.substring(0, firstComma).toFloat();
            theta1 = Input.substring(firstComma + 1, secondComma).toFloat();
            theta2 = Input.substring(secondComma + 1, thirdComma).toFloat();
            theta3 = Input.substring(thirdComma + 1, fourthComma).toFloat();
            theta4 = Input.substring(fourthComma + 1, fifthComma).toFloat();
            theta5 = Input.substring(fifthComma + 1).toFloat();

            Serial.print("Angles are (in degrees): ");
            Serial.print(theta0);
            Serial.print(",");
            Serial.print(theta1);
            Serial.print(",");
            Serial.print(theta2);
            Serial.print(",");
            Serial.print(theta3);
            Serial.print(",");
            Serial.print(theta4);
            Serial.print(",");
            Serial.println(theta5);
          } else {
            Serial.println("Invalid input. Please enter angles, separtated by commas");
          }

          jointLinkAngles[0] = theta0 * PI/180;
          jointLinkAngles[1] = theta1 * PI/180;
          jointLinkAngles[2] = theta2 * PI/180;
          jointLinkAngles[3] = theta3 * PI/180;
          jointLinkAngles[4] = theta4 * PI/180;
          jointLinkAngles[5] = theta5 * PI/180;


          Serial.println("******** Results ***********");
          Serial.println();

          Serial.println("Solving FK for:");
          printJointLinkAngles();
          calc_FK(jointLinkAngles);
          printEndEffectorPosOri();
          printJointLinkAngles();
          Serial.println();

          Serial.println("****** Moving Arms *********");
          for (int i = 0; i <6; i ++) {
            int servoAngle = jointLinkAngles[i] * 180 / PI;
            // for (int j = 0; j < servoAngle; j ++) {
            //   motorActivator(i, j);
            //   delay(10);
            // }
            motorActivator(i, servoAngle);
            delay(200);
            Serial.print("Servo ");
            Serial.print(i);
            Serial.print(" set to angle: ");
            Serial.println(servoAngle);
          }
        }
        break;
      case 1:
        Serial.println("Enter coordinates x,y,z cm and orientation Phi with commas then Enter: ");
        if (Serial.available() > 0) {
          String Input = Serial.readStringUntil('\n');
          Input.trim();  // Remove any leading/trailing whitespace or newline
          float x,y,z,phi;
          int firstComma = Input.indexOf(',');
          int secondComma = Input.indexOf(',', firstComma + 1);
          int thirdComma = Input.indexOf(',', secondComma + 1);

          if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
            x = Input.substring(0, firstComma).toFloat();
            y = Input.substring(firstComma + 1, secondComma).toFloat();
            z = Input.substring(secondComma + 1, thirdComma).toFloat();
            phi = Input.substring(thirdComma + 1).toFloat();

            Serial.print("You just enter coordinates and orientation: ");
            Serial.print("x: ");
            Serial.print(x);
            Serial.print("cm, y: ");
            Serial.print(y);
            Serial.print("cm, z: ");
            Serial.print(z);
            Serial.print("cm, phi: ");
            Serial.println(phi);
          } else {
            Serial.println("Invalid input. Please enter coordinates + orientation, separtated by commas");
          }

          // Set limits
          // if(abs(x) > (l1 + l2 + l3) / 10) {
          //   x = (l1 + l2 +l3) / 10;
          // }
          // if(abs(y) > (l1 + l2 + l3) / 10) {
          //   y = (l1 + l2 +l3) / 10;
          // }
          // if(abs(z) > (l1 + l2 + l3 +h0) / 10) {
          //   z = (l1 + l2 +l3 +h0) / 10;
          // }
          endEffectorPosOri[0] = x * 10;
          endEffectorPosOri[1] = y * 10;
          endEffectorPosOri[2] = z * 10;
          endEffectorPosOri[3] = phi * PI / 180;

          Serial.println("******** Results ***********");
          Serial.println();

          Serial.println("Solving IK for (mm):");
          printEndEffectorPosOri();
          Serial.println();
          calc_IK(endEffectorPosOri);
          printJointLinkAngles();
          Serial.println();

          Serial.println("****** Moving Arms *********");
          for (int i = 0; i <4; i ++) {
            int servoAngle = jointLinkAngles[i] * 180 / PI;
            // for (int j = 0; j < servoAngle; j ++) {
            //   motorActivator(i, j);
            //   delay(10);
            // }
            motorActivator(i, servoAngle);
            delay(200);
            Serial.print("Servo ");
            Serial.print(i);
            Serial.print(" set to angle: ");
            Serial.println(servoAngle);
          }
          
        }
      break;
    }
  }
}

bool PhotoGolf() {
  // Input your code here

  // Your code should replace the input coordinates from the Serial Monitor by 
  // bypassing Forward Kinematics and goes straight to Inverse Kinematics
  // Instantly send coordinates (x,y,z) of the golf ball for the robot move there
  // This should automatedly trigger the robot to move itself.

}

void motorActivator(uint8_t servoNum, float angle) {
  int pulseLength = map(round(angle), 0, 180, SERVO_MIN, SERVO_MAX);  // Need to round up and map to integer for PWM control
  pwm.setPWM(servoNum, 0, pulseLength);                               //actuate the servos
}

void printEndEffectorPosOri() {
  Serial.print("End Effector PosOri: X | ");
  Serial.print(endEffectorPosOri[0] / 10);
  Serial.print("cm, Y |  ");
  Serial.print(endEffectorPosOri[1] / 10);
  Serial.print("cm, Z | ");
  Serial.print(endEffectorPosOri[2] / 10);
  Serial.print("cm, Phi | ");
  Serial.println(endEffectorPosOri[3] * 180/PI);
}

void printJointLinkAngles() {
  Serial.print("Joint Angles: ");
  Serial.print(jointLinkAngles[0] * 180 / PI);
  Serial.print(", ");
  Serial.print(jointLinkAngles[1] * 180 / PI);
  Serial.print(", ");
  Serial.print(jointLinkAngles[2] * 180 / PI);
  Serial.print(", ");
  Serial.print(jointLinkAngles[3] * 180 / PI);
  Serial.print(",");
  Serial.print(jointLinkAngles[4] * 180 / PI);
  Serial.print(",") ;
  Serial.println(jointLinkAngles[5] * 180 / PI);

}

void calc_FK(float jointAngles[]) {
  float j0 = jointAngles[0];  //theta0
  Serial.print("theta0 is:");
  Serial.println(j0);
  float j1 = jointAngles[1];  //theta1
  float j2 = jointAngles[2];
  float j3 = jointAngles[3];  //theta3

  float x1 = l1*cos(PI - j1);

  float L_l3 = x1 + l2*sin(j2 + (PI/2 - asin(x1/l1)));

  float L =  L_l3 + l3;

  float Z =  l1*sin(PI - j1) - l2*cos(j2 + (PI/2 - asin(x1/l1)));
  
  float z = Z + h0;

  float x = sqrt(sq(L) / (sq(tan(j0)) + 1));

  Serial.print("theta0 is:");
  Serial.println(jointLinkAngles[0]);

  float y = x*tan(j0);

  float phi = j3 + j2 - j1;

  endEffectorPosOri[0] = x;
  endEffectorPosOri[1] = y;
  endEffectorPosOri[2] = z;
  endEffectorPosOri[3] = phi;

}

void calc_IK(float* PosOri) {
  // Assign known variables
  float x = PosOri[0];
  float y = PosOri[1];
  float z = PosOri[2];
  float phi = PosOri[3];
  //Serial.println(phi);
  // Serial.print("Debugging IK:");
  // for (int i = 0; i < 4; i ++) {
  //   Serial.print(PosOri[i]);
  //   Serial.print(",");
  // }
  // Serial.println();

  // Z is the distance from end effector to the first joint's horizontal line
  float Z = z - h0 ;

  float L = sqrt(sq(x) + sq(y)); // The radius from the axis of rotation of the base

  float r = atan(Z/(L-l3));

  float OB =  sqrt(sq(L-l3) + sq(Z));

  float alpha  = acos((sq(l1) - sq(l2) + sq(OB))/(2*l1*OB) );

  float theta1 = PI - r - alpha;

  float beta = acos((sq(l1) + sq(l2) - sq(OB)) / (2*l1*l2));

  float theta2 = beta - PI/2;

  float theta3 = 13*PI/180 + theta1 - theta2; // phi + theta1 - theta2

  //Plugging back into the robot joint array
  jointLinkAngles[0] = atan(y/x);
  jointLinkAngles[1] = theta1;
  jointLinkAngles[2] = theta2;
  jointLinkAngles[3] = theta3;
  

}
