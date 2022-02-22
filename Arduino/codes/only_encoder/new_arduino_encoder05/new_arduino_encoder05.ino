// ** TEST FOR DRIVING at Various velocities **
// ** added ROS subscriber to receive cmd_vel and drive wheels
// ** added ROS publisher to publish encoder counts

#include "ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <Wire.h>

ros::NodeHandle nh;

#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/code/PIDLibrary

double Pk1 = 100;
double Ik1 = 100;
double Dk1 = 0.001;

double Setpoint1, Input1, Output1, Output1a;  // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT);  //PID Setup

double Pk2 = 100;
double Ik2 = 100;
double Dk2 = 0.001;

double Setpoint2, Input2, Output2, Output2a;  // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT);  //PID Setup

float demand1;
float demand2;
float demandx;
float demandz;

float speed_act_left;  // actual left wheel speed in m/s
float speed_act_right;  // actual right wheel speed in m/s

unsigned long currentMillis;
unsigned long previousMillis;
int loopTime = 10;

// motor

#define EN_L 6
#define IN1_L 9
#define IN2_L 8

#define EN_R 7
#define IN1_R 11
#define IN2_R 12


// wheel encoder interrupts

#define encoder0PinA 2      // encoder 1
#define encoder0PinB 3

#define encoder1PinA 18     // encoder 2
#define encoder1PinB 19

volatile long encoder0Pos = 0;     // encoder 1
volatile long encoder1Pos = 0;     // encoder 2

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;



// ** ROS callback & subscriber **

void velCallback(const geometry_msgs::Twist& vel)
{
     demandx = vel.linear.x;
     demandz = vel.angular.z;

     demandx = constrain(demandx, -0.25, 0.25);    // try to keep it under control (min speed,max speed)
     demandz = constrain(demandz, -1, 1);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);     //create a subscriber for ROS cmd_vel topic
geometry_msgs::Vector3Stamped speed_msg;                               //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                         //create a publisher to ROS topic "speed" using the "speed_msg" type


// ** setup **

void setup() {

  nh.initNode();              // int ROS
  nh.subscribe(sub);          // subscribe to cmd_vel
  nh.advertise(speed_pub);    // publish speed_pub

  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  digitalWrite(EN_L, LOW);
  digitalWrite(EN_R, LOW);
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);

  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);

  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(1, doEncoderB, CHANGE);

  attachInterrupt(4, doEncoderC, CHANGE);
  attachInterrupt(5, doEncoderD, CHANGE);

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-200, 200);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-200, 200);
  PID2.SetSampleTime(10);

  //Serial.begin(115200);
  
}




void loop() {

     nh.spinOnce();
     //delay(100);
  
     currentMillis = millis();

     if (currentMillis - previousMillis >= loopTime) {  // start timed loop for everything else
         previousMillis = currentMillis;




         // work out the two values for differential drive of each wheel

         demand1 = demandx - (demandz*0.075);
         demand2 = demandx + (demandz*0.075);


         // There are 15300 encoder counts in one metre
         // That is 153 per 10 millisecond loop at 1 m/s velocity

         // Demand in m/s but we need to convert to encoder counts per 1ms loop

         // Distance between wheels is 150mm, half of that is 75mm
         // Circumference of 150mm circle is 470mm, to turn 180* (pi radians), each wheel needs to drive half of that, which is 235mm
         // To turn one radian, each wheel needs ro drive 235/pi = 75mm (per second for 1 rad/s)
         
 

         
         // work out difference in encoder counts per loop

         encoder0Diff = encoder0Pos - encoder0Prev;  // work out difference from last time
         encoder1Diff = encoder1Pos - encoder1Prev;  // this is the current speed in counts per 10ms

         encoder0Error = (demand1*153) - encoder0Diff;
         encoder1Error = (demand2*153) - encoder1Diff;

         encoder0Prev = encoder0Pos; 
         encoder1Prev = encoder1Pos;

         Setpoint1 = demand1*153;
         Input1 = encoder0Diff;
         PID1.Compute();

         Setpoint2 = demand2*153;
         Input2 = encoder1Diff;
         PID2.Compute();


         //work out actual motor speed from wheel encoders

         speed_act_left = encoder0Diff/153;        // 1 metre/sec is 153 counts per 10ms loop
         speed_act_right = encoder1Diff/153;       // so we look at the counts per 10ms loop and divide by 153 to get the m/s




         // drive motor
         // motor1
         if (Output1 > 0) {
          Output1a = abs(Output1);
          analogWrite(EN_L, Output1a);
          digitalWrite(IN1_L, HIGH);
          digitalWrite(IN2_L, LOW);
         }
         
         else if (Output1 < 0) {
          Output1a = abs(Output1);
          analogWrite(EN_L, Output1a);
          digitalWrite(IN1_L, LOW);
          digitalWrite(IN2_L, HIGH);
         }

         else if (Output1 == 0) {
          Output1a = abs(Output1);
          analogWrite(EN_L, Output1a);
          digitalWrite(IN1_L, LOW);
          digitalWrite(IN2_L, LOW);
         }


         // motor2
         if (Output2 > 0) {
          Output2a = abs(Output2);
          analogWrite(EN_R, Output2a);
          digitalWrite(IN1_R, LOW);
          digitalWrite(IN2_R, HIGH);
         }
        
         else if (Output2 < 0) {
          Output2a = abs(Output2);
          analogWrite(EN_R, Output2a);
          digitalWrite(IN1_R, HIGH);
          digitalWrite(IN2_R, LOW);
         }
         
         else if (Output2 == 0) {
          Output2a = abs(Output2);
          analogWrite(EN_R, Output2a);
          digitalWrite(IN1_R, LOW);
          digitalWrite(IN2_R, LOW);
         }



         publishSpeed(loopTime);   //publish odometry on ROS topic

         //nh.spinOnce();

     } // end of timed loop



} // end of main loop


// ********* publish function for odometry, ***********
// ********* Uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)


void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime,should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("publishing odometry");

}









// ************* encoders interrupts ****************

// ************* encoder 1 ******************



void doEncoderA(){

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // cw
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // ccw
    }
  }
  else    // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // cw
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // ccw
    }
  }


}



void doEncoderB(){

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // cw
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // ccw
    }
  }
  else    // must be a high-to-low edge on channel B
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // cw
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // ccw
    }
  }


}


// *************** encoder 2 **********************


void doEncoderC(){

  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {
      encoder1Pos = encoder1Pos - 1;         // cw
    }
    else {
      encoder1Pos = encoder1Pos + 1;          // ccw
    }
  }
  else    // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == HIGH) {
      encoder1Pos = encoder1Pos - 1;          // cw
    }
    else {
      encoder1Pos = encoder1Pos + 1;          // ccw
    }
  }


}



void doEncoderD(){

  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinA) == HIGH) {
      encoder1Pos = encoder1Pos - 1;         // cw
    }
    else {
      encoder1Pos = encoder1Pos + 1;          // ccw
    }
  }
  else    // must be a high-to-low edge on channel B
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinA) == LOW) {
      encoder1Pos = encoder1Pos - 1;          // cw
    }
    else {
      encoder1Pos = encoder1Pos + 1;          // ccw
    }
  }


}
