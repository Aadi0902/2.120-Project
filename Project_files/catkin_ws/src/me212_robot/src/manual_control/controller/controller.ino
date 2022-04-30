
// Zack Bright        - zbright  _ mit _ edu,    Sept 2015
// Daniel J. Gonzalez - dgonz    _ mit _ edu,    Sept 2015
// Fangzhou Xia       - xiafz    _ mit _ edu,    Sept 2015
// Peter KT Yu        - peterkty _ mit _ edu,    Sept 2016
// Ryan Fish          - fishr    _ mit _ edu,    Sept 2016
// Jerry Ng           - jerryng  _ mit _ edu,    Feb  2019

// rosrun rosserial_python serial_node.py __name:="node1" _port:=/dev/ttyACM4 _baud:=115200

#include "Arduino.h"
#include "helper.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

EncoderMeasurement  encoder(26);      // FIX THIS: encoder handler class, set the motor type 53 or 26 here
RobotPose           robotPose;        // robot position and orientation calculation class
PIController        wheelVelCtrl;     // velocity PI controller class
//SerialComm          serialComm;       // serial communication class
PathPlanner         pathPlanner;      // path planner
unsigned long       prevTime = 0;

char c[5];
ros::NodeHandle node_handle;

float VL = 0;
float VR = 0;

boolean usePathPlanner = false;

// ROS
void subscriberCallback(const std_msgs::String& inp) {
  String tempStr = inp.data;
  tempStr.toCharArray(c,5);
  node_handle.loginfo("Received value");
}
ros::Subscriber<std_msgs::String> char_subscriber("manual_inp", &subscriberCallback);

void drive_forwards();
void drive_backwards();
void drive_ccw();
void drive_cw();
void stall();

void setup() {
    node_handle.getHardware()->setBaud(115200);
    encoder.init();  // connect with encoder
    wheelVelCtrl.init();        // connect with motor
    node_handle.initNode();
    node_handle.subscribe(char_subscriber);
    
    delay(1e3);                 // delay 1000 ms so the robot doesn't drive off without you
}

void loop() {
    //timed loop implementation
    node_handle.subscribe(char_subscriber);
    unsigned long currentTime = micros();
    
    if (currentTime - prevTime >= PERIOD_MICROS) {
      
        // 1. Obtain and convert encoder measurement
        encoder.update(); 

        // 2. Compute robot odometry
        robotPose.update(encoder.dPhiL, encoder.dPhiR); 

        // 3. Send robot odometry through serial port
        //serialComm.send(robotPose); 


        // 4. Compute desired wheel velocity without or with motion planner
        if (!usePathPlanner) {
        // 4.1 wheel speed depends on keys pressed
            if (c[0] == 'w'){
              drive_forwards();
              node_handle.loginfo("Forwards");
            } else if (c[0] == 's'){
              drive_backwards();
              node_handle.loginfo("Backwards");
            } else if (c[0] == 'a'){
              drive_ccw();
              node_handle.loginfo("Left");
            } else if (c[0] == 'd'){
              drive_cw(); 
              node_handle.loginfo("Right");
            } else if (c[0] == ' '){
              stall();
              node_handle.loginfo("Stopp");
            }
          pathPlanner.desiredWV_R = VR;   
          pathPlanner.desiredWV_L = VL;
        }
        else{
            // 4.2 compute wheel speed from using a navigation policy
            pathPlanner.navigateTrajU(robotPose); 
        }

        // 5. Command desired velocity with PI controller
        wheelVelCtrl.doPIControl("Left",  pathPlanner.desiredWV_L, encoder.v_L); 
        wheelVelCtrl.doPIControl("Right", pathPlanner.desiredWV_R, encoder.v_R);

        prevTime = currentTime; // update time
    } 
    node_handle.spinOnce();
}

void drive_forwards() {
  VL = 1;
  VR = 1;
}

void drive_backwards() {
  VL = -1;
  VR = -1;
}

void drive_cw() {
  VL = 0.3;
  VR = -0.3;
}

void drive_ccw() {
  VL = -0.3;
  VR = 0.3;
}

void stall(){
  //Setting both wheel velocities to zero, at stopped position
  VL = 0;
  VR = 0; 
}
