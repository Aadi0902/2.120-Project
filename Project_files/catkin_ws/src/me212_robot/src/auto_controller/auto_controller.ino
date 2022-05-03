  // Aadi KOthari - aadi@mit.edu
#include "Arduino.h"
#include "helper.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>

// rosrun rosserial_python serial_node.py __name:="node1" _port:=/dev/ttyACM0 _baud:=115200

EncoderMeasurement  encoder(26);      // FIX THIS: encoder handler class, set the motor type 53 or 26 here
RobotPose           robotPose;        // robot position and orientation calculation class
PIController        wheelVelCtrl;     // velocity PI controller class
PathPlanner         pathPlanner;      // path planner
unsigned long       prevTime = 0;
unsigned long       prev_task_time;
boolean usePathPlanner = true;
bool use_manual_contr = false;

float VL = 0, VR = 0;

char c;
void drive_forwards();
void drive_backwards();
void drive_ccw();
void drive_cw();
void stall();

// ROS
ros::NodeHandle node_handle;

void vel_curv_callback(const std_msgs::Float32MultiArray& vel_curv) {
  float vel = vel_curv.data[0];
  float k = vel_curv.data[1];
  if(!use_manual_contr)
  {
    pathPlanner.updateDesiredV(vel, k);
    node_handle.loginfo("Received vel, k values");
  }
}

void manual_auto_callback(const std_msgs::Bool& manual_contr) {
  use_manual_contr = manual_contr.data;
  if(use_manual_contr)
  {
    node_handle.loginfo("Manual control");
  }
  else
  {
      node_handle.loginfo("Autonomous control");
  }
}
void manual_callback(const std_msgs::Char& inp) {
  c = inp.data;
  node_handle.loginfo("Received keyboard input");
}

ros::Subscriber<std_msgs::Char> char_subscriber("manual_inp", &manual_callback);
ros::Subscriber<std_msgs::Float32MultiArray> vel_curv_subscriber("vel_curvature", &vel_curv_callback);
ros::Subscriber<std_msgs::Bool> manual_auto_sub("manual_control", &manual_auto_callback);


std_msgs::Float32 path_dist;
ros::Publisher path_dist_pub("path_distance", &path_dist);


void setup() {
    node_handle.getHardware()->setBaud(115200);
    node_handle.initNode(); // Initialize node
    node_handle.advertise(path_dist_pub);
    node_handle.subscribe(vel_curv_subscriber);
    node_handle.subscribe(manual_auto_sub);
    node_handle.subscribe(char_subscriber);
    path_dist.data = 0;
    
    encoder.init();  // connect with encoder
    wheelVelCtrl.init();        // connect with motor
    prev_task_time = micros();
    delay(1000);                 // delay 1000 ms so the robot doesn't drive off without you
}

void loop() {
    //timed loop implementation
    node_handle.subscribe(vel_curv_subscriber);
    node_handle.subscribe(manual_auto_sub);
    node_handle.subscribe(char_subscriber);

    unsigned long currentTime = micros();
    if (currentTime - prevTime >= PERIOD_MICROS){
        float robotVel = 0, K = 0;
        // 1. Obtain and convert encoder measurement
        encoder.update(); 

        if(use_manual_contr)
        {
            if (c == 'w'){
              drive_forwards();
              node_handle.loginfo("Forwards");
            } else if (c == 's'){
              drive_backwards();
              node_handle.loginfo("Backwards");
            } else if (c == 'a'){
              drive_ccw();
              node_handle.loginfo("Left");
            } else if (c == 'd'){
              drive_cw(); 
              node_handle.loginfo("Right");
            } else if (c == ' '){
              stall();
              node_handle.loginfo("Stopp");
            }
          pathPlanner.desiredWV_R = VR;   
          pathPlanner.desiredWV_L = VL;
        }
        else
        {
            // 2. Compute robot odometry
            robotPose.update(encoder.dPhiL, encoder.dPhiR); 
        }
        path_dist.data = robotPose.pathDistance;
        // 5. Command desired velocity with PI controller
        wheelVelCtrl.doPIControl("Left",  pathPlanner.desiredWV_L, encoder.v_L); 
        wheelVelCtrl.doPIControl("Right", pathPlanner.desiredWV_R, encoder.v_R);

        prevTime = currentTime; // update time 
        
        path_dist_pub.publish(&path_dist);      
    }
    node_handle.spinOnce(); 
    delay(3);
}

void drive_forwards() {
  VL = 0.3;
  VR = 0.3;
}

void drive_backwards() {
  VL = -0.3;
  VR = -0.3;
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
