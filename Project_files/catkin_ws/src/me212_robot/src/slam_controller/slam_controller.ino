// Author: Aadi Kothari aadi@mit.edu

#include "Arduino.h"
#include "helper.h"
#include <ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>


// rosrun rosserial_python serial_node.py __name:="node1" _port:=/dev/ttyACM0 _baud:=115200
#define period_us 50000 
EncoderMeasurement  encoder(26);      // FIX THIS: encoder handler class, set the motor type 53 or 26 here
RobotPose           robotPose;        // robot position and orientation calculation class
PIController        wheelVelCtrl;     // velocity PI controller class
//SerialComm          serialComm;       // serial communication class
PathPlanner         pathPlanner;      // path planner
unsigned long       prevTime = 0;

boolean usePathPlanner = true;
float x_slam = 0, y_slam = 0, Th_slam = 0;

// ROS
ros::NodeHandle node_handle;



void subscriberCallback(const geometry_msgs::PoseStamped& pose_val) {
  x_slam = pose_val.pose.position.x;
  y_slam = pose_val.pose.position.y;
  float q[4] = {pose_val.pose.orientation.x, pose_val.pose.orientation.y, pose_val.pose.orientation.z, pose_val.pose.orientation.w};

  Th_slam = quat2euler(q);
  String out = "X: "+String(x_slam)+" | Y: "+String(y_slam)+" | Th: " + String(Th_slam);
  node_handle.loginfo("Receiving pose from slam");
}

float quat2euler(float q[])
{
  double sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2]);
  double cosr_cosp = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
  float roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q[3] * q[1] - q[2] * q[0]);

  float pitch = 0;
  if (abs(sinp) >= 1)
  {
      pitch = (sinp/abs(sinp)) * M_PI / 2; // use 90 degrees if out of range
  }
  else
  {
      pitch = asin(sinp);
  }

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1]);
  double cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  float yaw = atan2(siny_cosp, cosy_cosp);

  return(yaw);
}
ros::Subscriber<geometry_msgs::PoseStamped> pose_subscriber("/orb_slam2_mono/pose", &subscriberCallback);

void setup() {
    node_handle.getHardware()->setBaud(115200);
    node_handle.initNode();
    node_handle.subscribe(pose_subscriber);
    
    encoder.init();  // connect with encoder
    wheelVelCtrl.init();        // connect with motor
    //delay(1e3);                 // delay 1000 ms so the robot doesn't drive off without you
}

void loop() {
    //timed loop implementation
    
    //node_handle.loginfo("Called sub");
    unsigned long currentTime = micros();
    
    if (currentTime - prevTime >= period_us) {
        node_handle.subscribe(pose_subscriber);
        // 1. Obtain and convert encoder measurement
        encoder.update(); 

        // 2. Compute robot odometry
        robotPose.update(x_slam, y_slam, Th_slam); 

        if(x_slam < 0.3)
        {
          node_handle.loginfo("Going straight");
          }
          else{
            node_handle.loginfo("Taking a turn");
            }
        // 3. Send robot odometry through serial port
        //serialComm.send(robotPose); 
        
        // 4. Compute desired wheel velocity without or with motion planner
        if (!usePathPlanner) {
            // 4.1 a fixed wheel speed for testing odometry
            pathPlanner.desiredWV_R = 0.2;   
            pathPlanner.desiredWV_L = 0.2;
        }
        else{
            // 4.2 compute wheel speed from using a navigation policy
            pathPlanner.navigateTrajU(robotPose); 
        }

        // 5. Command desired velocity with PI controller
        wheelVelCtrl.doPIControl("Left",  pathPlanner.desiredWV_L, encoder.v_L); 
        wheelVelCtrl.doPIControl("Right", pathPlanner.desiredWV_R, encoder.v_R);

        prevTime = currentTime; // update time
        node_handle.spinOnce();
    } 
    
}




