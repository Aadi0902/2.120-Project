// Author: Aadi Kothari aadi@mit.edu

#include "Arduino.h"
#include "helper.h"
#include <ros.h>
#include <geometry_msgs/Pose2D.h>



// rosrun rosserial_python serial_node.py __name:="node1" _port:=/dev/ttyACM0 _baud:=115200
//#define period_us 50000 
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

int flag = 0;

void subscriberCallback(const geometry_msgs::Pose2D& pose_val) {
  x_slam = pose_val.x;
  y_slam = pose_val.y;
  Th_slam = pose_val.theta;//quat2euler(q);
  if(flag == 0)
  {
    flag = 1;
    robotPose.X = x_slam;
    robotPose.Y = y_slam;
  }
  //node_handle.loginfo("Receiving pose from slam");
}

ros::Subscriber<geometry_msgs::Pose2D> pose_subscriber("robot_pose2d", &subscriberCallback); //"/orb_slam2_mono/pose"

void setup() {
    node_handle.getHardware()->setBaud(115200);
    node_handle.initNode();
    node_handle.subscribe(pose_subscriber);
    
    encoder.init();  // connect with encoder
    wheelVelCtrl.init();        // connect with motor

    delay(1e3);                 // delay 1000 ms so the robot doesn't drive off without you
}

void loop() {
    //timed loop implementation
    
    //node_handle.loginfo("Called sub");
    unsigned long currentTime = micros();
    node_handle.subscribe(pose_subscriber);
    if (currentTime - prevTime >= PERIOD_MICROS) {
        // 1. Obtain and convert encoder measurement
        encoder.update(); 

        // 2. Compute robot odometry
        //robotPose.update(encoder.dPhiL, encoder.dPhiR); 
        robotPose.update(x_slam, y_slam, Th_slam);//, node_handle); 
        
        // 4. Compute desired wheel velocity without or with motion planner
        if (!usePathPlanner) {
            // 4.1 a fixed wheel speed for testing odometry
            pathPlanner.desiredWV_R = 0.2;   
            pathPlanner.desiredWV_L = 0.2;
        }
        else{
            // 4.2 compute wheel speed from using a navigation policy
            pathPlanner.navigateTrajU(robotPose);
            
//            float  val =  robotPose.pathDistance;  // test value
//            char sz[20] = {' '} ;
//            int val_int = (int) val;   // compute the integer part of the float
//            float val_float = (abs(val) - abs(val_int)) * 100000;
//            int val_fra = (int)val_float;
//            sprintf (sz, "%d.%05d", val_int, val_fra); //
//            node_handle.loginfo(sz);
            
        }
        // 5. Command desired velocity with PI controller
        wheelVelCtrl.doPIControl("Left",  pathPlanner.desiredWV_L, encoder.v_L); 
        wheelVelCtrl.doPIControl("Right", pathPlanner.desiredWV_R, encoder.v_R);

        prevTime = currentTime; // update time
        
    }
    node_handle.spinOnce(); 
    
}





