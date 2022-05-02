// Aadi KOthari - aadi@mit.edu
#include "Arduino.h"
#include "helper.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>

// rosrun rosserial_python serial_node.py __name:="node1" _port:=/dev/ttyACM0 _baud:=115200

EncoderMeasurement  encoder(26);      // FIX THIS: encoder handler class, set the motor type 53 or 26 here
RobotPose           robotPose;        // robot position and orientation calculation class
PIController        wheelVelCtrl;     // velocity PI controller class
PathPlanner         pathPlanner;      // path planner
unsigned long       prevTime = 0;
unsigned long       prev_task_time;
boolean usePathPlanner = true;

float mode = 5;
int flag = 0;
float task_time_delay = 1;

float y_e_up = 0.06;
float y_e_down = -0.065;
float y_e_home = 0;
float y_e_carry = -0.02;
float theta_up = 0;
float theta_home = -48.01 * PI/180;
float theta_down = 2*-48.01*PI/180;

// ROS
ros::NodeHandle node_handle;

void subscriberCallback(const std_msgs::Float64MultiArray& vel_curv) {
  float vel = vel_curv.data[0];
  float k = vel_curv.data[1];
  pathPlanner.updateDesiredV(0, 0);
  node_handle.loginfo("Received value");
}
ros::Subscriber<std_msgs::Float64MultiArray> vel_curv_subscriber("vel_curvature", &subscriberCallback);

std_msgs::Float32 path_dist;
ros::Publisher path_dist_pub("path_distance", &path_dist);


void setup() {

    node_handle.getHardware()->setBaud(115200);
    node_handle.initNode(); // Initialize node
    node_handle.advertise(path_dist_pub);
    node_handle.subscribe(vel_curv_subscriber);
    
    encoder.init();  // connect with encoder
    wheelVelCtrl.init();        // connect with motor
    prev_task_time = micros();
    delay(1000);                 // delay 1000 ms so the robot doesn't drive off without you
}

void loop() {
    //timed loop implementation
    node_handle.subscribe(vel_curv_subscriber);
    unsigned long currentTime = micros();
    if (currentTime - prevTime >= PERIOD_MICROS){
        float robotVel = 0, K = 0;
        // 1. Obtain and convert encoder measurement
        encoder.update(); 

        // 2. Compute robot odometry
        robotPose.update(encoder.dPhiL, encoder.dPhiR); 

        path_dist.data = robotPose.pathDistance;
        
        
        // 4. Compute desired wheel velocity without or with motion planner
        if (!usePathPlanner) {
            // 4.1 a fixed wheel speed for testing odometry
            pathPlanner.desiredWV_R = 0.2;   
            pathPlanner.desiredWV_L = 0.2;
        }
        else{
            // 4.2 compute wheel speed from using a navigation policy
            //pathPlanner.navigateTrajU(robotPose);           
            
        }
        
        // 5. Command desired velocity with PI controller
        wheelVelCtrl.doPIControl("Left",  pathPlanner.desiredWV_L, encoder.v_L); 
        wheelVelCtrl.doPIControl("Right", pathPlanner.desiredWV_R, encoder.v_R);

        prevTime = currentTime; // update time 
        
        path_dist_pub.publish(&path_dist);      
        node_handle.spinOnce(); 
    }
    delay(3);
}


