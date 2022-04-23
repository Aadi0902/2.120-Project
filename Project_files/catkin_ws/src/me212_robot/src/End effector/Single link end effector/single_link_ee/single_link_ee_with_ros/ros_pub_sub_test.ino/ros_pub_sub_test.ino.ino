/**********************************************************************************

***********************************************************************************/
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <Encoder.h>
#include "Endeffector_helper.h"


// ================================================================
// ===               DFROBOT MOTOR SHIELD DEFINITION            ===
// ================================================================

const int
// motor connected to M1
PWM_1   = 9,
DIR_1   = 7;
const int
// motor connected to M2
PWM_2   = 10,
DIR_2   = 8;
int M1FB = A0; // To read current in milliamps
int M2FB = A1; // To read current in milliamps
int _nD2 = 4;
int _nSF = 12;
// ================================================================
// ===               ENCODER DEFINITION                         ===
// ================================================================

// The motor has a dual channel encoder and each has 120 counts per revolution
// The effective resolution is 480 CPR with quadrature decoding.
// Calibrate the conversion factor by hand.

float C2Rad1 = 1080/(2*PI);
float C2Rad2 = 98 * 12 * 4 / (2 * PI);

Encoder Mot1(3, 5);
Encoder Mot2(2, 6);

double q_1 = 0.0, pre_q_1 = 0.0; // Current and previous joint angles of the motor 1
double q_2 = 0.0, pre_q_2 = 0.0; // Current and previous joint angles of the motor 2
double error_1, sum_error_1 = 0.0, d_error_1 = 0.0, filt_d_error_1 = 0.0, error_pre_1;
double error_2, sum_error_2 = 0.0, d_error_2 = 0.0, filt_d_error_2 = 0.0, error_pre_2;

const float l_1 = 0.09; // link1 lenth (m)
const float q1_limit =  0.89; // joint angle limit for q1 (rad), M_PI is slightly more accurate and more portable then PI
const float q2_limit = 150 * M_PI / 180; // joint angle limit for q2 (rad), M_PI is slightly more accurate and more portable then PI


float         alpha = 0.25;    // low pass filter constant
unsigned long timer;
double        loop_time = 0.01;
double        Pcontrol, Icontrol, Dcontrol;
double        pwm;
float         vc;
double        set_point_1 = 0; // Set point (desired joint position) for motor 1
double        set_point_2 = 0; // Set point (desired joint position) for motor 2
double         x_e, y_e, theta; // end-effector position
float         in_y_e = 0;
float         in_theta = 0;
double        q_1_ik, q_2_ik; // inverse kinematics solutions
int           i = 0; // To generate waypoints of a path.
int           prev_mode = -1;
bool          reached = false;
PathPlanner ee_planner;      // path planner


#define period_us 20000 // 20ms (0.02 second, 50Hz), 1000000 microseconds = 1 second

// ================================================================
// ===               ROS                   ===
// ================================================================
ros::NodeHandle node_handle;
//std_msgs::UInt16 ee_mode;
geometry_msgs::Pose2D set_ee_pose;
std_msgs::Bool pos_reached;
std_msgs::Float64MultiArray enc_values;
// ee_mode = [current_mode desired_y_e desired_theta task_time]


void subscriberCallback(const std_msgs::Float64MultiArray& ee_mode) {
   ee_planner.current_mode = ee_mode.data[0];
   ee_planner.desired_y_e = ee_mode.data[1];
   ee_planner.desired_theta = ee_mode.data[2];
   ee_planner.task_time = ee_mode.data[3];
}

ros::Subscriber<std_msgs::Float64MultiArray> ee_mode_subscriber("ee_mode", &subscriberCallback); // "ee_mode" is the topic name, &subscriberCallback is the subscriber call back function
ros::Publisher ee_pose_publisher("set_ee_pose", &set_ee_pose); // "Set_EE_pose" is the name of the topic to publish, set_ee_pose is the variable type
ros::Publisher pos_reached_publisher("reached_pos", &pos_reached);
ros::Publisher enc_publisher("Enocoder_values", &enc_values);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  // configure motor shield M1 and M2 outputs
  pinMode(PWM_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH);
  
  node_handle.initNode(); // Initialize node
  node_handle.advertise(ee_pose_publisher); // Advertise publisher
  node_handle.advertise(pos_reached_publisher);
  node_handle.advertise(enc_publisher);
  node_handle.subscribe(ee_mode_subscriber); // Subscribe to the subscriber
  delay(500);    // delay 0.5 second
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (micros() - timer >= period_us) {

    timer = micros();

    // Joint position from the encoder counts

    q_1 = Mot1.read() / C2Rad1;    // convert to radians
    q_2 = Mot2.read() / C2Rad2;    // convert to radians

    enc_values.data_length = 2;
    enc_values.data[0] = q_1;
    enc_values.data[1] = q_2;
    ee_planner.setCurrentPos(q_1, q_2);

    if(ee_planner.current_mode != prev_mode)
    {
      in_y_e = l_1 * sin(q_1);
      in_theta = q_1 + q_2;
      prev_mode = ee_planner.current_mode;
      reached = false;
      i = 0;
    }
    
    // ================================================================
    // ===                    GET SETPOINT                          ===
    // ================================================================

    if(ee_planner.current_mode == 1) // Home postion - excavation position
    {
      ee_planner.lower_y_e = ee_planner.desired_y_e; // Update lower limit of y_e
      ee_planner.theta_home = ee_planner.desired_theta; // Update home value of theta
      
      i = i + 1;
      y_e = in_y_e - i*(in_y_e - ee_planner.lower_y_e)/(ee_planner.task_time/(period_us/1000000.0)); // Change denominator for tuning
      
      if(y_e < ee_planner.lower_y_e || reached)
      {
        y_e = ee_planner.lower_y_e;
        i = 0;
        reached = true;
      }
      theta = ee_planner.desired_theta;
      enc_values.data[0] = 100;
      enc_values.data[1] = 100;
    }
    else if(ee_planner.current_mode == 2) // Excavate
    {
      ee_planner.theta_up = ee_planner.desired_theta; // Update upper limit of theta
      ee_planner.lower_y_e = ee_planner.desired_y_e; // Update lower limit of y_e
      
      theta = in_theta + i*(ee_planner.theta_up - in_theta)/(ee_planner.task_time/(period_us/1000000.0));
      if(theta > ee_planner.theta_up || reached)
      {
         theta = ee_planner.theta_up;
         i = 0;
         reached = true;
      }
      y_e = ee_planner.desired_y_e;
      enc_values.data[0] = 200;
      enc_values.data[1] = 200;  
    }
    else if(ee_planner.current_mode == 3) // Straight line up
    {
      ee_planner.upper_y_e = ee_planner.desired_y_e; // Update upper limit of y_e
      ee_planner.theta_up = ee_planner.desired_theta; // Update upper limit of theta
      
      i = i + 1;
      y_e = in_y_e + i*(ee_planner.upper_y_e - in_y_e)/(ee_planner.task_time/(period_us/1000000.0)); // Change denominator for tuning

      if(y_e > ee_planner.upper_y_e || reached)
      {
        y_e = ee_planner.upper_y_e;
        i = 0;
        reached = true;
      }
      theta = ee_planner.desired_theta;
      enc_values.data[0] = 300;
      enc_values.data[1] = 300;
    }
    else if (ee_planner.current_mode == 4) // Dump
    {
      ee_planner.theta_down = ee_planner.desired_theta; // Update lower limit of theta
      ee_planner.upper_y_e = ee_planner.desired_y_e; // Update upper limit of y_e

      theta = in_theta - i*(in_theta - ee_planner.theta_down)/(ee_planner.task_time/(period_us/1000000.0)); // Change denominator for tuning

      if(theta < ee_planner.theta_down || reached)
      {
         theta = ee_planner.theta_down;
         i = 0;
         reached = true;
      }
      y_e = ee_planner.desired_y_e;
      enc_values.data[0] = 400;
      enc_values.data[1] = 400;   
    }
    else
    {
      y_e = in_y_e; // Define value for testing
      theta = in_theta; // Define value for testing
      reached = true;
      enc_values.data[0] = 500;
      enc_values.data[1] = 500;
    }

    pos_reached.data = reached;
    Inverse_K();

    set_ee_pose.x = x_e;
    set_ee_pose.y = y_e;
    set_ee_pose.theta = theta;
    ee_pose_publisher.publish(&set_ee_pose); // Publish message
    pos_reached_publisher.publish(&pos_reached);
    enc_publisher.publish(&enc_values);
    node_handle.spinOnce();


    i++;  // change this line to i+=2; for every 2 degrees per samping period.
    //  delay(30);
  }
  loop_time = (micros() - timer) / 1000000.0;  //compute actual sample time
}


void Inverse_K()
{
  double y_e_max = l_1; // Constrain y_e so that unrealistic values are not possible
  double y_e_min = -l_1; // Constrain y_e so that unrealistic values are not possible
  if(y_e > y_e_max)
  {
    y_e = y_e_max;
  }
  else if(y_e < y_e_min)
  {
    y_e = y_e_min;
  }
  x_e = l_1*cos(atan(y_e/sqrt(l_1*l_1 - y_e*y_e)));
  q_1_ik = atan(y_e/x_e);  
  q_2_ik = theta - q_1_ik; 

  // position limit constraints (update set_point_1 and set_point_2 when they are within the limits)
  if (abs(set_point_1) < q1_limit && abs(set_point_2) < q2_limit)
  {
    set_point_1 = q_1_ik;
    set_point_2 = q_2_ik;
  }
  else
  {
    Serial.print("Joint limit reached!");
  }
}
