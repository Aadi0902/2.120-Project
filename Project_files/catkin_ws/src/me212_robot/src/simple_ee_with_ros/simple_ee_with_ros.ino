/**********************************************************************************

***********************************************************************************/
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>



// rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
// ================================================================
// ===               PID FEEDBACK CONTROLLER                    ===
// ================================================================

// Provide PID controller gains here (use the gains from Lab 5 as your starting point

// Motor 1 requires higher gains due to larger moment of inertia
// Try 1.5 - 2.5 times the Motor 2 gains
float Kp_1 = 150.0;
float Kd_1 = 0.0;
float Ki_1 = 10.0;

// Motor 2 requires smaller gains compared to Motor 1
// Try 3 - 5 times the gains from previous lab
float Kp_2 = 100.0;
float Kd_2 = 0.0;
float Ki_2 = 10.0;

int pwm_max_m1 = 125;
int pwm_max_m2 = 255;

// control sampling period
#define period_us 50000  // 20Hz, microseconds (1 sec = 1000000 us)


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

// ================================================================
// ===               VARIABLE DEFINITION                        ===
// ================================================================

float q_1 = 0.0, pre_q_1 = 0.0; // Current and previous joint angles of the motor 1
float q_2 = 0.0, pre_q_2 = 0.0; // Current and previous joint angles of the motor 2
float current_1 = 0.0, current_2 = 0.0;
double error_1, sum_error_1 = 0.0, d_error_1 = 0.0, filt_d_error_1 = 0.0, error_pre_1;
double error_2, sum_error_2 = 0.0, d_error_2 = 0.0, filt_d_error_2 = 0.0, error_pre_2;



float         alpha = 0.25;    // low pass filter constant
unsigned long timer;
double        loop_time = 0.01;
double        Pcontrol, Icontrol, Dcontrol;
double        pwm;
float         vc;
double        set_point_1 = 0; // Set point (desired joint position) for motor 1
double        set_point_2 = 0; // Set point (desired joint position) for motor 2

void(* resetFunc) (void) = 0;

// ================================================================
// ===               ROS                   ===
// ================================================================
ros::NodeHandle node_handle;

std_msgs::Float64MultiArray enc_current_values;
void subscriberCallback(const std_msgs::Float64MultiArray& set_point_val) {
   set_point_1 = set_point_val.data[0];
   set_point_2 = set_point_val.data[1];
}

ros::Subscriber<std_msgs::Float64MultiArray> set_point_subscriber("set_point_topic", &subscriberCallback); // "set_point_topic" is the topic name, &subscriberCallback is the subscriber call back function
ros::Publisher enc_current_publisher("encoder_current_val", &enc_current_values);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {


  // configure motor shield M1 and M2 outputs
  pinMode(PWM_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(DIR_2, OUTPUT);

  pinMode(M1FB,INPUT);
  pinMode(M2FB,INPUT);
  
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH);
  pinMode(_nSF,INPUT);

  node_handle.getHardware()->setBaud(115200);
  node_handle.initNode(); // Initialize node
  node_handle.advertise(enc_current_publisher);
  node_handle.subscribe(set_point_subscriber); // Subscribe to the subscriber
  delay(10);    // delay 0.1 second
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  node_handle.subscribe(set_point_subscriber);
  if (micros() - timer >= period_us) {
    timer = micros();

    // Joint position from the encoder counts

    q_1 = Mot1.read() / C2Rad1;    // convert to radians
    q_2 = Mot2.read() / C2Rad2;    // convert to radians

    current_1 = analogRead(M1FB)*9 / 1000; // Amps
    current_2 = analogRead(M2FB)*9 / 1000; // Amps   

    //float enc_current_val[4] = {q_1, q_2, set_point_1, set_point_2};
    float enc_current_val[4] = {q_1, q_2, current_1, current_2};
    enc_current_values.data = enc_current_val;
    enc_current_values.data_length = 4;

    // ================================================================
    // ===                    CONTROLLER CODE                       ===
    // ================================================================

    // PID controller for motor 1
    error_1 = set_point_1 - q_1;
    d_error_1 = (error_1 - error_pre_1) / loop_time;
    // 1st order filter to clean up noise
    filt_d_error_1 = alpha * d_error_1 + (1 - alpha) * filt_d_error_1;
    sum_error_1 += error_1 * loop_time;
    error_pre_1 = error_1;
    motorControl(DIR_1, PWM_1, error_1, d_error_1, sum_error_1, Kp_1, Kd_1, Ki_1, pwm_max_m1, 1);

    // PID controller for motor 2
    error_2 = set_point_2 - q_2;
    d_error_2 = (error_2 - error_pre_2) / loop_time;
    // 1st order filter to clean up noise
    filt_d_error_2 = alpha * d_error_2 + (1 - alpha) * filt_d_error_2;
    sum_error_2 += error_2 * loop_time;
    error_pre_2 = error_2;
    motorControl(DIR_2, PWM_2, error_2, d_error_2, sum_error_2, Kp_2, Kd_2, Ki_2, pwm_max_m2, 2);
    
    //  delay(30);
    
    
    enc_current_publisher.publish(&enc_current_values);
    //node_handle.spinOnce();
  }
  loop_time = (micros() - timer) / 1000000.0;  //compute actual sample time
  node_handle.spinOnce();
}


// ================================================================
// ===                   MOTOR CONTROLLER                       ===
// ================================================================

void motorControl(int DIR_x, int PWM_x, float error, float d_error, float sum_error, float Kp_x, float Kd_x, float Ki_x, int pwm_max, int motor)
{
  float pwm_command;

  Pcontrol = error * Kp_x;
  Icontrol = sum_error * Ki_x;
  Dcontrol = d_error * Kd_x;

  Icontrol = constrain(Icontrol, -1*pwm_max, pwm_max);  // I control saturation limits for anti-windup

  pwm_command = Pcontrol + Icontrol + Dcontrol;

  if (pwm_command > 0)
  { 
    if(motor==1)
    {
      digitalWrite(DIR_x, HIGH);
    }
    else
    {
      digitalWrite(DIR_x, LOW);
    }
    analogWrite(PWM_x, (int) constrain(pwm_command, 0, pwm_max));
    
  }
  else
  {
    if(motor==1)
    {
      digitalWrite(DIR_x, LOW);
    }
    else
    {
      digitalWrite(DIR_x, HIGH);
    }
    analogWrite(PWM_x, (int) constrain(abs(pwm_command), 0, pwm_max));
  }
  //delay(2);
}
