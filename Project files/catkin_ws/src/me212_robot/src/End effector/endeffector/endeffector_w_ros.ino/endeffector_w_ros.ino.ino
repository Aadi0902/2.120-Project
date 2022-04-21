/**********************************************************************************

***********************************************************************************/
#include <Encoder.h>



#define straight_line_up
//#define excavate
//#define dump
//#define home_pos


// pathPlanner.set_mode(1) // Set mode (Usually a ros node should do this)
// Manipulator dimensions and joint angle limits
const float l_1 = 0.09; // link1 lenth (m)
const float l_2 = 0.11185; // link2 lenth (m)
const float q1_limit =  360 * M_PI / 180; // joint angle limit for q1 (rad), M_PI is slightly more accurate and more portable then PI
const float q2_limit = 360 * M_PI / 180; // joint angle limit for q2 (rad), M_PI is slightly more accurate and more portable then PI


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
#define period_us 10000  // microseconds (1 sec = 1000000 us)

// ================================================================
// ===               SERIAL OUTPUT CONTROL                      ===
// ================================================================

// Switch between built-in Serial Plotter and Matlab streaming
// comment out both to disable serial output

#define PRINT_DATA
//#define MATLAB_SERIAL_READ

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

float C2Rad1 = 1080/(2*PI); // TO DO: replace it with your conversion factor
float C2Rad2 = 98 * 12 * 4 / (2 * PI); // TO DO: replace it with your conversion factor

Encoder Mot1(3, 5);
Encoder Mot2(2, 6);

// ================================================================
// ===               VARIABLE DEFINITION                        ===
// ================================================================

double q_1 = 0.0, pre_q_1 = 0.0; // Current and previous joint angles of the motor 1
double q_2 = 0.0, pre_q_2 = 0.0; // Current and previous joint angles of the motor 2
double error_1, sum_error_1 = 0.0, d_error_1 = 0.0, filt_d_error_1 = 0.0, error_pre_1;
double error_2, sum_error_2 = 0.0, d_error_2 = 0.0, filt_d_error_2 = 0.0, error_pre_2;



float alpha = 0.25;    // low pass filter constant
unsigned long timer;
double loop_time = 0.01;
double Pcontrol, Icontrol, Dcontrol;
double pwm;
float vc;
double set_point_1 = 0; // Set point (desired joint position) for motor 1
double set_point_2 = 0; // Set point (desired joint position) for motor 2
float x_e, y_e, theta; // end-effector position
double q_1_ik, q_2_ik; // inverse kinematics solutions
double q_1_offset = 0; // Hacky way to test individually
double q_2_offset = 0; // Hacky way to test individually
int i = 0; // To generate waypoints of a path.
int flag = 0;
float y_e_down = -0.07;
float y_e_up = 0.07;
float y_e_home = 0;
float theta_home = 0;
float theta_up = 48*PI/180;
float theta_down = -48*PI/180;
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Serial.begin(115200);

  // configure motor shield M1 and M2 outputs
  pinMode(PWM_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH);
  
  delay(500);    // delay 0.01 second

  #ifdef straight_line_up
  y_e = y_e_down;
  theta = theta_up;
  #endif

  #ifdef excavate
  y_e = y_e_down;
  theta = theta_home;
  #endif

  #ifdef dump
  y_e = y_e_up;
  theta = theta_up;
  #endif

  #ifdef home_pos
  y_e = y_e_home;
  theta = theta_home;
  #endif

  Inverse_K();
  q_1_offset = set_point_1;
  q_2_offset = set_point_2;
  
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (micros() - timer >= period_us) {

    timer = micros();

    // Joint position from the encoder counts

    q_1 = q_1_offset + Mot1.read() / C2Rad1;    // convert to radians
    q_2 = q_2_offset + Mot2.read() / C2Rad2;    // convert to radians


    // Straight line up:
    #ifdef straight_line_up
    i = i + 1;
    y_e = y_e_down + i*(y_e_up - y_e_down)/1000; // Change 1000 for tuning

    if(y_e > y_e_up || flag == 1)
    {
        y_e = y_e_up;
        i = 0;
        flag = 1;
    }
    theta = theta_up;
    #endif

    #ifdef excavate
    i = i + 1;
    theta = theta_home + i*(theta_up - theta_home)/1000; // Change 1000 for tuning

    if(theta > theta_up || flag == 1)
    {
        theta = theta_up;
        i = 0;
        flag = 1;
    }
    y_e = y_e_down;
    #endif

    #ifdef dump
    i = i + 1;
    theta = theta_up - i*(theta_up - theta_down)/1000; // Change 1000 for tuning

    if(theta < theta_down || flag == 1)
    {
        theta = theta_down;
        i = 0;
        flag = 1;
    }
    y_e = l_1*sin(asin((y_e_up-l_2*sin(theta_up))/l_1)) + l_2*sin(theta);
    #endif

    #ifdef home_pos
    i = i + 1;
    y_e = y_e_home - i*(y_e_home - y_e_down)/500; // Change 1000 for tuning

    if(y_e < y_e_down || flag == 1)
    {
        y_e = y_e_down;
        i = 0;
        flag = 1;
    }
    theta = theta_home;
    #endif
    // ================================================================
    // ===                    GET SETPOINT                          ===
    // ================================================================

    Inverse_K();

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
    motorControl(DIR_1, PWM_1, error_1, d_error_1, sum_error_1, Kp_1, Kd_1, Ki_1, pwm_max_m1,1);

    // PID controller for motor 2
    error_2 = set_point_2 - q_2;
    d_error_2 = (error_2 - error_pre_2) / loop_time;
    // 1st order filter to clean up noise
    filt_d_error_2 = alpha * d_error_2 + (1 - alpha) * filt_d_error_2;
    sum_error_2 += error_2 * loop_time;
    error_pre_2 = error_2;
    motorControl(DIR_2, PWM_2, error_2, d_error_2, sum_error_2, Kp_2, Kd_2, Ki_2, pwm_max_m2, 2);

    // ================================================================
    // ===                    PRINT DATA                            ===
    // ================================================================


#ifdef PRINT_DATA
    Serial.print("y_e theta: ");
    Serial.print(y_e);  Serial.print(", ");
    Serial.print(theta);  Serial.print("\t"); // print out  y_e, theta
    Serial.print("sp_1, q_1: ");
    Serial.print(set_point_1);  Serial.print(", ");
    Serial.print(q_1); Serial.print("\t"); // set_point_1, q_1
    Serial.print("sp_2, q_2: ");
    Serial.print(set_point_2);  Serial.print(", ");
    Serial.print(q_2); Serial.print("\t"); // set_point_2, q_2
    //  Serial.print(Mot1.read()); Serial.print("\t");
    //  Serial.print(Mot2.read()); Serial.print("\t");
    Serial.print("\n");
#endif

#ifdef MATLAB_SERIAL_READ
  Serial.print(timer / 1000000.0);   Serial.print("\t");
  Serial.print(set_point_1,4);    Serial.print("\t");
  Serial.print(q_1,4);    Serial.print("\t");
  Serial.print(set_point_2,4);    Serial.print("\t");
  Serial.print(q_2,4);    Serial.print("\t");
  Serial.print("\n");
#endif

    i++;  // change this line to i+=2; for every 2 degrees per samping period.
    //  delay(30);
  }
  loop_time = (micros() - timer) / 1000000.0;  //compute actual sample time
}


void Inverse_K()
{
  double y_e_max = l_1 + l_2 *sin(theta); // Constrain y_e so that unrealistic values are not possible
  double y_e_min = -l_1 + l_2 *sin(theta); // Constrain y_e so that unrealistic values are not possible
  if(y_e > y_e_max)
  {
    y_e = y_e_max;
  }
  else if(y_e < y_e_min)
  {
    y_e = y_e_min;
  }
  //x_e = (2*l_2*cos(theta) + sqrt(pow(2*l_2*cos(theta),2) + 8*l_2*y_e*sin(theta) - 4*l_2*l_2 + 4 * l_1*l_1 - 4 * y_e * y_e))/2; // change sign before sqrt for othe direction
  x_e = sqrt(l_1*l_1 - pow(y_e - l_2*sin(theta),2)) + l_2*cos(theta);
  q_1_ik = atan((y_e - l_2*sin(theta))/(x_e - l_2 * cos(theta)));  
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
  { if(motor==1)
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
}
