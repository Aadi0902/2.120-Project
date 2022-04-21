//#ifndef ArduinoLab2Helper_h
//#define ArduinoLab2Helper_h

//#include "Arduino.h"
//#include "DualMC33926MotorShield.h"
//
//#include "LS7366.h"





class PathPlanner {
  public:
    float current_mode = 1;
    float prev_mode = 1;

    float task_time = 0.1; // Time to finish the task

    float lower_y_e = -0.1;
    float upper_y_e = 0.2;
    float theta_up = 48.01;
    float theta_home = 0;
    float theta_down = -48.01;

    float desired_y_e = lower_y_e;
    float desired_theta = theta_home;

    float current_phi1;
    float current_phi2;


    
    PathPlanner(): current_mode(1) {}

    void setCurrentPos(float cur_q1, float cur_q2);
    void set_mode(float mode);
    
  private: 
    unsigned long prevSerialTime;
};
