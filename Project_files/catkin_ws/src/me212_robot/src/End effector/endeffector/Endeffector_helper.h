//#ifndef ArduinoLab2Helper_h
//#define ArduinoLab2Helper_h

//#include "Arduino.h"
//#include "DualMC33926MotorShield.h"
//
//#include "LS7366.h"





class PathPlanner {
  public:
    float current_mode = 1;

    float lower_ye = -0.1;
    float upper_ye = 0.2;
    float theta_carry = 48.01;
    float theta_home = 0;
    float theta_dump = -48.01;

    float desired_y_e = lower_ye;
    float desired_theta = theta_home;

    float current_phi1;
    float current_phi2;


    
    PathPlanner(): current_mode(1) {}

    void setCurrentPos(float cur_q1, float cur_q2);
    void set_mode(float mode);
    
  private: 
    unsigned long prevSerialTime;
};
