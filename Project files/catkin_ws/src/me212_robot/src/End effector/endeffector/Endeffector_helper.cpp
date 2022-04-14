#include "Endeffector_helper.h"



//PathPlanner Class function implementation
void PathPlanner::setCurrentPos(float cur_q1, float cur_q2) {
    current_phi1 = cur_q1;
    current_phi2 = cur_q2;
}

void PathPlanner::set_mode(float mode) {
    current_mode = mode;
    if(mode == 1) // Home position
    {
        desired_y_e = lower_ye;
        desired_theta = theta_home;
    }
    else if(mode == 2) // Scoop and effector parallel to the ground
    {
        desired_y_e = lower_ye;
        desired_theta = theta_carry;
    }
    else if (mode == 3) // Straight line up
    {
        desired_y_e = upper_ye;
        desired_theta = theta_carry;
    }
    else if(mode == 4) // Dump
    {
        desired_y_e = upper_ye;
        desired_theta = theta_dump;
    }
    else // Home
    {
        desired_y_e = lower_ye;
        desired_theta = theta_home;
    }
}
