// Zack Bright        - zbright  _ mit _ edu,    Sept 2015
// Daniel J. Gonzalez - dgonz    _ mit _ edu,    Sept 2015
// Fangzhou Xia       - xiafz    _ mit _ edu,    Sept 2015
// Peter KT Yu        - peterkty _ mit _ edu,    Sept 2016
// Ryan Fish          - fishr    _ mit _ edu,    Sept 2016
// Jerry Ng           - jerryng  _ mit _ edu,    Feb 2019

#include "helper.h"

//Encoder Measurement Class function implementation
EncoderMeasurement::EncoderMeasurement(int motor_type): 
    encoder1CountPrev(0),
    encoder2CountPrev(0),
    v_R(0), v_L(0)
{
    float rev2enc, gearing;
    if(motor_type == 53){
        rev2enc = rev2enc_53;
        gearing = gearing_53;
    }
    else if (motor_type == 26){
        rev2enc = rev2enc_26;
        gearing = gearing_26;
    }
    else;
        //Serial.println("ERROR: BAD MOTOR TYPE INPUT, SHOULD BE 26 or 53");
        
    enc2rev = 1.0 / rev2enc;
    enc2rad = enc2rev * 2 * PI;
    enc2wheel = enc2rad * wheelRadius; 
    
    maxMV = voltage * motor_const / gearing * wheelRadius;  // max wheel speed (m/2)
}

void EncoderMeasurement::update() {
    float encoder1Count = readEncoder(1);
    float encoder2Count = -1 * readEncoder(2);
    float dEncoder1 = (encoder1Count - encoder1CountPrev);
    float dEncoder2 = (encoder2Count - encoder2CountPrev);
    
    //update the angle incremet in radians
    float dphi1 = (dEncoder1 * enc2rad);
    float dphi2 = (dEncoder2 * enc2rad);
    
    //for encoder index and motor position switching (Right is 1, Left is 2)
    dPhiR = dphi1;
    dPhiL = dphi2;
    
    float dWheel1 = (dEncoder1 * enc2wheel);
    float dWheel2 = (dEncoder2 * enc2wheel);
    
    //wheel velocity (Right is 1, Left is 2)
    float mV1 = dWheel1 / PERIOD;
    float mV2 = dWheel2 / PERIOD;
    v_R = mV1;
    v_L = mV2;
    encoder1CountPrev = encoder1Count;
    encoder2CountPrev = encoder2Count;
}

//RobotPose Class function implementation
void RobotPose::update(float dThetaL, float dThetaR) {
    // orientation angle theta increment in radians
    float dTh = (r / (2.0 * b)) * (dThetaR - dThetaL);
    
    Th += dTh;
    
    // robot X, Y position increment in meters
    float dX = (r / 2.0) * cos(Th) * (dThetaR + dThetaL); 
    float dY = (r / 2.0) * sin(Th) * (dThetaR + dThetaL);
    
    X += dX;
    Y += dY;
    
    pathDistance += sqrt(dX * dX + dY * dY);
}

//PathPlanner Class function implementation
void PathPlanner::navigateTrajU(const RobotPose & robotPose) {
    float f2m = 0.3048; // feet to m conversion
    float in_bias = 2*1.414*f2m;
    float rad_turn = 0.25;

    float d1 = 1;
    float d2 = rad_turn*PI/2;
    float d3 = 1 - rad_turn;
    float d4 = 1;
    float d5 = rad_turn*PI/2;
    float d6 = 1;
    float d7 = 0.2;
    float d8 = 0.4;
    
    // Straight line forward
    if (robotPose.pathDistance < d1) { 
        float robotVel = 0.2, K = 0;
        updateDesiredV(robotVel, K);
    } 
    // Hemicircle
    else if (robotPose.pathDistance < d1+d2){
      float robotVel = 0.2, K = -1/rad_turn;
      updateDesiredV(robotVel, K);
    }
    // Straight line back
    else if(robotPose.pathDistance < d1+d2+d3){
      float robotVel = 0.2, K = 0;
      updateDesiredV(robotVel, K);
    }
    else if(robotPose.pathDistance < d1+d2+d3+d4){
      float robotVel = -0.2, K = 0;
      updateDesiredV(robotVel, K);
    }
    else if(robotPose.pathDistance < d1+d2+d3+d4+d5){
      float robotVel = -0.2, K = -1/rad_turn;
      updateDesiredV(robotVel, K);
    }
    else if(robotPose.pathDistance < d1+d2+d3+d4+d5 +d6){
      float robotVel = 0.2, K = 0;
      updateDesiredV(robotVel, K);
    }
    else if(robotPose.pathDistance < d1+d2+d3+d4+d5 +d6+d7){
      float robotVel = 0.2, K = -1/rad_turn;
      updateDesiredV(robotVel, K);
    }
    else if(robotPose.pathDistance < d1+d2+d3+d4+d5 +d6+d7+d8){
      float robotVel = 0.2, K = 0;
      updateDesiredV(robotVel, K);
    }
    // Stop at the end
    else {
      float robotVel = 0, K = 0;
      updateDesiredV(robotVel, K);
    }
}

void PathPlanner::updateDesiredV(float robotVel, float K) {
    // command wheel velocities based on K and average forwardVel
    
    // MODIFY CODE BELOW TO SET THE CORRECT VALUES
    desiredWV_L = robotVel * (1 - K *b);
    desiredWV_R = robotVel * (1 + K *b);
}

// PIController Class function implementation (not the focus of this lab)

void PIController::doPIControl(String side, float desV, float currV) {
    float error = desV - currV;

    if (side == "Right") {     // Motor 1 
        // P
        float PCommand = Kpv1 * error;

        // I
        mIntegratedVError1 = constrain(mIntegratedVError1 + error * PERIOD, -0.02, 0.02);
        float ICommand = Kiv1 * mIntegratedVError1;
        
        // Sum
        float command =  PCommand + ICommand;

        int sign = 1;
        md.setM1Speed(constrain(sign * command, -400, 400));   // use sign to make sure positive commands move robot forward
    }
    else if (side == "Left") {  // Motor 2
        // P
        float PCommand = Kpv2 * error;

        // I
        mIntegratedVError2 = constrain(mIntegratedVError2 + error * PERIOD, -0.02, 0.02);
        float ICommand = Kiv2 * mIntegratedVError2;

        // Sum
        float command =  PCommand + ICommand;

        int sign = -1;
        md.setM2Speed(constrain(sign * command, -400, 400));   // use sign to make sure positive commands move robot forward
    }
    else {
        md.setM1Speed(0);
        md.setM2Speed(0);
        //Serial.println("ERROR: INVALID MOTOR CHOICE GIVEN TO PI CONTROLLER");
    }
}


