// Aadi
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
//        Serial.println("ERROR: BAD MOTOR TYPE INPUT, SHOULD BE 26 or 53");
        
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

////RobotPose Class function implementation
//void RobotPose::update(float dThetaL, float dThetaR) {
//    // orientation angle theta increment in radians
//    float dTh = (r / (2.0 * b)) * (dThetaR - dThetaL);
//    
//    Th += dTh;
//    
//    // robot X, Y position increment in meters
//    float dX = (r / 2.0) * cos(Th) * (dThetaR + dThetaL); 
//    float dY = (r / 2.0) * sin(Th) * (dThetaR + dThetaL);
//    
//    X += dX;
//    Y += dY;
//    
//    pathDistance += sqrt(dX * dX + dY * dY);
//}

void RobotPose::update(float x_slam, float y_slam, float Th) {
 
    // robot X, Y position increment in meters
    x_slam = round(x_slam * 100)/100.0;
    y_slam = round(y_slam * 100)/100.0;
    Th = round(Th * 1000)/1000.0;
    
    float dX = x_slam - X;
    float dY = y_slam - Y;
    
    X = x_slam; 
    Y = y_slam;
    
    pathDistance += sqrt(dX * dX + dY * dY);
}

//PathPlanner Class function implementation
void PathPlanner::navigateTrajU(const RobotPose & robotPose) {

    float f2m = 0.3048; // feet to m conversion
    float in_bias = 2*1.414*f2m;
    float rad_turn = 0.25;
    // Straight line forward
    if (robotPose.pathDistance < (4*1.414*f2m - in_bias - rad_turn)) { 
        float robotVel = .2, K = 0;
        updateDesiredV(robotVel, K);
    } 
    // Hemicircle
    else if (robotPose.pathDistance < (4*1.414*f2m - in_bias - rad_turn + rad_turn*PI/2)){
      float robotVel = .2, K = -1/rad_turn;
      updateDesiredV(robotVel, K);
    }
    // Straight line back
    else if(robotPose.pathDistance < (4*1.414*f2m - in_bias - rad_turn + rad_turn*PI/2 + 4*1.414*f2m - in_bias - rad_turn)){
      float robotVel = .2, K = 0;
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
        command =  PCommand + ICommand;

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
        command =  PCommand + ICommand;

        int sign = -1;
        md.setM2Speed(constrain(sign * command, -400, 400));   // use sign to make sure positive commands move robot forward
    }
    else {
        md.setM1Speed(0);
        md.setM2Speed(0);
//        Serial.println("ERROR: INVALID MOTOR CHOICE GIVEN TO PI CONTROLLER");
    }
}


