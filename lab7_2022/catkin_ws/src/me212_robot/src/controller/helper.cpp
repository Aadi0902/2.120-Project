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
    else
        Serial.println("ERROR: BAD MOTOR TYPE INPUT, SHOULD BE 26 or 53");
        
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
void RobotPose::update(float dPhiL, float dPhiR) {
    // orientation angle theta increment in radians
    float dTh;
    
    // robot X, Y position increment in meters
    float dX, dY;
    
    // MODIFY CODE BELOW TO SET THE CORRECT VALUES
    //   Relavent constants: r, b
    //   Relavent function: sqrt()
    //   Use the equations referenced in the handout to set these values.
    
    dTh = r/(2*b)*(dPhiR-dPhiL);
    
    
    Th = Th+dTh;
    dX = r/2*(cos(Th)*dPhiR+cos(Th)*dPhiL);
    dY = r/2*(sin(Th)*dPhiR+sin(Th)*dPhiL);
    X = X+dX;
    Y = Y+dY;

    pathDistance += sqrt(dX^2+dY^2);

//PathPlanner Class function implementation
void PathPlanner::navigateTrajU(const RobotPose & robotPose) {
    // Use a series of IF statements to break up the path into steps.
    // Check whether you are ready to move forward into the next stage of the path
    
    // an example for the first straight line has been provided:
    bool collected = false;
    float robotVel, K;
    int robotCase;

    if (robotPose.X<.1 && robotPose.Y<.1):
    // in starting box
        robotCase=1
        if (robotPose.Th<PI/4) {
            float robotVel = .1 K=-1/b
        }
        else if (robot.Pose>PI/4) {
            float robotVel = .1 K=1/b
        }
        else {
            float robotVel =.5 K=0
        }


    // if (robotPose.Th < PI/4 ) {
    //     float robotVel = .5, K=1/b
    //     updateDesiredV(robotVel, K);
    // }
    else if (robotPose.X<1.1 && && robotPose.Y<1.1) {
        //drive from start box to middle
        robotCase = 2
        robotVel= .5 K=0
        updateDesiredV(robotVel, K);
        
    }
    else if (robotCase==2) {
        robotVel=.1, K=1/b
        // turn left to 45
        if (abs(robotPose.Th-3*PI/4)<=PI/90) {
            robotCase=3
        }

    }
    else if (robotCase==3) {
        robotVel=-.5, K=0
        // reverse to middle
        if (abs(robotPose.Y-1.1)<.1 and abs(robotPose.X-1.1)<.1) {
            //collected regolith, back to middle
            robotCase=4
        }
        
        // if (robotPose.Y<.1 && robotPose.X>2) {
        //     if (!collected) {
        //         collected=true
        //         // take the time
        //     }
        //     robotVel=0, K=0
        // }
        // else {
        //     robotVel=.5, K=0
        // }
    }
    // turn to april tag bottom right corner
    // else if (robotPose.X>1.1 and robotPose.Y>1.1) {
    //     robotVel = .1, K=-1/b
        
    else if (robotCase==4) {
        robotVel =.1 K=1/b
        if (abs(robotPose.Th-PI/4)<=PI/90) {
            // collected regolith, back to the middle, turning to 45
            robotCase= 5
        }
    }
    else if (robotCase==5) {

    }
    // Straight line forward
    // if (robotPose.pathDistance < 1.0) { 
    //     float robotVel = .2, K = 0;
    //     updateDesiredV(robotVel, K);
    // } 
    // Hemicircle
    // trying to commit new code 
    //else if (){
    //}
    // Straight line back
    //else if(){
    //}
    // Stop at the end
    //else {
    //}
    updateDesiredV(robotVel, K)
}

void PathPlanner::updateDesiredV(float robotVel, float K) {
    // command wheel velocities based on K and average forwardVel
    
    // MODIFY CODE BELOW TO SET THE CORRECT VALUES
     desiredWV_R = robotVel*(K*b-1)/r;
     desiredWV_L = robotVel*(3-K*b)/r;
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
        Serial.println("ERROR: INVALID MOTOR CHOICE GIVEN TO PI CONTROLLER");
    }
}
//test

