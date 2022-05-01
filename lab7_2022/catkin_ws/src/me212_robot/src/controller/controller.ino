// Zack Bright        - zbright  _ mit _ edu,    Sept 2015
// Daniel J. Gonzalez - dgonz    _ mit _ edu,    Sept 2015
// Fangzhou Xia       - xiafz    _ mit _ edu,    Sept 2015
// Peter KT Yu        - peterkty _ mit _ edu,    Sept 2016
// Ryan Fish          - fishr    _ mit _ edu,    Sept 2016
// Jerry Ng           - jerryng  _ mit _ edu,    Feb  2019

#include "Arduino.h"
#include "helper.h"

EncoderMeasurement  encoder(26);      // FIX THIS: encoder handler class, set the motor type 53 or 26 here
RobotPose           robotPose;        // robot position and orientation calculation class
PIController        wheelVelCtrl;     // velocity PI controller class
SerialComm          serialComm;       // serial communication class
PathPlanner         pathPlanner;      // path planner
unsigned long       prevTime = 0;

boolean usePathPlanner = false;

void setup() {
    Serial.begin(115200);       // initialize Serial Communication
    encoder.init();  // connect with encoder
    wheelVelCtrl.init();        // connect with motor
    delay(1e3);                 // delay 1000 ms so the robot doesn't drive off without you
}

void loop() {
    //timed loop implementation
    unsigned long currentTime = micros();
    
    if (currentTime - prevTime >= PERIOD_MICROS) {
      
        // 1. Obtain and convert encoder measurement
        encoder.update(); 

        // 2. Compute robot odometry
        robotPose.update(encoder.dPhiL, encoder.dPhiR); 

        // 3. Send robot odometry through serial port
        serialComm.send(robotPose); 
        
        // 4. Compute desired wheel velocity without or with motion planner
        if (!usePathPlanner) {
            // 4.1 a fixed wheel speed for testing odometry
            pathPlanner.desiredWV_R = 0.2;   
            pathPlanner.desiredWV_L = 0.2;
        }
        else{
            // 4.2 compute wheel speed from using a navigation policy
            pathPlanner.navigateTrajU(robotPose); 
        }
        Serial.print(robotPose.Y);
        // 5. Command desired velocity with PI controller
        wheelVelCtrl.doPIControl("Left",  pathPlanner.desiredWV_L, encoder.v_L); 
        wheelVelCtrl.doPIControl("Right", pathPlanner.desiredWV_R, encoder.v_R);

        prevTime = currentTime; // update time
    } 
}




