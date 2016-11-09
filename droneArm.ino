///////////////////////////////////////////
//
//  DroneArm
//    Author: Jose Maria Aguilar
//    Modified: Pablo R.S.
//
///////////////////////////////////////////
// Main code for controlling hexa arm.

#include "DroneArm.h"
#include <SoftwareSerial.h>

DroneArm droneArm;

void setup() {  
  // Opens serial communication
  Serial.begin(9600);

  // Prepare arm
  droneArm.setup(); 
  delay(2000);

  // Shows the instructions
  showHelp();
}


void loop() {
  if(Serial.available()){
    char order = Serial.read();
    double x, y, z, angle;
    if( order == 'A' || order == 'a' ){
      x = Serial.parseFloat();
      y = Serial.parseFloat();
      angle = Serial.parseFloat(); 
  
      double globalX = x*cos(angle/180.0*pi);
      double globalY = x*sin(angle/180.0*pi);
      double globalZ = y;
      droneArm.move(globalX, globalY, globalZ);
    }else if( order == 'P' || order == 'p' ){
      x = Serial.parseFloat();
      y = Serial.parseFloat();
      z = Serial.parseFloat();
      droneArm.move(x, y, z);  
    }else if( order == 'C' || order == 'c' ){
      int gripperState = Serial.parseInt();
      if(gripperState == 0){
        droneArm.stopGripper();
      }else if(gripperState == 1){
        droneArm.openGripper();
      }else if(gripperState == 2){
        droneArm.closeGripper();
      }
    }
    else if( order == 'H' || order == 'h' ){
      droneArm.home();
    }
    else if( order == 'S' || order == 's' ){  
      droneArm.speed(Serial.parseInt());
    }
  }
}

// Shows info and instruction messages
void showHelp() {
    // Shows the commands for the arm
    Serial.print("Waiting for an order: \n");
    Serial.print("A -> new position in the plane of the bars 2 & 3 \n");
    Serial.print("P -> new position with coordenates x, y, z \n");
    Serial.print("C -> modify the claw state 0 stop, 1 open and 2 closes\n");
    Serial.print("H -> home position \n");
    Serial.print("S -> modify the speed of the servos\n\n");
}

