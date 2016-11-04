///////////////////////////////////////////
//
//  DroneArm
//    Author: Jose Maria Aguilar
//    Modified: Pablo R.S.
//
///////////////////////////////////////////
// Interface with arm mounted on the drone (vigus grvc us).

#ifndef DRONEARM_DRONEARM_H_
#define DRONEARM_DRONEARM_H_

#include <Servo.h>
#include<math.h>


const double pi = 3.14159265359;

class DroneArm{
  public:
   bool setup();
 
  private: 
  const double mHumerus = 13.5;
  const double mRadius = 28;
  
  Servo   mServo1,  // Yaw
          mServo2,  // pitch 1
          mServo3,  // pitch 2
          mServo4,  // Wirst
          mServo5;  // gripper
          
  int mOffset1 = 90, 
      mOffset2 = 0, 
      mOffset3 = 0, 
      mOffset4 = 100, 
      mOffset5 = 90;  
}


#endif // DRONEARM_DRONEARM_H_
