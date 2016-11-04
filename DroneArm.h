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

#include "VarSpeedServo.h"
#include <math.h>

const double pi = 3.14159265359;

class DroneArm{
  public:
    bool setup();
    void speed(const int _speed);
    int speed();

    void move(const double &_x,const double &_y, const double &_z);
    void openGripper();
    void closeGripper();
    void stopGripper();

    void home();
  private: 
    bool inverseKinematic(const double _target[3], double _joints[4]);
  
  private:
  
  const double mHumerus = 13.5;
  const double mRadius = 28;
  
  VarSpeedServo   mServo1,  // Yaw
                  mServo2,  // pitch 1
                  mServo3,  // pitch 2
                  mServo4,  // Wirst
                  mServo5;  // gripper
          
  int mOffset1 = 90, 
      mOffset2 = 0, 
      mOffset3 = 0, 
      mOffset4 = 100, 
      mOffset5 = 90;  
      
  // Loseness of the servos 2 and 3 due to gravity.
  const int mLosenessServo2 = 15;
  const int mLosenessServo3 = 15;

  // Default speed of servos.
  int mSpeed = 10;
};


#endif // DRONEARM_DRONEARM_H_
