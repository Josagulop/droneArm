///////////////////////////////////////////
//
//  DroneArm
//    Author: Jose Maria Aguilar
//    Modified: Pablo R.S.
//
///////////////////////////////////////////
// Interface with arm mounted on the drone (vigus grvc us).

#include "DroneArm.h"

//--------------------------------------------------------------------------------------------------
bool DroneArm::setup() {
  // Attaches the servos to the Arduino pines
  mServo1.attach(3);
  mServo2.attach(5);
  mServo3.attach(6);
  mServo4.attach(9);
  mServo5.attach(10);

  // Sets the servos to their initial positions
  home();
}

//--------------------------------------------------------------------------------------------------
void DroneArm::speed(int _speed) {
  mSpeed = _speed;
}

//--------------------------------------------------------------------------------------------------
int DroneArm::speed() {
  return mSpeed;
}

//--------------------------------------------------------------------------------------------------
void DroneArm::move(const double &_x,const double &_y, const double &_z) {
  double pos[3];
  pos[0] = _x;
  pos[1] = _y;
  pos[2] = _z;
  double ik[4];
  inverseKinematic(pos, ik);
  
  mServo1.write(ik[0], mSpeed, false);
  mServo2.write(ik[1], mSpeed, false);
  mServo3.write(ik[2], mSpeed, false);
  mServo4.write(ik[3], mSpeed, false);
}

//--------------------------------------------------------------------------------------------------
void DroneArm::openGripper(){
  mServo5.write(0);
}

//--------------------------------------------------------------------------------------------------
void DroneArm::closeGripper(){
  mServo5.write(180);
}

//--------------------------------------------------------------------------------------------------
void DroneArm::stopGripper(){
  mServo5.write(mOffset5);
}

//--------------------------------------------------------------------------------------------------
void DroneArm::home(){
  mServo1.write(mOffset1, mSpeed, false);
  mServo2.write(mOffset2, mSpeed, false);
  mServo3.write(mOffset3, mSpeed, false);
  mServo4.write(mOffset4, mSpeed, false);
  mServo5.write(mOffset5, mSpeed, false);
}

//--------------------------------------------------------------------------------------------------
// Calculates the positions of each servo for the point (x,y,z)
bool DroneArm::inverseKinematic(const double _target[3], double _joints[4]) {
  // Coordenate q1
  double phi = atan2(_target[1], _target[0]) * 180 / pi;
  _joints[0] = phi + mOffset1;

  // Coordenate q2
  double p = sqrt(pow(_target[0], 2) + pow(_target[1], 2));
  double d = sqrt(pow(p, 2) + pow(_target[2], 2)); // Virtual bar

  if ( d > mHumerus + mRadius ) {
    d = mHumerus + mRadius;
  }

  double alfa = atan2(_target[2], p) * 180 / pi;

  double gamma = acos((pow(mRadius, 2) - pow(d, 2) - pow(mHumerus, 2)) / ( -2 * d * mHumerus )) * 180 / pi;

  _joints[1] = 180 - alfa - gamma + mLosenessServo2 + mOffset2;

  // Coordenate q3
  double omega = acos( (pow(d, 2) - pow(mHumerus, 2) - pow(mRadius, 2)) / (-2 * mHumerus * mRadius) ) * 180 / pi;
  _joints[2] = omega - mLosenessServo3 + mOffset3;

  // Coordenate q4
  _joints[3] = mOffset4;

  return true;
}
