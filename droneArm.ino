#include <Servo.h>
#include<SoftwareSerial.h>
#include<math.h>
#include<time.h>

// Number Pi
const double pi = 3.1415;

// Servos object
Servo servo1,servo2,servo3,servo4,servo5;

// Reading values
double x,y,z,angle;
char order;

// Initial positions
const int initialPositionServo1 = 90;
const int initialPositionServo2 = 0;
const int initialPositionServo3 = 0;
const int initialPositionServo4 = 100;
const int initialPositionServo5 = 90;

// Destination
int positionServo1 = initialPositionServo1;
int positionServo2 = initialPositionServo2;
int positionServo3 = initialPositionServo3;
int positionServo4 = initialPositionServo4;

// Actual position
int actualPosition1 = initialPositionServo1;
int actualPosition2 = initialPositionServo2;
int actualPosition3 = initialPositionServo3;
int actualPosition4 = initialPositionServo4;

// Possibles position for Servo5 (claw)
const int clawOpen = 0;
const int clawClose = 180;

// Robot dimensions (cm)
const double bar2 = 13.5;
const double bar3 = 28;

// Delay for servos writing
int servoDelay = 100;

// Time for closing/opening the claw and flag to know its state
const int timeClaw = 4500; // Default time
int timeClawReceived = timeClaw;

// Flags for indicate the last position type introduced
int followXYZ = 0;
int followAngle = 0;

// Flag of the state of the claw, initially open
//int clawFlag = clawOpen;

// Step for servos increments or decrements
int servoStep = 1;

// Constant values for servos 2 y 3
const double losenessServo2 = 15;
const double losenessServo3 = 20;

// Possible cases for reading data or show messages
const int caseOrder = 0;
const int instruction = 1;
const int caseAngle = 2;
const int case3D = 3;
const int caseClaw = 4;
const int caseHome = 5;
const int caseStep = 6;
const int caseInfoStep = 7;
const int caseClawTime = 8;

void setup() {  
  // Attaches the servos to the Arduino pines
  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo5.attach(10);

  // Opens serial communication
  Serial.begin(9600);

  // Sets the servos to their initial positions
  servo1.write(initialPositionServo1);
  servo2.write(initialPositionServo2);
  servo3.write(initialPositionServo3);
  servo4.write(initialPositionServo4);
  servo5.write(initialPositionServo5);
}


void loop() {

//  showMessage(instruction);
  readSerial(caseOrder);

  if( order == 'A' || order == 'a' ){
    
    showMessage(caseAngle);  
    readSerial(caseAngle);    
//    inverseKinematic(x,y,angle); 
    followAngle = 1;   
    
  }else if( order == 'P' || order == 'p' ){
    
    showMessage(case3D);
    readSerial(case3D);
//    inverseKinematic3D(x,y,z);  
    followXYZ = 1;
      
  }else if( order == 'C' || order == 'c' ){
    
    showMessage(caseClaw);
    clawState();
    
  }
  else if( order == 'H' || order == 'h' ){
    
    showMessage(caseHome);
    home();
   
  }
  else if( order == 'S' || order == 's' ){
    
    showMessage(caseStep);    
    readSerial(caseStep);
    showMessage(caseInfoStep);
    
  }else if( order == 'T' || order == 't' ){
    showMessage(caseClawTime);
    readSerial(caseClawTime);
    clawState();
    timeClawReceived = timeClaw;
  }

  if( followXYZ ){
    inverseKinematic3D(x,y,z); 
  }else if( followAngle ){
    inverseKinematic(x,y,angle); 
  }

  aproxPosition(actualPosition1,positionServo1);
  servo1.write(actualPosition1);
  
  aproxPosition(actualPosition2,positionServo2);
  servo2.write(actualPosition2);
        
  aproxPosition(actualPosition3,positionServo3);
  servo3.write(actualPosition3);
        
  aproxPosition(actualPosition4,positionServo4);
  servo4.write(actualPosition4);
    
  delay(servoDelay);
  
}

// Reads data
void readSerial(int readingCase)
{
  switch(readingCase){
    case caseOrder:
//      while(!Serial.available());
      order = Serial.read();
      break;
    case caseAngle:
//      while(!Serial.available());
      x = Serial.parseFloat();
      y = Serial.parseFloat();
      angle = Serial.parseFloat(); 
      break;
    case case3D:
//      while(!Serial.available());
      x = Serial.parseFloat();
      y = Serial.parseFloat();
      z = Serial.parseFloat();
      break;
    case caseStep:
//      while(!Serial.available());
      servoStep = Serial.parseInt();
      break;
    case caseClawTime:
//      while(!Serial.available());
      timeClawReceived = Serial.parseInt();
      break;
  }
}

// Calculates the positions 
// of each servo for the point (x,y,angle)
void inverseKinematic(double x,double y,double angle)
{ 
  // Virtual bar, equal to the distance between
  // the given point and zero 
  double barAux = sqrt(pow(x,2) + pow(y,2));

  if( barAux > bar2+bar3){
    barAux = bar2+bar3;
  }
  
  // Coordenate q1
  positionServo1 = angle + initialPositionServo1; 

  // Coordenate q2
  
  double gamma = acos( (pow(bar3,2)-pow(barAux,2)-pow(bar2,2)) / (-2*barAux*bar2) );
  radians2degrees(gamma);
  
  double alfa = atan2(y,x);
  radians2degrees(alfa);
  
  positionServo2 = 180 - alfa - gamma + losenessServo2;

  // Coordenate q3
  double theta = acos( (pow(barAux,2)-pow(bar2,2)-pow(bar3,2)) / (-2*bar2*bar3) );
  radians2degrees(theta);

  positionServo3 = theta - losenessServo3;

  // Coordenate q4
  positionServo4 = initialPositionServo4; // Por definir
  
  // Applies new position
//  newPosition(positionServo1,positionServo2,positionServo3,positionServo4);
  
  followAngle = 0;
}

// Calculates the positions 
// of each servo for the point (x,y,z)
void inverseKinematic3D(double x,double y,double z)
{
  // Coordenate q1
  double phi = atan2(y,x);
  radians2degrees(phi);
  
  positionServo1 = phi + initialPositionServo1;
    
  // Coordenate q2
  double p = sqrt(pow(x,2)+pow(y,2));
  double d = sqrt(pow(p,2)+pow(z,2));

  if( d>bar2+bar3 ){
    d = bar2+bar3;
  }

  double alfa = atan2(z,p);

  double gamma = acos( (pow(bar3,2)-pow(d,2)-pow(bar2,2)) / ( -2*d*bar2 ) );
  radians2degrees(gamma);

  positionServo2 = 180 - alfa - gamma + losenessServo2;

  // Coordenate q3
  double omega = acos( (pow(d,2)-pow(bar2,2)-pow(bar3,2)) / (-2*bar2*bar3) );
  radians2degrees(omega);
  
  positionServo3 = omega - losenessServo3;

  // Coordenate q4
  positionServo4 = initialPositionServo4;

  // Applies new position
//  newPosition(positionServo1,positionServo2,positionServo3,positionServo4);

  followXYZ = 0;
}


// Applies a new position to the servos
//void newPosition(int positionServo1,int positionServo2,int positionServo3,int positionServo4,int positionServo5)
//{
//  writeSlow(servo1,servo2,servo3,servo4,positionServo1,positionServo2,positionServo3,positionServo4);
//  servo5.write(positionServo5);
//}

void newPosition(int positionServo1,int positionServo2,int positionServo3,int positionServo4)
{
  writeSlow(servo1,servo2,servo3,servo4,positionServo1,positionServo2,positionServo3,positionServo4);
}

void newPosition(int positionServo5)
{
//  if( clawFlag!=positionServo5 ){
      
    unsigned long timeStart,timeFinish;

    servo5.write(positionServo5);
    
    timeStart = millis();
    timeFinish = millis();

    while(timeFinish-timeStart < timeClawReceived){
        timeFinish = millis();
    }

    servo5.write(initialPositionServo5);

//    clawFlag = positionServo5;
//  }  
}


// Opens or closes the claw
void clawState()
{
  // Waits for data
  while(!Serial.available());

  int state = Serial.parseInt();
  switch (state){
    case 0:
        Serial.print("Closing the claw...\n");
     
        newPosition(clawClose);
        
        Serial.print("Claw closed \n\n");
        break;
    case 1:
        Serial.print("Opening the claw... \n");

        newPosition(clawOpen);
      
        Serial.print("Claw opened \n\n");
        break;     
  }
}


// Returns to the initial position
void home()
{
//  newPosition(initialPositionServo1,initialPositionServo2,initialPositionServo3,initialPositionServo4,initialPositionServo5); 
//  newPosition(initialPositionServo1,initialPositionServo2,initialPositionServo3,initialPositionServo4); 
//  if( clawFlag==clawClose ){
//      newPosition(clawOpen);
//  }

 positionServo1 = initialPositionServo1;
 positionServo2 = initialPositionServo2;
 positionServo3 = initialPositionServo3;
 positionServo4 = initialPositionServo4;
}


// Inverse trigonometric functions: 
// Both work in the same way: once you know
// either the sine or the cosine, it's easy
// to calculate the other one by the known trigonometric
// relation cos^2+sin^2=1. As tangent is defined
// as sine divided by cosine, and the inverse tangent
// function is provided by Arduino math.h header, to 
// calculate the angle is as easy as use that function
// with sine/cosine.

// Inverse cosine function
double acos(double cos)
{
  double sin = sqrt(1-pow(cos,2));
  return atan2(sin,cos);
}

// Inverse sine function
double asin(double sin)
{
  double cos = sqrt(1-pow(sin,2));
  return atan2(sin,cos);
}


// Converts radians to degrees
void radians2degrees(double &angle)
{
  angle = angle*180/pi;
}


// Writes the new position gradually, in a slow manner
void writeSlow(Servo servo1,Servo servo2,Servo servo3,Servo servo4,int newPosition1,int newPosition2,int newPosition3,int newPosition4)
{ 
  actualPosition1 = servo1.read();
  actualPosition2 = servo2.read();
  actualPosition3 = servo3.read();
  actualPosition4 = servo4.read();
  
  while( actualPosition1 != newPosition1 || actualPosition2 != newPosition2 
  || actualPosition3 != newPosition3 || actualPosition4 != newPosition4 ){
    
    aproxPosition(actualPosition1,newPosition1);
    servo1.write(actualPosition1);
    
    aproxPosition(actualPosition2,newPosition2);
    servo2.write(actualPosition2);
        
    aproxPosition(actualPosition3,newPosition3);
    servo3.write(actualPosition3);
        
    aproxPosition(actualPosition4,newPosition4);
    servo4.write(actualPosition4);
    
    delay(servoDelay);
    }
}


// Auxiliar function for writeSlow: reduces or increases
// the actual position of the Servo to approaches it 
// to the deserves position by servoStep step
void aproxPosition(int &actualPosition,int newPosition)
{
  if( actualPosition > newPosition ){
    actualPosition = constrain(actualPosition-servoStep,newPosition,actualPosition);
  }
  else if( actualPosition < newPosition){
    actualPosition = constrain(actualPosition+servoStep,actualPosition,newPosition);
  }
}


// Shows info and instruction messages
void showMessage(int typeMessage)
{
  switch(typeMessage){
    case instruction:
      // Shows the commands for the arm
      Serial.print("Waiting for an order: \n");
      Serial.print("A -> new position in the plane of the bars 2 & 3 \n");
      Serial.print("P -> new position with coordenates x, y, z \n");
      Serial.print("C -> modify the claw state \n");
      Serial.print("T -> modify the claw state during a given time \n");
      Serial.print("H -> home position \n");
      Serial.print("S -> modify the step of the servos movement \n\n");
      break;
    case caseAngle:
      Serial.print("Introduce three integers: x, y, angle \n"); 
      break;
    case case3D:
      Serial.print("Introduces three integers: x, y, z \n");
      break;
    case caseClaw:
      Serial.print("Introduce 0 or 1: \n");
      Serial.print("0 -> close the claw\n");
      Serial.print("1 -> open the claw\n\n");
      break;
    case caseHome:
      Serial.print("Going back home \n");
      break;
    case caseStep:
      Serial.print("Introduce an integer: the new servoStep\n");
      Serial.print("Previous step: ");
      Serial.println(servoStep);
      break;
    case caseInfoStep:
      Serial.print("New step: ");
      Serial.println(servoStep);
      Serial.print("\n\n");
      break;
    case caseClawTime:
      Serial.print("First, introduce the time desired in miliseconds (integer)\n");
      Serial.print("Then, introduce 0 or 1: \n");
      Serial.print("0 -> close the claw\n");
      Serial.print("1 -> open the claw\n\n");
      break;
  }
}

