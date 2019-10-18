#include "odometry/odometry.h"

#include <thread>

#include <iostream>
#include <sstream>
#include <string>
#include <math.h>

#define PI 3.1415926

namespace Odometry{
  Vector2::Vector2(double x, double y){
    this->x = x;
    this->y = y;
  }

  double Vector2::getX(){ return this->x; };
  double Vector2::getY(){ return this->y; };
  void Vector2::setX(double newX) { this->x = newX;}
  void Vector2::setY(double newY) { this->y = newY;}

  double Vector2::getTheta(){
    return atan2(this->getY(), this->getX());
  }

  double Vector2::getMagnitude(){
    return sqrt(this->getX()*this->getX() + this->getY()*this->getY());
  }

  std::string Vector2::ToString(){
    std::stringstream stream;
    stream << "(" << this->getX() << "," << this->getY() << ")";
    return stream.str();

  }

  Vector2 Vector2::normalize(){
    return *this *  (1.0/this->getMagnitude());
  }

  Vector2 Vector2::operator+(Vector2 other){
    return Vector2(this->getX() + other.getX(), this->getY() + other.getY());
  }

  Vector2 Vector2::operator*(double other){
    return Vector2(this->getX() * other, this->getY() * other);
  }

  Vector2 Vector2::operator-(){
    return Vector2(-(this->getX()), -(this->getY()));
  }

  Vector2 Vector2::operator-(Vector2 other){
    return *this + -other;
  }

  Vector2 Vector2::FromPolar(double theta, double r){
    return Vector2(r * cos(theta), r * sin(theta));
  }

  double Vector2::Distance(Vector2 a, Vector2 b){
    return sqrt( ((a.x-b.x)*(a.x-b.x)) + ((a.y-b.y)*(a.y-b.y)) );
  }

  Vector2 Vector2::RotateBy(double theta){
    return this->FromPolar(this->getTheta() + theta, this->getMagnitude());
  }

  Vector2 Vector2::Transpose(){
    return Vector2(this->getY(), this->getX());
  }


  /*
  void SetTrackingCenterParameters(double sL, double sR, double sS); // should be called before init
  void SetStartingPositionAndRotation(Vector2 pos, double rotation); //should be called before init, otherwise default to 0
  void SetTrackingWheelEncoders(pros::ADIEncoderodometryLeftWheel, pros::ADIEncoderodometryRightWheel, pros::ADIEncoderodometryBackWheel); //MUST BE CALLED BEFORE INIT
  void Init(); //Spawns a thread that runs the tracking algorithm
  void Stop(); //stops the thread that runs the tracking algorithm
  Vector2 GetRobotPosition(); //Gets the robot's absolute position
  double GetRobotRotation(); //Gets the robot's absolute rotation
  */

  double sL = 0.0;
  double sR = 0.0;
  double sS = 0.0;
  void SetTrackingCenterParameters(double _sL, double _sR, double _sS){
    sL = _sL;
    sR = _sR;
    sS = _sS;
  }

  Vector2 lastGlobalPosition = Vector2(0.0,0.0);
  double lastRotation = 0.0; //in radians
  double resetRotation = 0.0;

  void SetStartingPositionAndRotation(Vector2 pos, double rotation){
    lastGlobalPosition = Vector2(-pos.getY(), pos.getX()); //inverse of p.getY(), -p.getX()
    lastRotation = -rotation;
    resetRotation = lastRotation;
  }

  pros::Mutex Mutex;

  pros::ADIEncoder*odometryLeftWheel; //each unit is one degree
  int lastLeftWheel = 0;
  int resetLeftWheel = 0;
  pros::ADIEncoder*odometryRightWheel;
  int lastRightWheel = 0;
  int resetRightWheel = 0;
  pros::ADIEncoder*odometryBackWheel;
  int lastBackWheel = 0;
  int resetBackWheel = 0;

  double WheelCircumference = 3.1415926 * 1 * 2.75;

  void SetTrackingWheelEncoders(pros::ADIEncoder* _leftWheel, pros::ADIEncoder* _rightWheel, pros::ADIEncoder* _backWheel){
   odometryLeftWheel = _leftWheel;
    lastLeftWheel = _leftWheel->get_value();
    resetLeftWheel = lastLeftWheel;

   odometryRightWheel = _rightWheel;
    lastRightWheel = _rightWheel->get_value();
    resetRightWheel = lastRightWheel;

   odometryBackWheel = _backWheel;
    lastBackWheel = _backWheel->get_value();
    resetBackWheel = lastBackWheel;
  }

  void SetWheelCircumference(double c){
    WheelCircumference = c;
  }

  bool OdometryRunning = false;
  pros::Task* OdometryThread;

  #define DEG_TO_RAD 0.0174533

  #define LOOP_DELAY 4
  double thingToPrint;

  void OdometryLoop(void* param){
    while (OdometryRunning){
      if (!Mutex.take(5)) continue; //make sure that thread conflicts dont happen
      //store the current encoder values
      int currentLeftWheel = odometryLeftWheel->get_value();
      int currentRightWheel = odometryRightWheel->get_value();
      int currentBackWheel = odometryBackWheel->get_value();


      //calculate change in each encoders value
      double deltaLeftWheelDistance = (currentLeftWheel - lastLeftWheel)/360.0 * WheelCircumference;
      double deltaRightWheelDistance = (currentRightWheel - lastRightWheel)/360.0 * WheelCircumference;
      double deltaBackWheelDistance = (currentBackWheel - lastBackWheel)/360.0 * WheelCircumference;

      //update previous values
      lastLeftWheel = currentLeftWheel;
      lastRightWheel = currentRightWheel;
      lastBackWheel = currentBackWheel;

      //calculate total change since last reset
      double totalChangeLeft = (currentLeftWheel - resetLeftWheel)/360.0 * WheelCircumference;
      double totalChangeRight= (currentRightWheel - resetRightWheel)/360.0 * WheelCircumference;

      //new absolute orientation
      double newRotation = resetRotation + (totalChangeLeft-totalChangeRight)/(sL + sR);

      //change in angle
      double deltaTheta = newRotation - lastRotation;

      //local offset
      Vector2 localOffset = Vector2(0,0);
      if (deltaTheta == 0.0){
        localOffset = Vector2(deltaBackWheelDistance, deltaRightWheelDistance);
      } else {
        localOffset = Vector2(deltaBackWheelDistance/deltaTheta + sS, deltaRightWheelDistance/deltaTheta + sR);
        localOffset = localOffset.operator*(2.0 * sin(deltaTheta/2.0));
      }

      thingToPrint = deltaRightWheelDistance;

      //average orientation
      double averageTheta = lastRotation + deltaTheta/2.0;

      Vector2 globalOffset = localOffset.RotateBy(-averageTheta);
      lastGlobalPosition = lastGlobalPosition + globalOffset;


      lastRotation = newRotation;

      Mutex.give(); //release the mutex to allow other threads to use the encoders


      pros::delay(LOOP_DELAY);
    }
  }

  void Init(){

    OdometryRunning = true;
    //spawn thread
    std::string OdometryThreadName("Odometry Thread");
    OdometryThread = new pros::Task(OdometryLoop, &OdometryThreadName);
  }



  void Stop(){
    OdometryRunning = false;
  }

  Vector2 GetRobotPosition(){
    Mutex.take(100);
    Vector2 p = lastGlobalPosition;
    Mutex.give();
    return Vector2(p.getY(), -p.getX());
  }

  double GetRobotRotation(){
    Mutex.take(100);
    double r = lastRotation;
    Mutex.give();
    return -r;
  }
}
