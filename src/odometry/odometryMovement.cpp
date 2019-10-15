#include "odometry/odometryMovement.h"
#include "odometry/odometry.h"
#include "odometry/angle.h"
#include "odometry/mecanum.h"

#include <math.h>
#include <algorithm>

namespace Odometry::Movement {

  OdometryMotor::OdometryMotor(int port, bool reversed, pros::motor_gearset_e gearset){
    this->motor = new pros::Motor(port);
    this->reversed = reversed;
    if (gearset == MOTOR_GEARSET_36) this->gearset = 36;
    else if (gearset == MOTOR_GEARSET_6) this->gearset = 6;
    else this->gearset = 18;
  }

  OdometryMotor::~OdometryMotor(){
    delete this->motor;
  }

  void OdometryMotor::set_brake_mode(pros::motor_brake_mode_e brakeMode){
    this->motor->set_brake_mode(brakeMode);
  }

  void OdometryMotor::move_velocity(double v){
    this->motor->move_velocity(18.0/this->gearset * v * (this->reversed?-1:1));
  }

  PIDSettings::PIDSettings(double PFactor, double IFactor, double DFactor){
    this->PFactor = PFactor;
    this->IFactor = IFactor;
    this->DFactor = DFactor;
  }

  double PIDSettings::eval(double error, double timestep, PIDState& state){
    double derivative = (error - state.lastError) / timestep;
    state.update(error, timestep);
    double result = error * this->PFactor + state.integral * this->IFactor + derivative * this->DFactor;

    return result;
  }

  void PIDState::update(double error, double timestep){
    this->lastError = error;
    this->integral += error*timestep;
  }

  PIDState::PIDState(double lastError, double integral){
    this->lastError = lastError;
    this->integral += integral;
  }

  double eval(double pError, double iError, double dError);

  double clampf(double x, double min, double max){
    if (x > max) return max;
    if (x < min) return min;
    return x;
  }

  #ifdef MECANUM_WHEEL
  OdometryMotor* frontLeft;
  OdometryMotor* frontRight;
  OdometryMotor* backLeft;
  OdometryMotor* backRight;
  void SetMotorsMecanum(OdometryMotor* frontLeftm, OdometryMotor* backLeftm, OdometryMotor* frontRightm, OdometryMotor* backRightm){
    frontLeft = frontLeftm;
    frontRight = frontRightm;
    backLeft = backLeftm;
    backRight = backRightm;
  }
  #endif

  double mod(double a, double b){
    return a - floor(a/b) * b;
  }

  #define PI 3.1415926

  #define MOVEMENT_DELAY 20

  #define DEG_TO_RAD 0.0174533

  //PIDSettings TurnTowardsPID = PIDSettings(4,.45,-.05);

  void TurnTowards(Angle targetAngle, PIDSettings TurnTowardsPID){
   PIDState state = PIDState();
   Angle currentAngle = 0.0;

   double threshold = DEG_TO_RAD * 2;
   bool first = false;
   while (Angle::UnsignedAngleBetween(currentAngle, targetAngle) > threshold || first==false){
     first = true;
     //pros::lcd::print(1, "%f",angleBetween(targetAngle,GetRobotRotation())/DEG_TO_RAD);
     //pros::lcd::print(2, "%f",GetRobotRotation()/DEG_TO_RAD);
     currentAngle = GetRobotRotation();
     pros::lcd::print(1, "%f",currentAngle/DEG_TO_RAD);

       double error = Angle::SignedAngleBetween(currentAngle, targetAngle);


       double turnStrength = TurnTowardsPID.eval(error, MOVEMENT_DELAY/1000.0, state);

       int left = -(turnStrength/DEG_TO_RAD);
       int right = (turnStrength/DEG_TO_RAD);

       pros::lcd::print(3, "Angle: %f", currentAngle.GetDegreeMeasure());
       pros::lcd::print(4, "Angle Error: %f", error/DEG_TO_RAD);

       #ifdef MECANUM_WHEEL

       frontLeft->move_velocity(left);
       backLeft->move_velocity(left);

       frontRight->move_velocity(right);
       backRight->move_velocity(right);

       #endif

     pros::delay(MOVEMENT_DELAY);
   }
   frontLeft->move_velocity(0);
   backLeft->move_velocity(0);

   frontRight->move_velocity(0);
   backRight->move_velocity(0);
  }

  //PIDSettings OdometryMovementGoToSpotTurnPID = PIDSettings(4,.35,-.05);
  //PIDSettings OdometryMovementGoToSpotMovePID = PIDSettings(5,0.1,-1);

  void GoToSpot(Vector2 targetSpot){
    GoToSpot(targetSpot, PIDSettings(4,.35,-.05), PIDSettings(5,0.1,-1));
  }

  void GoToSpot(Vector2 targetSpot, PIDSettings OdometryMovementGoToSpotTurnPID, PIDSettings OdometryMovementGoToSpotMovePID){
    double distanceThreshold = 1;
    bool first = false;

    Vector2 currentSpot = Vector2(0,0);
    Angle currentAngle = 0.0;

    PIDState TurnPIDState = PIDState();
    PIDState MovePIDState = PIDState();

    while (first==false || Vector2::Distance(currentSpot, targetSpot) > distanceThreshold){
      first = true;

      currentSpot = GetRobotPosition();
      currentAngle = GetRobotRotation();

      Vector2 distanceError = targetSpot - currentSpot;
      double angleError = Angle::SignedAngleBetween(currentAngle, distanceError.getTheta());

      Vector2 normalizedDistanceError = distanceError.RotateBy(-currentAngle);

     //normalizedDistanceError is the point that the robot wants to go to relative to the robot, angle-wise and position-wise

     bool backwards = false;
      if (normalizedDistanceError.getX() < 0){
        //robot needs to go backwards into the point
        backwards = true;
      }

      #ifdef MECANUM_WHEEL

      double pidSpeed = OdometryMovementGoToSpotMovePID.eval(distanceError.getMagnitude(), MOVEMENT_DELAY/1000.0, MovePIDState) * (backwards?1:-1);
      double pidAngle = OdometryMovementGoToSpotTurnPID.eval(angleError, MOVEMENT_DELAY/1000.0, TurnPIDState);

      double pidSpeedRangeLow = -180.0;
      double pidSpeedRangeHigh = -pidSpeedRangeLow;

      double clampedSpeed = clampf(pidSpeed,pidSpeedRangeLow,pidSpeedRangeHigh);

      double left = -pidAngle/DEG_TO_RAD - clampedSpeed;
      double right = pidAngle/DEG_TO_RAD - clampedSpeed;

      frontLeft->move_velocity(left);
      backLeft->move_velocity(left);

      frontRight->move_velocity(right);
      backRight->move_velocity(right);

      #endif

      pros::lcd::print(1, "Target: %s", targetSpot.ToString());
      pros::lcd::print(2, "Position: %s", currentSpot.ToString());
      pros::lcd::print(3, "Angle: %f", currentAngle/DEG_TO_RAD);
      pros::lcd::print(4, "Angle Error: %f", angleError/DEG_TO_RAD);
      pros::lcd::print(5, "Distance Error: %s", distanceError.ToString());
      pros::lcd::print(6, "PidSpeed %f", pidSpeed);
      pros::lcd::print(7, "PidAngle %f", pidAngle/DEG_TO_RAD);

      pros::delay(MOVEMENT_DELAY);
    }

    frontLeft->move_velocity(0);
    backLeft->move_velocity(0);

    frontRight->move_velocity(0);
    backRight->move_velocity(0);
  }

  #if defined(MECANUM_WHEEL) || defined(XDRIVE)
  void MoveLinear(Vector2 spot, PIDSettings LinearMoveSettings, PIDSettings LinearTurnSettings){
    MoveLinearComplete(spot, 0.0, false, LinearMoveSettings, LinearTurnSettings);
  }
  void MoveLinear(Vector2 spot, Angle angle, PIDSettings LinearMoveSettings, PIDSettings LinearTurnSettings){
    MoveLinearComplete(spot, angle, true, LinearMoveSettings, LinearTurnSettings);
  }

  namespace{

    void MoveLinearComplete(Vector2 spot, Angle angle, bool doAngleError, PIDSettings LinearMoveSettings, PIDSettings LinearTurnSettings){

      //PIDSettings LinearMoveSettings = PIDSettings(0,0,0);
      //PIDSettings LinearTurnSettings = PIDSettings(0,0,0);
/*
      if (doAngleError == false){
        LinearMoveSettings = PIDSettings(10,0.01,-1);
        LinearTurnSettings = PIDSettings(4,.35,-.05);
      } else {
        LinearMoveSettings = PIDSettings(8,-.01,-.1);
        LinearTurnSettings = PIDSettings(4,0,-.1);
      }
*/
      double distanceThreshold = 1;
      double angleThreshold = Angle::FromDegrees(5);

      bool first = false;
      Vector2 currentSpot = Vector2(0,0);
      Angle currentAngle = 0.0;

      PIDState TurnPIDState = PIDState();
      PIDState MoveXPIDState = PIDState();
      PIDState MoveYPIDState = PIDState();

      Mecanum::MecanumMotorSpeeds forwardBasis (1, 1, 1, 1);
      Mecanum::MecanumMotorSpeeds leftBasis (-1, 1, 1, -1);
      Mecanum::MecanumMotorSpeeds turnLeftBasis = doAngleError ? Mecanum::MecanumMotorSpeeds(-1, -1, 1, 1) : Mecanum::MecanumMotorSpeeds(0,0,0,0);

      while (first==false || (Vector2::Distance(currentSpot, spot) > distanceThreshold ||
      (Angle::UnsignedAngleBetween(currentAngle, angle) > angleThreshold  && doAngleError))){
        first = true;

        currentSpot = GetRobotPosition();
        currentAngle = GetRobotRotation();

        Vector2 offset = (spot - currentSpot);

        double angleError = Angle::SignedAngleBetween(currentAngle, angle)/DEG_TO_RAD;

        double pidX = LinearMoveSettings.eval(offset.getX(), MOVEMENT_DELAY/1000.0, MoveXPIDState);
        double pidY = LinearMoveSettings.eval(offset.getY(), MOVEMENT_DELAY/1000.0, MoveYPIDState);

        double pidAngle = LinearTurnSettings.eval(angleError, MOVEMENT_DELAY/1000.0, TurnPIDState);

        Vector2 direction = Vector2(pidX, pidY).RotateBy(-currentAngle);

        Mecanum::MecanumMotorSpeeds all = forwardBasis*direction.getX() + leftBasis*direction.getY() + turnLeftBasis*pidAngle;

        all = all.normalize(200);

        all.ApplyToMotors(*frontLeft, *backLeft, *frontRight, *backRight);

        pros::lcd::print(0, "offset: %s", offset.ToString());
    		pros::lcd::print(1, "angle error: %f", angleError);

        pros::lcd::print(2, "pidX: %f", pidX);
        pros::lcd::print(3, "pidY: %f", pidY);
        pros::lcd::print(4, "pidAngle: %f", pidAngle);

        pros::delay(MOVEMENT_DELAY);
      }

      frontLeft->move_velocity(0);
      backLeft->move_velocity(0);

      frontRight->move_velocity(0);
      backRight->move_velocity(0);
    }
  }
  #endif
}
