#ifndef ODOMETRY_MOVEMENT_API
#define ODOMETRY_MOVEMENT_API

#include "main.h"
#include "odometry/odometry.h"
#include "odometry/angle.h"

#define MECANUM_WHEEL

namespace Odometry::Movement{

  struct PIDState{
  public:
    double lastError;
    double integral;

    PIDState(double lastError = 0, double integral = 0);

    void update(double error, double timestep);
  };

  struct PIDSettings{
  public:
    double PFactor;
    double IFactor;
    double DFactor;

    PIDSettings(double PFactor, double IFactor, double DFactor);
    double eval(double error, double timestep, PIDState& state);
  };

  void TurnTowards(Angle targetAngle, PIDSettings TurnTowardsPID);
  void GoToSpot(Vector2 spot, PIDSettings OdometryMovementGoToSpotTurnPID, PIDSettings OdometryMovementGoToSpotMovePID);


  class OdometryMotor{
  public:
    pros::Motor* motor;
    bool reversed;
    double gearset; //18 for regular motor, 36 for torque motor, 6 for speed motor
    OdometryMotor(int port, bool reversed, pros::motor_gearset_e gearset);
    ~OdometryMotor();

    void set_brake_mode(pros::motor_brake_mode_e brakeMode);
    void move_velocity(double v);
  };



  #ifdef MECANUM_WHEEL
    void SetMotorsMecanum(OdometryMotor* frontLeft, OdometryMotor* backLeft, OdometryMotor* frontRight, OdometryMotor* backRight);
  #endif

  #if defined(MECANUM_WHEEL) ||defined(XDRIVE)
    void MoveLinear(Vector2 spot, PIDSettings LinearMoveSettings, PIDSettings LinearTurnSettings, double distanceThreshold = 1);
    void MoveLinear(Vector2 spot, Angle angle, PIDSettings LinearMoveSettings, PIDSettings LinearTurnSettings, double distanceThreshold = 1);
    namespace{
      void MoveLinearComplete(Vector2 spot, Angle angle, bool doAngleError, PIDSettings LinearMoveSettings, PIDSettings LinearTurnSettings, double distanceThreshold);
    }
  #endif
}

#endif
