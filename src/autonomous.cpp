#include "main.h"
#include "odometry/odometry.h"
#include "odometry/odometryMovement.h"
#include "odometry/angle.h"
#include "odometry/mecanum.h"
#include "pidparams.h"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  pros::Motor trayMotor(11);
  pros::Motor arm(12);
  arm.set_gearing(MOTOR_GEARSET_36);
  arm.set_brake_mode(MOTOR_BRAKE_HOLD);
  trayMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
  trayMotor.set_gearing(MOTOR_GEARSET_36);
  arm.set_brake_mode(MOTOR_BRAKE_HOLD);
  pros::Motor leftIntake(15);
  pros::Motor rightIntake(14);
  pros::ADIPotentiometer pot = pros::ADIPotentiometer('h');
  leftIntake.set_brake_mode(MOTOR_BRAKE_HOLD);
  rightIntake.set_brake_mode(MOTOR_BRAKE_HOLD);

  arm.move_relative(680, 100);

  //Odometry::Movement::GoToSpot(Odometry::Vector2(0,0), OdometryMovementGoToSpotTurnPID, OdometryMovementGoToSpotMovePID);
}
