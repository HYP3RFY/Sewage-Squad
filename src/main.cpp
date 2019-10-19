#include "main.h"
#include "subsystems/tray.h"
#include "odometry/odometry.h"
#include "odometry/odometryMovement.h"
#include "odometry/angle.h"
#include "odometry/mecanum.h"
#include "pidparams.h"

#define DEADZONE(x) fabs(x)<15?0:x
#define THROTTLE_FORWARD ANALOG_LEFT_Y
#define STRAFE ANALOG_LEFT_X
#define TURN_CONTROL ANALOG_RIGHT_X
int autonSelect = 1;
std::string autonName = "none";
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();

auto brake = MOTOR_BRAKE_BRAKE;

Odometry::Movement::OdometryMotor* frontLeftCMtr = new Odometry::Movement::OdometryMotor(2, false, MOTOR_GEARSET_18);
frontLeftCMtr->motor->set_brake_mode(brake);

Odometry::Movement::OdometryMotor* backLeftCMtr = new Odometry::Movement::OdometryMotor(4, false, MOTOR_GEARSET_18);
backLeftCMtr->motor->set_brake_mode(brake);

Odometry::Movement::OdometryMotor* backRightCMtr = new Odometry::Movement::OdometryMotor(3, true, MOTOR_GEARSET_18);
backRightCMtr->motor->set_brake_mode(brake);

Odometry::Movement::OdometryMotor* frontRightCMtr = new Odometry::Movement::OdometryMotor(1, true, MOTOR_GEARSET_18);
frontRightCMtr->motor->set_brake_mode(brake);

pros::ADIEncoder* leftWheelEncoder = new pros::ADIEncoder (3, 4, true);
pros::ADIEncoder* rightWheelEncoder = new pros::ADIEncoder(1, 2, false);
pros::ADIEncoder* backWheelEncoder = new pros::ADIEncoder(5, 6, true);

Odometry::SetTrackingCenterParameters(6.75,6.75,-5);
Odometry::SetTrackingWheelEncoders(leftWheelEncoder, rightWheelEncoder, backWheelEncoder);
Odometry::SetStartingPositionAndRotation(Odometry::Vector2(0, 0), Odometry::Angle::FromDegrees(0).GetRadianMeasure());
Odometry::SetWheelCircumference(2.75 * 3.14159);

Odometry::Movement::SetMotorsMecanum(frontLeftCMtr, backLeftCMtr, frontRightCMtr, backRightCMtr);

pros::delay(100);

Odometry::Init();

pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	while (true){
	 if (master.get_digital(DIGITAL_UP)){
		 autonSelect = 1;
	 }else if (master.get_digital(DIGITAL_LEFT)){
		 autonSelect = 2;
	 }else if (master.get_digital(DIGITAL_RIGHT)){
		 autonSelect = 3;
	 }else if (master.get_digital(DIGITAL_LEFT)){
		 autonSelect = 4;
	}else if (master.get_digital(DIGITAL_DOWN)){
		autonSelect = 5;
	}else if (master.get_digital(DIGITAL_Y)){
		autonSelect = 6;
	}else {
		autonSelect = 0;
 }

 //---------------------------------------------------------------------------

 if (autonSelect == 0){
	 autonName = "none";
 }else if (autonSelect == 1){
	 autonName = "Red Square";
 }else if (autonSelect == 2){
	 autonName = "Red Rectangle";
 }else if (autonSelect == 3){
	 autonName = "Blue Square";
 }else if (autonSelect == 4){
	 autonName = "Blue Rectangle";
 }else if (autonSelect == 5){
	 autonName = "Skills Blue";
 }
	pros::lcd::print(4, "%s", autonName);
	}
}

void autonomous() {
	pros::Motor leftFrontMtr(2);
	pros::Motor rightFrontMtr(1);
	pros::Motor leftBackMtr(4);
	pros::Motor rightBackMtr(3);

	leftFrontMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightFrontMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	leftBackMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightBackMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
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


  autonName = "none";
  arm.move_absolute(550, 100);
	pros::delay(1000);
	arm.move_absolute(-270, 100);
	pros::delay(1000);

  if (autonSelect == 0){
//red squareq
  }else if(autonSelect == 1){
		leftIntake.move_velocity(200);
		rightIntake.move_velocity(-200);
		Odometry::Movement::MoveLinear(Odometry::Vector2(29,0), Odometry::Angle::FromDegrees(0), PIDSettings(1.6,0.1,-.1), OdometryMovementGoToSpotTurnPID);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		Odometry::Movement::MoveLinear(Odometry::Vector2(18,.35), Odometry::Angle::FromDegrees(120), PIDSettings(3.2,0.1,-.5), PIDSettings(2.5,0.1,-.3));
		Odometry::Movement::MoveLinear(Odometry::Vector2(5.3,10), Odometry::Angle::FromDegrees(120), PIDSettings(3.2,0.1,-.5), PIDSettings(2.8,0.1,-.3));

		Subsystems::Tray::MoveTrayToPosition(Subsystems::Tray::TrayPosition::Push);
		pros::delay(200);
		leftIntake.move_velocity(-15);
		rightIntake.move_velocity(15);
		leftFrontMtr.move_velocity(-10);
		rightFrontMtr.move_velocity(10);
		leftBackMtr.move_velocity(-10);
		rightBackMtr.move_velocity(10);
		pros::delay(3000);
		Subsystems::Tray::MoveTrayToPosition(Subsystems::Tray::TrayPosition::Storage);

		//Odometry::Movement::GoToSpot(Odometry::Vector2(0,14), OdometryMovementGoToSpotTurnPID, PIDSettings(1.5,0.1,-1));
//red rec
  }else if(autonSelect == 2){
//blue square
  }else if(autonSelect == 3){
//blue rec
  }else if(autonSelect == 4){
//skills
  }else if(autonSelect == 5){

  }
}

void opcontrol() {
	pros::delay(100);
	//DriveTrain
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor leftFrontMtr(2);
	pros::Motor rightFrontMtr(1);
	pros::Motor leftBackMtr(4);
	pros::Motor rightBackMtr(3);

	leftFrontMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightFrontMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	leftBackMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightBackMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	//Intake Rollers
	pros::Motor leftIntake(15);
	pros::Motor rightIntake(14);
	pros::ADIPotentiometer pot = pros::ADIPotentiometer('h');

	leftIntake.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightIntake.set_brake_mode(MOTOR_BRAKE_HOLD);
	//Tray and Lift Arm
	pros::Motor trayMotor(11);
	pros::Motor arm(12);
	arm.set_gearing(MOTOR_GEARSET_36);
	arm.set_brake_mode(MOTOR_BRAKE_HOLD);
	trayMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	trayMotor.set_gearing(MOTOR_GEARSET_36);
	arm.set_brake_mode(MOTOR_BRAKE_HOLD);
	//Initialize Tray Code
	Subsystems::Tray::Init(&trayMotor, &pot);
	Subsystems::Tray::MoveTrayToPosition(Subsystems::Tray::TrayPosition::Storage);

	bool goBack = false;
	bool liftToggle = false;

	//Main Motion Loop
	while (true) {
		//Display Pot Value
		//pros::lcd::print(0, "%d", pot.get_value());
		pros::lcd::print(4, "%f", arm.get_position());
		//Display Odometry values
		pros::lcd::print(1, "%s", Odometry::GetRobotPosition().ToString());
		pros::lcd::print(2, "%f", Odometry::GetRobotRotation()/0.0174533);
		//AutonSelector test
		if (master.get_digital(DIGITAL_Y)){
			void competition_initialize();
		}

		//Drive values
		int throttle = DEADZONE(master.get_analog(THROTTLE_FORWARD) * (200/128.0));
		int strafe = DEADZONE(master.get_analog(STRAFE) * (-200/128.0));
		int turn = DEADZONE(master.get_analog(TURN_CONTROL) * 0.9);

		//Drive Train
		leftFrontMtr.move_velocity(turn + throttle - strafe);
		leftBackMtr.move_velocity(turn + throttle + strafe);
		rightFrontMtr.move_velocity(turn - throttle - strafe);
		rightBackMtr.move_velocity(turn - throttle + strafe);

		//Stacks Cubes and backs up automatically
		if (pot.get_value() >= 1570 && goBack) {
			leftIntake.move_velocity(-15);
			rightIntake.move_velocity(15);

			leftFrontMtr.move_velocity(-10);
			rightFrontMtr.move_velocity(10);
			leftBackMtr.move_velocity(-10);
			rightBackMtr.move_velocity(10);

			pros::delay(3000);
			Subsystems::Tray::MoveTrayToPosition(Subsystems::Tray::TrayPosition::Storage);
			goBack=false;
		}

	//Potentiometer value for bool value to stack cubes
		if (pot.get_value() <= 1400) goBack=true;
	//Lift mechanism, pulls tray up a bit so arms can lift
		if (master.get_digital(DIGITAL_X)){
			Subsystems::Tray::MoveTrayToPosition(Subsystems::Tray::TrayPosition::Stack);
			liftToggle = true;
			pros::delay(20);
		}
		//ARM cODE
		if (master.get_digital(DIGITAL_R1)){
			arm.move_velocity(200);
		} else if (master.get_digital(DIGITAL_R2)){
			arm.move_velocity(-200);
		} else {
			arm.move_velocity(0);
		}
		//Arm Code
		if (liftToggle == true){
			if (master.get_digital(DIGITAL_X)){
				liftToggle = false;
			}
		}

		//Push Tray to Stack Cubes
		if (master.get_digital(DIGITAL_A)){
			Subsystems::Tray::MoveTrayToPosition(Subsystems::Tray::TrayPosition::Push);

			//Store Cubes
		} else if (master.get_digital(DIGITAL_B)) {
			Subsystems::Tray::MoveTrayToPosition(Subsystems::Tray::TrayPosition::Storage);
		}

		//Intake Rollers
		if (master.get_digital(DIGITAL_L1)){
			leftIntake.move_velocity(200);
			rightIntake.move_velocity(-200);
		} else if (master.get_digital(DIGITAL_L2)){
			leftIntake.move_velocity(-200);
			rightIntake.move_velocity(200);
		} else{
			leftIntake.move_velocity(0);
			rightIntake.move_velocity(0);
		}

		pros::delay(20);
	}
	}
