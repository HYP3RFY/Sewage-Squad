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

//Check here for erros:
using namespace pros;
using namespace Subsystems;

bool armUnfold = true;
std::string autonName = "Red Square";


Controller master(E_CONTROLLER_MASTER);
Motor leftFrontMtr(2);
Motor rightFrontMtr(1);
Motor leftBackMtr(4);
Motor rightBackMtr(3);
Motor trayMotor(11);
Motor arm(12);
Motor leftIntake(15);
Motor rightIntake(14);
ADIPotentiometer pot = ADIPotentiometer('h');



void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		lcd::set_text(2, "I was pressed!");
	} else {
		lcd::clear_line(2);
	}
}

void initialize() {
	lcd::initialize();

auto brake = MOTOR_BRAKE_BRAKE;

Odometry::Movement::OdometryMotor* frontLeftCMtr = new Odometry::Movement::OdometryMotor(2, false, MOTOR_GEARSET_18);
frontLeftCMtr->motor->set_brake_mode(brake);

Odometry::Movement::OdometryMotor* backLeftCMtr = new Odometry::Movement::OdometryMotor(4, false, MOTOR_GEARSET_18);
backLeftCMtr->motor->set_brake_mode(brake);

Odometry::Movement::OdometryMotor* backRightCMtr = new Odometry::Movement::OdometryMotor(3, true, MOTOR_GEARSET_18);
backRightCMtr->motor->set_brake_mode(brake);

Odometry::Movement::OdometryMotor* frontRightCMtr = new Odometry::Movement::OdometryMotor(1, true, MOTOR_GEARSET_18);
frontRightCMtr->motor->set_brake_mode(brake);

ADIEncoder* leftWheelEncoder = new ADIEncoder (3, 4, true);
ADIEncoder* rightWheelEncoder = new ADIEncoder(1, 2, false);
ADIEncoder* backWheelEncoder = new ADIEncoder(5, 6, true);

Odometry::SetTrackingCenterParameters(6.75,6.75,-5);
Odometry::SetTrackingWheelEncoders(leftWheelEncoder, rightWheelEncoder, backWheelEncoder);
Odometry::SetStartingPositionAndRotation(Odometry::Vector2(0, 0), Odometry::Angle::FromDegrees(0).GetRadianMeasure());
Odometry::SetWheelCircumference(2.75 * 3.14159);

Odometry::Movement::SetMotorsMecanum(frontLeftCMtr, backLeftCMtr, frontRightCMtr, backRightCMtr);

delay(100);

Odometry::Init();

Tray::Init(&trayMotor, &pot);

lcd::register_btn1_cb(on_center_button);
}


void disabled() {
}

void competition_initialize() {
//put auton selector here
	Controller master(E_CONTROLLER_MASTER);
	lcd::print(1, "%s", autonName);

 if (master.get_digital(DIGITAL_R1)){
	 autonName = "Red Square";
 }else if (master.get_digital(DIGITAL_R2)){
	 autonName = "Red Rectangle";
 }else if (master.get_digital(DIGITAL_L1)){
	 autonName = "Blue Square";
 }else if (master.get_digital(DIGITAL_L2)){
	 autonName = "Blue Rectangle";
 }else if (master.get_digital(DIGITAL_X)){
	 autonName = "Skills";
 }else {
	 autonName = "none";
 }
delay(20);
}

void autonomous() {
	Motor leftFrontMtr(2);
	Motor rightFrontMtr(1);
	Motor leftBackMtr(4);
	Motor rightBackMtr(3);
	Motor trayMotor(11);
  Motor arm(12);
	Motor leftIntake(15);
	Motor rightIntake(14);

	leftFrontMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightFrontMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	leftBackMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightBackMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	arm.set_gearing(MOTOR_GEARSET_36);
  arm.set_brake_mode(MOTOR_BRAKE_HOLD);
  trayMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
  trayMotor.set_gearing(MOTOR_GEARSET_36);
  arm.set_brake_mode(MOTOR_BRAKE_HOLD);


	armUnfold = false;
  ADIPotentiometer pot =ADIPotentiometer('h');
  leftIntake.set_brake_mode(MOTOR_BRAKE_HOLD);
  rightIntake.set_brake_mode(MOTOR_BRAKE_HOLD);

  arm.move_velocity(100);
	trayMotor.move_velocity(-10);
	delay(250);
	arm.move_velocity(40);
	trayMotor.move_velocity(-10);
	delay(250);
	arm.move_velocity(100);
	delay(1600);
	arm.move_velocity(-100);
	delay(400);
	arm.move_velocity(0);


//Put Auton Code Here:
	if (autonName == "Red Square"){
		delay(20);
		Tray::MoveTrayToPosition(Tray::TrayPosition::Storage);
		leftIntake.move_velocity(200);
		rightIntake.move_velocity(-200);
		Odometry::Movement::MoveLinear(Odometry::Vector2(33,0),Odometry::Angle::FromDegrees(0),PIDSettings(3.2,.2,-.05),PIDSettings(2,.01,-.15));
		Odometry::Movement::MoveLinear(Odometry::Vector2(38,6),Odometry::Angle::FromDegrees(0),PIDSettings(6,.3,-.0001),PIDSettings(5,.2,-.05),1.5);
		Odometry::Movement::MoveLinear(Odometry::Vector2(45,6),Odometry::Angle::FromDegrees(0),PIDSettings(6,.3,-.005),PIDSettings(5,.2,-.05),1.5);
		Odometry::Movement::MoveLinear(Odometry::Vector2(15.5,12.7),Odometry::Angle::FromDegrees(-180),PIDSettings(7,.2,-.05),PIDSettings(5,.2,-.05));
		Odometry::Movement::MoveLinear(Odometry::Vector2(9,12.7),Odometry::Angle::FromDegrees(-180),PIDSettings(7,.2,-.05),PIDSettings(5,.2,-.05));
		Odometry::Movement::MoveLinear(Odometry::Vector2(5.3,-8.8),Odometry::Angle::FromDegrees(-135),PIDSettings(7,.2,-.05),PIDSettings(5,.2,-.05));
	}else if(autonName == "Red Rectangle"){

	}else if(autonName == "Blue Square"){

	}else if(autonName == "Blue Rectangle"){

	}else if(autonName == "Skills"){
		leftIntake.move_velocity(200);
		rightIntake.move_velocity(-200);
		Odometry::Movement::MoveLinear(Odometry::Vector2(33,0),Odometry::Angle::FromDegrees(0),PIDSettings(3.2,.08,-.25),PIDSettings(2,.01,-.15));
		Odometry::Movement::MoveLinear(Odometry::Vector2(33,-6),Odometry::Angle::FromDegrees(0),PIDSettings(6,.08,-.01),PIDSettings(3,.01,-.15));
		Odometry::Movement::MoveLinear(Odometry::Vector2(45,-6),Odometry::Angle::FromDegrees(0),PIDSettings(5,.08,-.1),PIDSettings(2,.01,-.15));
		Odometry::Movement::MoveLinear(Odometry::Vector2(9.7,-10),Odometry::Angle::FromDegrees(0),PIDSettings(6,.08,-.1),PIDSettings(2,.01,-.15));
		Odometry::Movement::MoveLinear(Odometry::Vector2(11,-21),Odometry::Angle::FromDegrees(0),PIDSettings(6,.08,-.1),PIDSettings(2,.01,-.15));
		Odometry::Movement::MoveLinear(Odometry::Vector2(33,-21),Odometry::Angle::FromDegrees(0),PIDSettings(3.2,.08,-.25),PIDSettings(3,.01,-.15));
	}
	delay(20);
}

//------------------------------------------------------------------------------

void opcontrol() {
	delay(100);
	//DriveTrain
	leftFrontMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightFrontMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	leftBackMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightBackMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	leftIntake.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightIntake.set_brake_mode(MOTOR_BRAKE_HOLD);
	//Tray and Lift Arm
	arm.set_gearing(MOTOR_GEARSET_36);
	arm.set_brake_mode(MOTOR_BRAKE_HOLD);
	trayMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	trayMotor.set_gearing(MOTOR_GEARSET_36);
	//Initialize Tray Code

	delay(100);
//------------------------------------------------------------------------------
	if (armUnfold == true){
//put unfolding arms here
	}

	bool moveBack = false;
	Tray::MoveTrayToPosition(Tray::TrayPosition::Push);
	delay(10);
	Tray::MoveTrayToPosition(Tray::TrayPosition::Storage);

	bool goBack = true;
	bool liftToggle = false;
	//Main Motion Loop
	while (true) {
		//Display Pot Value
		//pros::lcd::print(0, "%d", pot.get_value());
		//lcd::print(4, "%f", arm.get_position());

		//Display Odometry values
		lcd::print(1, "%s", Odometry::GetRobotPosition().ToString());
		lcd::print(2, "%f", Odometry::GetRobotRotation()/0.0174533);
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
		if (pot.get_value() >= 1600 && goBack) {


			delay(3000);
			Tray::MoveTrayToPosition(Tray::TrayPosition::Storage);
			goBack=false;
		}

	//Potentiometer value for bool value to stack cubes
	//Lift mechanism, pulls tray up a bit so arms can lift
		if (master.get_digital(DIGITAL_X)){
			Tray::MoveTrayToPosition(Tray::TrayPosition::Stack);
			liftToggle = true;
			delay(20);
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
			if (pot.get_value() >= 980){
				liftToggle = false;
			}
		}

		//Push Tray to Stack Cubes
		if (master.get_digital(DIGITAL_A)){
			Tray::MoveTrayToPosition(Tray::TrayPosition::Push);
			//Store Cubes
		} else if (master.get_digital(DIGITAL_B)) {
			Tray::MoveTrayToPosition(Tray::TrayPosition::Storage);
		} else if (master.get_digital_new_press(DIGITAL_Y)){
			while (master.get_digital_new_press(DIGITAL_Y) == false) {
				leftIntake.move_velocity(-15);
				rightIntake.move_velocity(15);
				leftFrontMtr.move_velocity(-10);
				rightFrontMtr.move_velocity(10);
				leftBackMtr.move_velocity(-10);
				rightBackMtr.move_velocity(10);
				}
				delay(20);
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

		delay(20);
	}
	}

	//Skills run starts with one orange cube in intake
	//Unpack arms
	//Start robot next to middle tower on blue side
	//Stack orange cube in tower
	//Grab first 2 of the L-shaped stack (purple + orange)
	//Grab first 2 of straight horizontal stack (purple + orange)
	//Stack them in small scoring zone
	//Grab nearest purple
	//Stack in allied tower
	//Grab 4 purple cubes from middle and stack them in big zone
	//Grab purple cube next to right middle tower and put it in the tower
	//Go to red side medium tower and stack a purple cube
	//Grab other purple cube and collect 3 more purples
	//Stack them
	//Run should end with 2 orange cubes and 10 purple cubes, 44 points
