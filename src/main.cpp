#include "main.h"
#include "subsystems/tray.h"
#include "odometry/odometry.h"
#include "odometry/odometryMovement.h"
#include "odometry/angle.h"
#include "odometry/mecanum.h"
#include "pidparams.h"
#include <thread>

#define DEADZONE(x) fabs(x)<15?0:x
#define THROTTLE_FORWARD ANALOG_LEFT_Y
#define STRAFE ANALOG_LEFT_X
#define TURN_CONTROL ANALOG_RIGHT_X
std::string autonName;
int trayPos;
//Check here for erros:
using namespace pros;
using namespace Subsystems;

bool armUnfold = true;
int autonNumber;
float slow = 1;

Controller master(E_CONTROLLER_MASTER);
Motor leftFrontMtr(2);
Motor rightFrontMtr(1);
Motor leftBackMtr(4);
Motor rightBackMtr(3);
Motor trayMotor(11);
Motor arm(12);
Motor leftIntake(10);//14 15 dead
Motor rightIntake(9);
ADIPotentiometer pot = ADIPotentiometer('h');
ADIPotentiometer autonSelector = ADIPotentiometer('g');


void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		lcd::set_text(2, "I was pressed!");
	} else {
		lcd::clear_line(2);
	}
}

pros::Task* SelectAutonTask;

void AutonSelector(void* params){

	while(true){
		if (autonSelector.get_value() <= 1000) {
			autonName = "None";
		}else if (autonSelector.get_value() <= 2000) {
			autonName = "Red Square";
		}else if (autonSelector.get_value() <= 3000) {
			autonName = "Skills";
		}else if (autonSelector.get_value() <= 4000) {
			autonName = "Blue Square";
		}
	lcd::print(6, "%s", autonName);
	delay(50);
	}
}

void autonSelectorMethod() {
	bool autonSelectRunning = true;
	//spawn thread
	std::string SelectAutonTaskName("Auton Selector Thread");
	SelectAutonTask = new pros::Task(AutonSelector, &SelectAutonTaskName);
}

void initialize() {
	lcd::initialize();

	trayMotor.set_reversed(true);
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
  autonSelectorMethod();
}

void competition_initialize() {
  autonSelectorMethod();
}

void autonomous() {
	leftFrontMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightFrontMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	leftBackMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightBackMtr.set_brake_mode(MOTOR_BRAKE_BRAKE);
	arm.set_gearing(MOTOR_GEARSET_36);
  arm.set_brake_mode(MOTOR_BRAKE_HOLD);
  trayMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
  trayMotor.set_gearing(MOTOR_GEARSET_36);
  arm.set_brake_mode(MOTOR_BRAKE_COAST);


	armUnfold = false;
  ADIPotentiometer pot = ADIPotentiometer('h');
  leftIntake.set_brake_mode(MOTOR_BRAKE_HOLD);
  rightIntake.set_brake_mode(MOTOR_BRAKE_HOLD);

//PUT UNFLIP HERE
	Tray::MoveTrayToPosition(Tray::TrayPosition::Stack);
	leftIntake.move_velocity(-100);
	rightIntake.move_velocity(100);
	delay(1300);
	Tray::MoveTrayToPosition(Tray::TrayPosition::Storage);
	leftIntake.move_velocity(-100);
	rightIntake.move_velocity(100);
	delay(1000);
//Put Auton Code Here:
delay(20);
if (autonSelector.get_value() <= 1000) {
	autonName = "None";
	autonNumber = 0;
	delay(20);
}else if (autonSelector.get_value() <= 2000) {
	autonName = "Red Square";
	autonNumber = 1;
	delay(20);
}else if (autonSelector.get_value() <= 3000) {
	autonName = "Skills";
	autonNumber = 2;
	delay(20);
}else if (autonSelector.get_value() <= 4000) {
	autonName = "Blue Square";
	autonNumber = 3;
	delay(20);
}

	if 	(autonNumber == 1){
		delay(20);
		leftFrontMtr.move_velocity(-200);
		leftBackMtr.move_velocity(200);
		rightFrontMtr.move_velocity(-200);
		rightBackMtr.move_velocity(200);

		leftIntake.move_velocity(200);
		rightIntake.move_velocity(-200);
		Odometry::Movement::MoveLinear(Odometry::Vector2(33,0),Odometry::Angle::FromDegrees(0),PIDSettings(3.2,.2,-.05),PIDSettings(2,.01,-.15),1.5);
		Odometry::Movement::MoveLinear(Odometry::Vector2(35,8),Odometry::Angle::FromDegrees(0),PIDSettings(7,.3,-.0001),PIDSettings(5,.2,-.0005),2.5);
		Odometry::Movement::MoveLinear(Odometry::Vector2(44,8),Odometry::Angle::FromDegrees(0),PIDSettings(6,.25,-.005),PIDSettings(5,.2,-.05),3);
		Odometry::Movement::MoveLinear(Odometry::Vector2(20,-5.7),Odometry::Angle::FromDegrees(-135),PIDSettings(7,.5,-.0005),PIDSettings(5,.3,-.05),3);
		delay(60);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		delay(60);
		leftIntake.move_velocity(-100);
		rightIntake.move_velocity(100);
		delay(460);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		delay(20);
		leftFrontMtr.move_velocity(80);
		rightFrontMtr.move_velocity(-80);
		leftBackMtr.move_velocity(80);
		rightBackMtr.move_velocity(-80);
		Tray::MoveTrayToPosition(Tray::TrayPosition::Storage);
		delay(1200);
		leftFrontMtr.move_velocity(0);
		rightFrontMtr.move_velocity(0);
		leftBackMtr.move_velocity(0);
		rightBackMtr.move_velocity(0);
		delay(20);
		leftIntake.move_velocity(15);
		rightIntake.move_velocity(-15);
		delay(800);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		delay(20);
		leftIntake.move_velocity(-15);
		rightIntake.move_velocity(15);
		leftFrontMtr.move_velocity(-10);
		rightFrontMtr.move_velocity(10);
		leftBackMtr.move_velocity(-10);
		rightBackMtr.move_velocity(10);
		delay(4000);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		leftFrontMtr.move_velocity(0);
		rightFrontMtr.move_velocity(0);
		leftBackMtr.move_velocity(0);
		rightBackMtr.move_velocity(0);
	  } else if(autonNumber == 2){
		delay(20);
		leftFrontMtr.move_velocity(200);
		leftBackMtr.move_velocity(-200);
		rightFrontMtr.move_velocity(200);
		rightBackMtr.move_velocity(-200);
		/*leftIntake.move_velocity(200);
		rightIntake.move_velocity(-200);
		Odometry::Movement::MoveLinear(Odometry::Vector2(33,0),Odometry::Angle::FromDegrees(0),PIDSettings(3.2,.2,-.05),PIDSettings(2,.01,-.15),1.5);
		Odometry::Movement::MoveLinear(Odometry::Vector2(35,-8),Odometry::Angle::FromDegrees(0),PIDSettings(7,.3,-.0001),PIDSettings(5,.2,-.0005),2.5);
		Odometry::Movement::MoveLinear(Odometry::Vector2(44,-8),Odometry::Angle::FromDegrees(0),PIDSettings(6,.25,-.005),PIDSettings(5,.2,-.05),3);
		Odometry::Movement::MoveLinear(Odometry::Vector2(20,5.7),Odometry::Angle::FromDegrees(135),PIDSettings(7,.5,-.0005),PIDSettings(5,.3,-.05),3);
		delay(60);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		delay(60);
		leftIntake.move_velocity(-100);
		rightIntake.move_velocity(100);
		delay(460);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		delay(20);
		leftFrontMtr.move_velocity(80);
		rightFrontMtr.move_velocity(-80);
		leftBackMtr.move_velocity(80);
		rightBackMtr.move_velocity(-80);
		Tray::MoveTrayToPosition(Tray::TrayPosition::Storage);
		delay(1200);
		leftFrontMtr.move_velocity(0);
		rightFrontMtr.move_velocity(0);
		leftBackMtr.move_velocity(0);
		rightBackMtr.move_velocity(0);
		delay(20);
		leftIntake.move_velocity(15);
		rightIntake.move_velocity(-15);
		delay(800);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		delay(20);
		leftIntake.move_velocity(-15);
		rightIntake.move_velocity(15);
		leftFrontMtr.move_velocity(-10);
		rightFrontMtr.move_velocity(10);
		leftBackMtr.move_velocity(-10);
		rightBackMtr.move_velocity(10);
		delay(4000);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		leftFrontMtr.move_velocity(0);
		rightFrontMtr.move_velocity(0);
		leftBackMtr.move_velocity(0);
		rightBackMtr.move_velocity(0);*/
	}else if(autonNumber == 3){
		delay(20);
		leftFrontMtr.move_velocity(200);
		leftBackMtr.move_velocity(-200);
		rightFrontMtr.move_velocity(200);
		rightBackMtr.move_velocity(-200);
	/*	leftIntake.move_velocity(200);
		rightIntake.move_velocity(-200);
		Odometry::Movement::MoveLinear(Odometry::Vector2(33,0),Odometry::Angle::FromDegrees(0),PIDSettings(3.2,.2,-.05),PIDSettings(2,.01,-.15),1.5);
		Odometry::Movement::MoveLinear(Odometry::Vector2(35,-8),Odometry::Angle::FromDegrees(0),PIDSettings(7,.3,-.0001),PIDSettings(5,.2,-.0005),2.5);
		Odometry::Movement::MoveLinear(Odometry::Vector2(44,-8),Odometry::Angle::FromDegrees(0),PIDSettings(6,.25,-.005),PIDSettings(5,.2,-.05),3);
		Odometry::Movement::MoveLinear(Odometry::Vector2(20,5.7),Odometry::Angle::FromDegrees(135),PIDSettings(7,.5,-.0005),PIDSettings(5,.3,-.05),3);
		delay(60);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		delay(60);
		leftIntake.move_velocity(-100);
		rightIntake.move_velocity(100);
		delay(460);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		delay(20);
		leftFrontMtr.move_velocity(80);
		rightFrontMtr.move_velocity(-80);
		leftBackMtr.move_velocity(80);
		rightBackMtr.move_velocity(-80);
		Tray::MoveTrayToPosition(Tray::TrayPosition::Storage);
		delay(1200);
		leftFrontMtr.move_velocity(0);
		rightFrontMtr.move_velocity(0);
		leftBackMtr.move_velocity(0);
		rightBackMtr.move_velocity(0);
		delay(20);
		leftIntake.move_velocity(15);
		rightIntake.move_velocity(-15);
		delay(800);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		delay(20);
		leftIntake.move_velocity(-15);
		rightIntake.move_velocity(15);
		leftFrontMtr.move_velocity(-10);
		rightFrontMtr.move_velocity(10);
		leftBackMtr.move_velocity(-10);
		rightBackMtr.move_velocity(10);
		delay(4000);
		leftIntake.move_velocity(0);
		rightIntake.move_velocity(0);
		leftFrontMtr.move_velocity(0);
		rightFrontMtr.move_velocity(0);
		leftBackMtr.move_velocity(0);
		rightBackMtr.move_velocity(0);*/
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
		Tray::MoveTrayToPosition(Tray::TrayPosition::Stack);
		leftIntake.move_velocity(-100);
		rightIntake.move_velocity(100);
		delay(1300);
		Tray::MoveTrayToPosition(Tray::TrayPosition::Storage);
		leftIntake.move_velocity(-100);
		rightIntake.move_velocity(100);
		delay(1000);
	}

	bool goBack = true;
	//Main Motion Loop

	while (true) {
		//Display Pot Value
		//pros::lcd::print(0, "%d", pot.get_value());


		//Display Odometry values
		lcd::print(1, "%s", Odometry::GetRobotPosition().ToString());
		lcd::print(2, "%f", Odometry::GetRobotRotation()/0.0174533);
		lcd::print(6, "%d", autonSelector.get_value());
		//Drive values
		int throttle = DEADZONE(master.get_analog(THROTTLE_FORWARD) * (200/128.0));
		int strafe = DEADZONE(master.get_analog(STRAFE) * (-200/128.0));
		int turn = DEADZONE(master.get_analog(TURN_CONTROL) * 0.9);

		//Drive Train

		leftFrontMtr.move_velocity((turn + throttle - strafe) * slow);
		leftBackMtr.move_velocity((turn + throttle + strafe) * slow);
		rightFrontMtr.move_velocity((turn - throttle - strafe) * slow);
		rightBackMtr.move_velocity((turn - throttle + strafe) * slow);

		if (master.get_digital_new_press(DIGITAL_X)){
				slow = 1;
			} else if (master.get_digital_new_press(DIGITAL_UP)) {
				slow = .2;
			}

		//ARM cODE
		if (master.get_digital(DIGITAL_R1)){
			arm.move_velocity(200);
		} else if (master.get_digital(DIGITAL_R2)){
			arm.move_velocity(-200);
		} else {
			arm.move_velocity(0);
		}


		if (master.get_digital(DIGITAL_B)) trayPos = 1;
		if (master.get_digital(DIGITAL_A)) trayPos = 2;
		//Push Tray to Stack Cubes
			//Store Cubes
		if (trayPos == 1) {
			delay(20);
			Tray::MoveTrayToPosition(Tray::TrayPosition::Storage);
			delay(20);
		}

		if (master.get_digital_new_press(DIGITAL_Y)){
			while (master.get_digital_new_press(DIGITAL_Y) == false) {
				leftIntake.move_velocity(-25);
				rightIntake.move_velocity(25);
				leftFrontMtr.move_velocity(-10);
				rightFrontMtr.move_velocity(10);
				leftBackMtr.move_velocity(-10);
				rightBackMtr.move_velocity(10);
				}
				delay(20);
			}
			if (trayPos == 2){
				delay(20);
				Tray::MoveTrayToPosition(Tray::TrayPosition::Stack);
				lcd::print(8, "%s", "A ran");
				delay(20);
			} else if (trayPos == 1) {
					delay(20);
					Tray::MoveTrayToPosition(Tray::TrayPosition::Storage);
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
