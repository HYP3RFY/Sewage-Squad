#include "main.h"
#include "subsystems/tray.h"

void opcontrol() {

#include "main.h"
#define DEADZONE(x) fabs(x)<15?0:x
#define THROTTLE_FORWARD ANALOG_LEFT_Y
#define STRAFE ANALOG_LEFT_X
#define TURN_CONTROL ANALOG_RIGHT_X

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

bool goBack = false;
bool liftToggle = false;

//Main Motion Loop
while (true) {
	//Display Pot Value
	//pros::lcd::print(0, "%d", pot.get_value());
	pros::lcd::print(4, "%f", arm.get_position());
	//Display Odometry values
	//pros::lcd::print(1, "%s", Odometry::GetRobotPosition().ToString());
	//pros::lcd::print(2, "%f", Odometry::GetRobotRotation()/0.0174533);

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
