#include "main.h"
#include "odometry/odometry.h"
#include "odometry/odometryMovement.h"
#include "odometry/angle.h"
#include "odometry/mecanum.h"
#include "autonomousSelector.h"

lv_obj_t * myButton;
lv_obj_t * myButtonLabel;
lv_obj_t * myLabel;

lv_style_t myButtonStyleREL; //relesed style
lv_style_t myButtonStylePR; //pressed style

static lv_res_t btn_click_action(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons

    if(id == 0)
    {
        char buffer[100];
		sprintf(buffer, "button was clicked %i milliseconds from start", pros::millis());
		lv_label_set_text(myLabel, buffer);
    }

    return LV_RES_OK;
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	/*
	pros::lcd::set_text(2, " /   _____/\\______   \\   \\/  /");
	pros::lcd::set_text(1, "  ___________________ ____  ___");
	pros::lcd::set_text(3, " \\_____  \\  |     ___/ \\     / ");
	pros::lcd::set_text(4, " /        \\ |    |     /     \\ ");
	pros::lcd::set_text(5, "/_______  / |____|    /___/\\  \\");
	pros::lcd::set_text(6, "        \\/                  \\_/");
	*/

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
  Odometry::SetStartingPositionAndRotation(Odometry::Vector2(0, 0), Odometry::Angle::FromDegrees(90).GetRadianMeasure());
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
   Autonomous::SelectAutonomousType();
}
