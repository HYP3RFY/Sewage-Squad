#include "main.h"

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
	pros::lcd::set_text(1, "   _____ _       _____ _            __   __        ");
	pros::lcd::set_text(2, "  / ____| |     |  __ (_)           \\ \\ / /        ");
	pros::lcd::set_text(3, " | (___ | |_    | |__) | _   _ ___   \\ V /         ");
	pros::lcd::set_text(4, "  \\___ \\| __|   |  ___/ | | | / __|   > <          ");
	pros::lcd::set_text(5, "  ____) | |_ _  | |   | | |_| \\__ \\  / . \\         ");
	pros::lcd::set_text(6, " |_____/ \\__(_) |_|   |_|\\__,_|___/ /_/ \\_\\        ");
	pros::lcd::set_text(7, " |  __ \\     | |         | |    (_)                ");
	pros::lcd::set_text(8, " | |__) |___ | |__   ___ | |     _  ___  _ __  ___ ");
	pros::lcd::set_text(9, " |  _  // _ \\| '_ \\ / _ \\| |    | |/ _ \\| '_ \\/ __|");
	pros::lcd::set_text(10, " | | \\ \\ (_) | |_) | (_) | |____| | (_) | | | \\__ \\");
	pros::lcd::set_text(11, " |_|  \\_\\___/|_.__/ \\___/|______|_|\\___/|_| |_|___/");

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
void competition_initialize() {}
