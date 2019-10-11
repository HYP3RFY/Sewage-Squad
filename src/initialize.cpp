#include "main.h"

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
