#include "main.h"
// #include "autism.h"
bool selected = false;

void left_button() {
  if (!selected) {
    lcdCounter--;
    if (lcdCounterGet() < 0) {
      lcdCounterSet(0);
    }
  }
}
void center_button() {
  if (!selected) {
    selected = true;
  }
}
void right_button() {
  if (!selected) {
    lcdCounter++;
    if (lcdCounterGet() > 4) {
      lcdCounterSet(4);
    }
  }
}
std::string convert (int arg) {
  switch (arg) {
    case 0:
      return "No Auton";
    case 1:
      return "Blue Front";
    case 2:
      return "Red Front";
    case 3:
      return "Blue Back";
    case 4:
      return "Red Back";
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
  pros::lcd::register_btn0_cb(left_button);
  pros::lcd::register_btn1_cb(center_button);
  pros::lcd::register_btn2_cb(right_button);

  while (!selected) {
    pros::lcd::set_text(0, convert(lcdCounterGet()));
    pros::delay(20);
  }

	pros::lcd::set_text(0, convert(lcdCounterGet()) + " (SELECTED)");
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
