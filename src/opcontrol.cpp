#include "main.h"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	okapi::Controller controller;
	okapi::Motor intake (8);
	okapi::Motor indexer (-9);
	okapi::Motor capFlipper(-3, false, okapi::AbstractMotor::gearset::red);
	okapi::Motor flywheel(-1);

	auto myChassis = okapi::ChassisControllerFactory::create(
		 2, 10, 20, 11
	);

	while (true) {
		myChassis.arcade(controller.getAnalog(okapi::ControllerAnalog::rightX), controller.getAnalog(okapi::ControllerAnalog::leftY));

		intake.controllerSet(controller.getDigital(okapi::ControllerDigital::L2) - controller.getDigital(okapi::ControllerDigital::L1));
		indexer.controllerSet(controller.getDigital(okapi::ControllerDigital::R2) - controller.getDigital(okapi::ControllerDigital::R1));
		capFlipper.controllerSet(controller.getDigital(okapi::ControllerDigital::up) - controller.getDigital(okapi::ControllerDigital::down));

		pros::delay(20);
	}
}
