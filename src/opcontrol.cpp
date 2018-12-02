#include "main.h"

//TODO: run prosv5 mut in terminal

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

okapi::Controller controller;

void flywheelTask(void* param);

void opcontrol() {

	std::string text("PROS");
	pros::Task my_task(flywheelTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "flywheelTask");

	okapi::Motor indexer (9, true, okapi::AbstractMotor::gearset::red);
	okapi::Motor capFlipper(3, true, okapi::AbstractMotor::gearset::red);

	auto myChassis = okapi::ChassisControllerFactory::create({2, 11}, {10, 20});

	while (true) {
		myChassis.arcade(controller.getAnalog(okapi::ControllerAnalog::rightX), controller.getAnalog(okapi::ControllerAnalog::leftY));
		indexer.controllerSet(controller.getDigital(okapi::ControllerDigital::R2) - controller.getDigital(okapi::ControllerDigital::R1));
		capFlipper.controllerSet(controller.getDigital(okapi::ControllerDigital::up) - controller.getDigital(okapi::ControllerDigital::down));

		pros::delay(20);
	}
}

void flywheelTask(void* param) {
	okapi::Motor flywheelTop(7, true, okapi::AbstractMotor::gearset::green);
	okapi::Motor flywheelBot(5, false, okapi::AbstractMotor::gearset::green);
	okapi::ADIEncoder encoder('H', 'G', false);

	int targetRPM = 2000;
	int curRPM;
	encoder.reset();
  while (true) {
		/*
		curRPM = encoder.get() * 100/3;
		encoder.reset();

		if (targetRPM < curRPM) {
			flywheelTop.moveVelocity(100);
			flywheelBot.moveVelocity(100);
		}
		else {
			flywheelTop.moveVelocity(200);
			flywheelBot.moveVelocity(200);
		}

		//flywheel1.controllerSet(controller.getDigital(okapi::ControllerDigital::L2) - controller.getDigital(okapi::ControllerDigital::L1));
		//flywheel2.controllerSet(controller.getDigital(okapi::ControllerDigital::L2) - controller.getDigital(okapi::ControllerDigital::L1));
		*/
		flywheelTop.moveVoltage(12000);
		flywheelBot.moveVoltage(12000);
		pros::delay(20);

  }
}
