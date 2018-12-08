#include "main.h"

//TODO: run prosv5 mut in terminal

okapi::Controller controller;

void flywheelTask(void* param);

void opcontrol() {

	std::string text("PROS");
	pros::Task my_task(flywheelTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "flywheelTask");

	okapi::Motor indexer (9, true, okapi::AbstractMotor::gearset::red);
	okapi::Motor capFlipper(5, true, okapi::AbstractMotor::gearset::red);

	auto myChassis = okapi::ChassisControllerFactory::create({1, 11}, {10, 20});

	while (true) {
		myChassis.arcade(controller.getAnalog(okapi::ControllerAnalog::rightX), controller.getAnalog(okapi::ControllerAnalog::leftY));
		indexer.controllerSet(controller.getDigital(okapi::ControllerDigital::R2) - controller.getDigital(okapi::ControllerDigital::R1));
		capFlipper.controllerSet(controller.getDigital(okapi::ControllerDigital::up) - controller.getDigital(okapi::ControllerDigital::down));

		pros::delay(20);
	}
}

void flywheelTask(void* param) {
	okapi::Motor flywheelTop(2, true, okapi::AbstractMotor::gearset::green);
	okapi::Motor flywheelBot(3, false, okapi::AbstractMotor::gearset::green);
	okapi::ADIEncoder encoder('C', 'D', true);

	int targetRPM = 2500;
	int curRPM;
	encoder.reset();
  while (true) {
		curRPM = encoder.get() * 25; // * 100/3
		std::cout << "RPM: " << curRPM << std::endl;
		encoder.reset();
		if (targetRPM < curRPM) {
			flywheelTop.controllerSet(0.5);
			flywheelBot.controllerSet(0.5);
		}
		else {
			flywheelTop.controllerSet(1);
			flywheelBot.controllerSet(1);
		}

		pros::delay(20);

  }
}
