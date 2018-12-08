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

	int targetRPM = 3000;
	int curRPM;
	int velocities[] = {0, 0 ,0, 0, 0, 0, 0, 0, 0, 0};
	encoder.reset();
  while (true) {
		for (int i = 0; i < 9; i++) {
			velocities[i] = velocities[i + 1];
		}
		velocities[9] = encoder.get();

		curRPM = (velocities[9] - velocities[0]) * 25 / 9;
		std::cout << "RPM: " << curRPM << std::endl;

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
