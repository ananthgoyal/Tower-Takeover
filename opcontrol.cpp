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
	auto chassis = ChassisControllerFactory::create({Motor(11), Motor(1)}, {Motor(-20), Motor(-10)});
	Controller controller;
	Motor indexer(9);
	Motor flywheelTop(2);
	Motor flywheelBot(-3);
	Motor flipper(5);
	while (true) {
		chassis.arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
		if (controller.getDigital(ControllerDigital::L1))
		{
			indexer.moveVelocity(-100);
		}
		else if (controller.getDigital(ControllerDigital::L2))
		{
			indexer.moveVelocity(100);
		}
		else
		{
			indexer.moveVelocity(0);
		}
		if (controller.getDigital(ControllerDigital::R2))
		{
			flywheelTop.moveVelocity(200);
			flywheelBot.moveVelocity(200);
		}
		else if (controller.getDigital(ControllerDigital::R1))
		{
			flywheelTop.moveVelocity(-200);
			flywheelBot.moveVelocity(-200);
		}
		else
		{
			flywheelTop.moveVelocity(0);
			flywheelBot.moveVelocity(0);
		}
		if (controller.getDigital(ControllerDigital::up))
		{
			flipper.moveVelocity(-100);
		}
		else if (controller.getDigital(ControllerDigital::down))
		{
			flipper.moveVelocity(100);
		}
		else
		{
			flipper.moveVelocity(0);
		}
		pros::delay(20);
	}
}
