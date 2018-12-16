#include "main.h"

okapi::Controller controller;
auto myChassis = okapi::ChassisControllerFactory::create({1, 11}, {-10, -20}, okapi::AbstractMotor::gearset::green, {4.125_in, 10_in});

okapi::Motor flywheelTop(2, true, okapi::AbstractMotor::gearset::green);
okapi::Motor flywheelBot(3, false, okapi::AbstractMotor::gearset::green);
okapi::ADIEncoder encoder('C', 'D', true);
okapi::Motor indexer (9, true, okapi::AbstractMotor::gearset::red);
okapi::Motor flipper(5, true, okapi::AbstractMotor::gearset::red);
okapi::ADIButton indexButton('E');

void gyroPID(int rotation);
void flywheelTask(void* param);
void flywheelTask2(void* param);
void indexerTask(void* param);

struct PID
{
	float kP;
	float kI;
	float kD;
	float error;
	float integral;
	float derivative;
	float previous_error;
	float speed;
	float target;
	float sensor;
};
struct PID FW;
struct PID GY;

int indexerToggle = 0;
int flywheelToggle = 0;

void opcontrol() {
	auto chassis = okapi::ChassisControllerFactory::create({1, 11}, {-10, -20}, okapi::AbstractMotor::gearset::green, {4.125_in, 10_in});

	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	chassis.setMaxVelocity(160);
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;

	pros::delay(3000);

	chassis.moveDistance(36_in);
	pros::delay(500);
	chassis.moveDistance(-41_in);

	gyroPID(910);
	chassis.setMaxVelocity(100);
	chassis.moveDistance(-6_in);

	indexer.moveVelocity(100);
	pros::delay(350);
	indexer.moveVelocity(0);

	pros::delay(100);
	indexer.moveVelocity(100);
	pros::delay(200);
	indexer.moveVelocity(0);

	chassis.moveDistance(33_in);

	indexer.moveVelocity(100);
	pros::delay(700);
	indexer.moveVelocity(0);

	FW.target = 0;
	flywheelToggle = 0;

	gyroPID(150);
	chassis.setMaxVelocity(130);
	chassis.moveDistance(13_in);
	gyroPID(-100);
	chassis.moveDistance(15_in);

	chassis.moveDistance(-14_in);
	gyroPID(600);
	flipper.moveVelocity(-100);
	pros::delay(500);
	flipper.moveVelocity(0);

	chassis.moveDistance(-8_in);
	
	flipper.moveVelocity(100);
	pros::delay(500);
	flipper.moveVelocity(0);



}

void indexerTask (void * params) {
	while (true)
	{
		if (controller.getDigital(ControllerDigital::X) && !(indexButton.isPressed()))
		{
			if(indexerToggle == 100){
				indexerToggle = 0;
			} else {
				indexerToggle = 100;
			}

			while(controller.getDigital(ControllerDigital::X)){
				pros::delay(20);
			}
		}
		if(indexButton.isPressed()){
			indexerToggle = 0;
		}
		pros::delay(20);
	}
}

void flywheelTask(void*) {
	while (true)
	{
		FW.kP = 0.1;
		FW.kD = 0.05;
		FW.kI = 0;
		FW.sensor = encoder.get() * 25;
		std::cout << "RPM: " << FW.sensor << std::endl;
		encoder.reset();
		FW.error = FW.target - FW.sensor;
		FW.derivative = FW.error - FW.previous_error;
		FW.integral += FW.error;
		FW.previous_error = FW.error;
		FW.speed = FW.kP*FW.error + FW.kD*FW.derivative + FW.kI*FW.integral;

		if (controller.getDigital(ControllerDigital::R2)) {
			FW.speed = -2;
		}
		else if (flywheelToggle == 0) {
			FW.speed = 0;
		}
		else if (FW.speed < 0.5) {
			FW.speed = 0.5;
		}


		flywheelTop.controllerSet(FW.speed);
		flywheelBot.controllerSet(FW.speed);

		pros::delay(20);
	}
}

void flywheelTask2(void* params) {
	while (true) {
		if (controller.getDigital(ControllerDigital::R1)) {
			pros::delay(20);
			flywheelToggle++;
			if (flywheelToggle > 3) {
				flywheelToggle = 0;
			}


			switch (flywheelToggle) {
				case 0: FW.target = 0;
						break;
				case 1: FW.target = 2000;
						break;
				case 2: FW.target = 3000;
						break;
			}

			while(controller.getDigital(ControllerDigital::R1)){
				pros::delay(20);
			}
			pros::delay(50);
		}
	}
}

void gyroPID(int rotation) {
	okapi::ADIGyro gyro('B', 1);

	GY.target = rotation;
	gyro.reset();
	GY.integral = 0;
	bool val = false;
	int timer = 0;
	while (timer < 50)	//timer < 1000
	{
		GY.kP = 0.1;
		GY.kD = 0.05;
		GY.kI = 0;
		GY.sensor = gyro.get();
		std::cout << "POS: " << GY.sensor << std::endl;
		GY.error = GY.target - GY.sensor;
		GY.derivative = GY.error - GY.previous_error;
		GY.integral += GY.error;
		GY.previous_error = GY.error;
		GY.speed = (GY.kP*GY.error + GY.kD*GY.derivative + GY.kI*GY.integral) * 2.0/127;

		myChassis.tank(GY.speed, -1 * GY.speed);

		val = GY.derivative == 0 && abs(GY.error) < 30;
		//if (val)
		//{
			timer++;
		//}

		pros::delay(20);
	}
	myChassis.tank(0, 0);
}