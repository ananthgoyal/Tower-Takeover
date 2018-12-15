#include "main.h"

//TODO: run prosv5 mut in terminal

okapi::Controller controller;
auto myChassis = okapi::ChassisControllerFactory::create({1, 11}, {-10, -20}, okapi::AbstractMotor::gearset::green, {4.125_in, 10_in});
okapi::Motor flywheelTop(2, true, okapi::AbstractMotor::gearset::green);
okapi::Motor flywheelBot(3, false, okapi::AbstractMotor::gearset::green);
okapi::ADIEncoder encoder('C', 'D', true);
okapi::Motor indexer (9, true, okapi::AbstractMotor::gearset::red);
okapi::Motor flipper(5, true, okapi::AbstractMotor::gearset::red);
okapi::ADIButton indexButton('E');
void gyroPID(int rotation);
void drivePID(int distance);
void drive(QLength distance);
void flywheelTask(void* param);
void flywheelTask2(void* param);
void indexerTask(void* param);

int indexerToggle = 0;
int flywheelToggle = 0;

void opcontrol() {

	drive(12_in);

}
void indexerTask (void * params) 
{
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
void flywheelTask(void*) {
	while (true)
	{
		FW.kP = 0.1;
		FW.kD = 0.05;
		FW.kI = 0;
		FW.sensor = encoder.get() * 25;
		//std::cout << "RPM: " << FW.sensor << std::endl;
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

struct PID GY;
void gyroPID(int rotation)
{
	okapi::ADIGyro gyro('B', 1);

	GY.target = rotation;
	gyro.reset();
	GY.integral = 0;
	bool val = false;
	int timer = 0;
	while (timer < 1000)
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

		myChassis.tank(-1 * GY.speed, GY.speed);

		val = GY.derivative == 0 && abs(GY.error) < 30;
		if (val)
		{
			timer++;
		}

		//pros::delay(20);
	}
	myChassis.tank(0, 0);
}


struct PID DR;
struct PID DL;
void drivePID(int distance){
	okapi::IntegratedEncoder encoderFR (okapi::Motor(1));
	//okapi::IntegratedEncoder encoderBR (okapi::Motor(11));
	okapi::IntegratedEncoder encoderFL (okapi::Motor(10));
	//okapi::IntegratedEncoder encoderBL (okapi::Motor(20));

	encoderFR.reset();
	//encoderBR.reset();
	encoderFL.reset();
	//encoderBR.reset();

	DR.target = distance;
	DL.target = distance;
	bool val = false;
	bool driveStop = false;
	int timer = 0;
	while (timer < 1000)
	{
		DR.kP = 0.42;
		DL.kP = 0.42;
		DR.kD = 0.35;
		DL.kD = 0.35;
		DR.sensor = encoderFR.get();
		DL.sensor = encoderFL.get();
		DL.error = DL.target - DL.sensor;
		DR.error = DR.target - DR.sensor;
		DL.derivative = DL.error - DL.previous_error;
		DR.derivative = DR.error - DR.previous_error;
		DL.previous_error = DL.error;
		DR.previous_error = DR.error;
		DL.speed = DL.kP*DL.error + DL.kD*DL.derivative * 2.0/127;
		DR.speed = DR.kP*DR.error + DR.kD*DR.derivative * 2.0/127;

		//myChassis.tank(DL.speed, -1 * DR.speed);
		myChassis.tank(100, 100);

		val = DR.derivative == 0 && abs(DR.error) < 30;
		if (val)
		{
			timer++;
		}

	}
	myChassis.tank(0, 0);
}

void drive(QLength distance) {

	myChassis.moveDistance(distance);
}
