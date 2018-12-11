#include "main.h"

//TODO: run prosv5 mut in terminal

okapi::Controller controller;
auto myChassis = okapi::ChassisControllerFactory::create({1, 11}, {-10, -20});
okapi::Motor flywheelTop(2, true, okapi::AbstractMotor::gearset::green);
okapi::Motor flywheelBot(3, false, okapi::AbstractMotor::gearset::green);
okapi::ADIEncoder encoder('C', 'D', true);
okapi::Motor indexer (9, true, okapi::AbstractMotor::gearset::red);
okapi::Motor flipper(5, true, okapi::AbstractMotor::gearset::red);

void gyroPID(int rotation);
void flywheelTask(void* param);

void opcontrol() {
	int flywheelToggle = 0;
	while (true) {
		//std::cout << "POS: " << flywheelToggle << std::endl;

		myChassis.arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
		if (controller.getDigital(ControllerDigital::L1))
		{
			indexer.moveVelocity(100);
		}
		else if (controller.getDigital(ControllerDigital::L2))
		{
			indexer.moveVelocity(-100);
		}
		else
		{
			indexer.moveVelocity(0);
		}

		if (controller.getDigital(ControllerDigital::R2))
		{
			flywheelTop.moveVelocity(-200);
			flywheelBot.moveVelocity(-200);
			flywheelToggle = 0;
		}
		else if (controller.getDigital(ControllerDigital::R1))
		{
			flywheelToggle += 1;
			if(flywheelToggle != 1){
				flywheelTop.moveVelocity(200);
				flywheelBot.moveVelocity(200);
			}
		}
		else
		{
			if(flywheelToggle == 0){
				flywheelTop.moveVelocity(0);
				flywheelBot.moveVelocity(0);
			}
		}

		if (controller.getDigital(ControllerDigital::up))
		{
			flipper.moveVelocity(100);
		}
		else if (controller.getDigital(ControllerDigital::down))
		{
			flipper.moveVelocity(-100);
		}
		else
		{
			flipper.moveVelocity(0);
		}
		pros::delay(20);
	}
}

void flywheelTask2(void* param) {
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
			flywheelTop.controllerSet(0.6);
			flywheelBot.controllerSet(0.6);
		}
		else {
			flywheelTop.controllerSet(2);
			flywheelBot.controllerSet(2);
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
void flywheelTask(void*)
{
	okapi::Motor flywheelTop(2, true, okapi::AbstractMotor::gearset::green);
	okapi::Motor flywheelBot(3, false, okapi::AbstractMotor::gearset::green);
	okapi::ADIEncoder encoder('C', 'D', true);

	FW.target = 3300;
	FW.integral = 0;
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

		if (FW.speed < 0.7) {
			FW.speed = 0.7;
		}

		//flywheelTop.controllerSet(10);
		//flywheelBot.controllerSet(10);
		flywheelTop.moveVelocity(200);
		flywheelBot.moveVelocity(200);

		pros::delay(20);
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
	while (true)	//timer<1000
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
/*void drivePID(int distance){
	okapi::IntegratedEncoder encoderFR (okapi::Motor(1));
	okapi::IntegratedEncoder encoderBR (okapi::Motor(1));
	okapi::IntegratedEncoder encoderFL (okapi::Motor(1));
	okapi::IntegratedEncoder encoderBL (okapi::Motor(1));

	float[] encoder = myChassis.getSensorVals();
	std::cout << "Values " << encoder << std::endl;


	encoderBL.reset();
	encoderBR.reset();
	encoderFL.reset();
	encoderFR.reset();

	DR.target = targetR;
	DL.target = targetL;
	bool val = false;
	bool driveStop = false;
	int timer = 0;
	while (timer < 1000)
	{
		DR.kP = 0.42;
		DL.kP = 0.42;
		DR.kD = 0.35;
		DL.kD = 0.35;
		DR.sensor = SensorValue[DriveEncoderR];
		AbstractMotor::brakeMode::
		DL.sensor = SensorValue[DriveEncoderL];
		DL.error = DL.target - DL.sensor;
		DR.error = DR.target - DR.sensor;
		DL.derivative = DL.error - DL.previous_error;
		DR.derivative = DR.error - DR.previous_error;
		DL.previous_error = DL.error;
		DR.previous_error = DR.error;
		DL.speed = DL.kP*DL.error + DL.kD*DL.derivative;
		DR.speed = DR.kP*DR.error + DR.kD*DR.derivative;
		motor[DriveFL] = DR.speed;
		motor[DriveBL] = DR.speed;
		motor[DriveFR] = DR.speed;
		motor[DriveBR] = DR.speed;

		val = DR.derivative == 0 && abs(DR.error) < 30 ;
		if (val)
		{
			driveStop = true;
			timer++;
		}
		else if (!val)
		{
			driveStop = false;
		}

	}
	motor[DriveFL] = 0;
	motor[DriveBL] = 0;
	motor[DriveFR] = 0;
	motor[DriveBR] = 0;
}
*/