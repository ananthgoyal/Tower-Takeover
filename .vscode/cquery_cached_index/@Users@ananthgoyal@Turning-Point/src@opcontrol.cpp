#include "main.h"
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
struct PID DL;
struct PID DR;
int indexerToggle = 0;
int flywheelToggle = 0;

okapi::Motor flywheelTop(2, true, okapi::AbstractMotor::gearset::green);
okapi::Motor flywheelBot(3, false, okapi::AbstractMotor::gearset::green);
okapi::ADIEncoder encoder('C', 'D', true);
okapi::Motor indexer(9, true, okapi::AbstractMotor::gearset::red);
okapi::Motor flipper(5, true, okapi::AbstractMotor::gearset::green);
okapi::ADIGyro gyro1('A', 1);
okapi::ADIGyro gyro2('B', 1);
okapi::Controller controller;
auto chassis = okapi::ChassisControllerFactory::create({1, 12}, {-10, -19}, okapi::AbstractMotor::gearset::green, {4.125, 10});

void flywheelTask(void *param);
void gyroPID(int rotation);
void movePID(int distanceL, int distanceR, int ms);
void flywheelTask2(void *param);
int lcdCounter = 0;

void opcontrol()
{
	flywheelToggle = 0;
	FW.target = 0;
	while (true)
	{
		//std::cout << gyro1.getRemapped(359, -359) << " " << gyro2.getRemapped(359, -359) << std::endl;
		chassis.arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
		indexer.moveVelocity(200 * controller.getDigital(ControllerDigital::L1) - 200 * controller.getDigital(ControllerDigital::L2));
		flipper.moveVelocity(200 * controller.getDigital(ControllerDigital::up) - 200 * controller.getDigital(ControllerDigital::down));

		pros::delay(20);
	}
}

void flywheelTask(void *)
{
	while (true)
	{
		//std::cout << "POS: " << gyro1.getRemapped(359, -359) << std::endl;
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
		FW.speed = FW.kP * FW.error + FW.kD * FW.derivative + FW.kI * FW.integral;

		if (controller.getDigital(ControllerDigital::R2))
		{
			FW.speed = -0.75;
		}
		else if (flywheelToggle == 0)
		{
			FW.speed = 0;
		}
		else if (FW.speed < 0.5)
		{
			FW.speed = 0.5;
		}

		flywheelTop.controllerSet(FW.speed);
		flywheelBot.controllerSet(FW.speed);

		pros::delay(20);
	}
}
void flywheelTask2(void *)
{
	while (true)
	{
		if (controller.getDigital(ControllerDigital::R1))
		{
			pros::delay(20);
			flywheelToggle++;
			if (flywheelToggle > 2)
			{
				flywheelToggle = 0;
			}

			switch (flywheelToggle)
			{
			case 0:
				FW.target = 0;
				break;
			case 1:
				FW.target = 2000;
				break;
			case 2:
				FW.target = 2500;
				break;
			}

			while (controller.getDigital(ControllerDigital::R1))
			{
				pros::delay(20);
			}
			pros::delay(50);
		}
	}
}
void gyroPID(int rotation)
{
	GY.target = rotation;
	gyro2.reset();
	GY.integral = 0;
	bool val = false;
	int timer = 0;
	while (timer < 50)
	{
		GY.kP = 0.1;
		GY.kD = 0.05;
		GY.kI = 0;
		GY.sensor = gyro2.get();
		//std::cout << "POS: " << GY.sensor << std::endl;
		GY.error = GY.target - GY.sensor;
		GY.derivative = GY.error - GY.previous_error;
		GY.integral += GY.error;
		GY.previous_error = GY.error;
		GY.speed = (GY.kP * GY.error + GY.kD * GY.derivative + GY.kI * GY.integral) * 2.0 / 127;

		chassis.tank(GY.speed, -1 * GY.speed);

		timer++;
		pros::delay(20);
	}
	chassis.tank(0, 0);
}
void movePID(int distanceL, int distanceR, int ms) {
	int targetL = distanceL * 360 / (2 * 3.1415 * (4.125 / 2));
	int targetR = distanceR * 360 / (2 * 3.1415 * (4.125 / 2));
	auto drivePIDL = okapi::IterativeControllerFactory::posPID(0.00275, 0, 0.0015);
	auto drivePIDR = okapi::IterativeControllerFactory::posPID(0.00275, 0, 0.0015);
	
	chassis.resetSensors();

	int timer = 0;
	double errorL;
	double errorR;
	double powerL;
	double powerR;
	double multiplier = 1;
	while(timer < ms){

		
		if(targetL < 0 && targetR < 0 && timer < 400) multiplier = 0.5;
		else {
			if(multiplier < 1){
				multiplier += 0.1;
			}
		}

		errorL = targetL - chassis.getSensorVals()[0];
		errorR = targetR - chassis.getSensorVals()[1];
		powerL = drivePIDL.step(errorL);
		powerR = drivePIDR.step(errorR);
		
		if(powerL > 1) powerL = 1;
		if(powerR > 1) powerL = 1;
		if(powerL < -1) powerL = -1;
		if(powerR < -1) powerR = -1;

		powerL *= multiplier;
		powerR *= multiplier;

		chassis.tank(-powerL, -powerR);	//second is powerR
		//std::cout << powerL << " " << powerR << std::endl;

		pros::delay(20);

		timer += 20;
	}
}

/*______________________________________________________________________________________________________________________________________________________________________________*/

void blueFront();
void redFront();
void blueBackTrap();
void redBackTrap();
void blueVasanth();
void redMeghaj();
void blueBackPark();
void redBackPark();
void progSkills();

void autonomous()
{
	switch (lcdCounter)
	{
	case 1:
		blueFront();
		break;
	case 2:
		redFront();
		break;
	case 3:
		blueBackTrap();
		break;
	case 4:
		redBackTrap();
		break;
	case 5:
		blueVasanth();
		break;
	case 6:
		redMeghaj();
		break;
	case 7:
		blueBackPark();
		break;
	case 8:
		redBackPark();
		break;
	case 9:
		progSkills();
		break;
	}
}


void blueFront()
{
}
void redFront()
{
}
void blueBackTrap()
{
}
void redBackTrap()
{
}
void blueVasanth()
{	
}
void redMeghaj()
{
}
void blueBackPark() 
{
}
void redBackPark() 
{
}
void progSkills()
{
}


/*______________________________________________________________________________________________________________________________________________________________________________*/

bool selected = true;	//TODO: false

void left_button()
{
	if (!selected)
	{
		lcdCounter--;
		if (lcdCounter < 0)
		{
			lcdCounter = 0;
		}
	}
}
void center_button()
{
	if (!selected)
	{
		selected = true;
	}
}
void right_button()
{
	if (!selected)
	{
		lcdCounter++;
		if (lcdCounter > 9)
		{
			lcdCounter = 9;
		}
	}
}
std::string convert(int arg)
{
	switch (arg)
	{
	case 1:
		return "Blue Front";
	case 2:
		return "Red Front";
	case 3:
		return "Blue Back Trap";
	case 4:
		return "Red Back Trap";
	case 5:
		return "Vasanth Blue";
	case 6:
		return "Meghaj Red";
	case 7:
		return "Blue Back Park";
	case 8:
		return "Red Back Park";
	case 9:
		return "Prog Skills";
	default:
		return "No Auton";
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::register_btn0_cb(left_button);
	pros::lcd::register_btn1_cb(center_button);
	pros::lcd::register_btn2_cb(right_button);

	
	while (!selected)
	{
		pros::lcd::set_text(0, convert(lcdCounter));
		pros::delay(20);
	}
	

	pros::lcd::set_text(0, convert(lcdCounter) + " (SELECTED)");

	pros::Task flywheelTaskHandle(flywheelTask);
	pros::Task flywheelTask2Handle(flywheelTask2);
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
void competitionitialize() {}