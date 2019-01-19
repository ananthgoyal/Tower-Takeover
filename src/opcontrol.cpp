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
int indexerToggle = 0;
int flywheelToggle = 0;

okapi::Motor flywheelTop(2, true, okapi::AbstractMotor::gearset::green);
okapi::Motor flywheelBot(3, false, okapi::AbstractMotor::gearset::green);
okapi::ADIEncoder encoder('C', 'D', true);
okapi::Motor indexer(9, true, okapi::AbstractMotor::gearset::red);
okapi::Motor flipper(5, true, okapi::AbstractMotor::gearset::red);
okapi::ADIGyro gyro('B', 1);
okapi::Controller controller;
auto chassis = okapi::ChassisControllerFactory::create({1, 11}, {-10, -20}, okapi::AbstractMotor::gearset::green, {4.125_in, 10_in});

void flywheelTask(void *param);
void gyroPID(int rotation);
void flywheelTask2(void *param);
int lcdCounter = 0;

void opcontrol()
{
	pros::Task flywheelTaskHandle(flywheelTask);
	pros::Task flywheelTask2Handle(flywheelTask2);

	int flywheelToggle = 0;
	while (true)
	{

		chassis.arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
		indexer.moveVelocity(100 * controller.getDigital(ControllerDigital::L1) - 100 * controller.getDigital(ControllerDigital::L2));
		flipper.moveVelocity(100 * controller.getDigital(ControllerDigital::down) - 100 * controller.getDigital(ControllerDigital::up));

		pros::delay(20);
	}
}

void flywheelTask(void *)
{
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
		FW.speed = FW.kP * FW.error + FW.kD * FW.derivative + FW.kI * FW.integral;

		if (controller.getDigital(ControllerDigital::R2))
		{
			FW.speed = -2;
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
				FW.target = 3000;
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
	//gyro.reset();
	GY.integral = 0;
	bool val = false;
	int timer = 0;
	while (timer < 50) //timer < 1000
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
		GY.speed = (GY.kP * GY.error + GY.kD * GY.derivative + GY.kI * GY.integral) * 2.0 / 127;

		chassis.tank(GY.speed, -1 * GY.speed);

		val = GY.derivative == 0 && abs(GY.error) < 30;
		//if (val)
		//{
		timer++;
		//}

		pros::delay(20);
	}
	chassis.tank(0, 0);
}

/*______________________________________________________________________________________________________________________________________________________________________________*/

void blueFront();
void redFront();
void blueBack();
void redBack();
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
		blueBack();
		break;
	case 4:
		redBack();
		break;
	case 5:
		progSkills();
		break;
	}
}

void blueFront()
{
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	chassis.setMaxVelocity(160);
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;

	//get ball
	chassis.moveDistance(36_in);
	pros::delay(500);
	chassis.moveDistance(-41_in);

	//turn to face flags
	gyroPID(910);
	chassis.setMaxVelocity(100);
	pros::delay(100);

	//shoot first ball
	indexer.moveVelocity(100);
	pros::delay(350);
	indexer.moveVelocity(0);

	//move second ball up
	pros::delay(100);
	indexer.moveVelocity(100);
	pros::delay(200);
	indexer.moveVelocity(0);

	//get in place to shoot second ball
	chassis.moveDistance(29_in);
	gyroPID(910);

	//shoot second ball
	indexer.moveVelocity(100);
	pros::delay(700);
	//indexer.moveVelocity(0);

	//stop flywheel
	FW.target = 0;
	flywheelToggle = 0;

	//hit bottom flag
	gyroPID(1060);
	chassis.setMaxVelocity(130);
	chassis.moveDistance(26_in);
	gyroPID(960);

	chassis.setMaxVelocity(100);
	chassis.moveDistance(-14_in);
	gyroPID(1560);
	flipper.moveVelocity(-100);
	pros::delay(400);
	flipper.moveVelocity(0);

	chassis.moveDistance(-10_in);

	flipper.moveVelocity(100);
	pros::delay(500);
	flipper.moveVelocity(0);
}
void redFront()
{
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	chassis.setMaxVelocity(160);
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;

	//get ball
	chassis.moveDistance(36_in);
	pros::delay(500);
	chassis.moveDistance(-41_in);

	//turn to face flags
	gyroPID(-910);
	chassis.setMaxVelocity(100);
	pros::delay(100);

	//get in place to shoot first ball
	chassis.moveDistance(-10_in);
	pros::delay(350);
	gyroPID(-970);

	//shoot first ball
	indexer.moveVelocity(100);
	pros::delay(350);
	indexer.moveVelocity(0);

	//move second ball up
	pros::delay(100);
	indexer.moveVelocity(100);
	pros::delay(200);
	indexer.moveVelocity(0);

	//get in place to shoot second ball
	chassis.moveDistance(29_in);
	gyroPID(-910);

	//shoot second ball
	indexer.moveVelocity(100);
	pros::delay(700);

	//stop flywheel
	FW.target = 0;
	flywheelToggle = 0;

	//hit bottom flag
	gyroPID(-1100);
	chassis.setMaxVelocity(130);
	chassis.moveDistance(30_in);
	gyroPID(-960);

	chassis.setMaxVelocity(100);
	chassis.moveDistance(-4_in);
	gyroPID(-1560);
	flipper.moveVelocity(-100);
	pros::delay(400);
	flipper.moveVelocity(0);

	chassis.moveDistance(-10_in);

	flipper.moveVelocity(100);
	pros::delay(500);
	flipper.moveVelocity(0);
}
void blueBack()
{
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	chassis.setMaxVelocity(150);
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;

	//get ball
	chassis.moveDistance(36_in);
	pros::delay(300);
	chassis.moveDistance(-41_in);

	//rotate to face flags
	gyroPID(930);
	chassis.moveDistance(37_in);
	pros::delay(350);

	//shoot first ball
	gyroPID(950);
	indexer.moveVelocity(100);
	pros::delay(350);
	indexer.moveVelocity(0);

	//move second ball up
	pros::delay(100);
	indexer.moveVelocity(100);
	pros::delay(200);
	indexer.moveVelocity(0);

	//move in position for shooting second ball
	chassis.moveDistance(40_in);
	gyroPID(930);

	//shoot second ball
	indexer.moveVelocity(100);
	pros::delay(800);

	//stop flywheel
	FW.target = 0;
	flywheelToggle = 0;

	//bottom flag
	gyroPID(1080);
	chassis.setMaxVelocity(130);
	chassis.moveDistance(30_in);
}
void redBack()
{
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	chassis.setMaxVelocity(150);
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;

	//get ball
	chassis.moveDistance(36_in);
	pros::delay(300);
	chassis.moveDistance(-41_in);

	//rotate to face flags
	gyroPID(-960);
	chassis.moveDistance(37_in);
	pros::delay(350);

	//shoot first ball
	gyroPID(-960);
	indexer.moveVelocity(100);
	pros::delay(350);
	indexer.moveVelocity(0);

	//move second ball up
	pros::delay(100);
	indexer.moveVelocity(100);
	pros::delay(200);
	indexer.moveVelocity(0);

	//move in position for shooting second ball
	chassis.moveDistance(40_in);
	gyroPID(-930);

	//shoot second ball
	indexer.moveVelocity(100);
	pros::delay(800);

	//stop flywheel
	FW.target = 0;
	flywheelToggle = 0;

	//bottom flag
	gyroPID(-1080);
	chassis.setMaxVelocity(130);
	chassis.moveDistance(30_in);
}
void progSkills()
{
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	chassis.setMaxVelocity(150);
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;

	//get ball
	pros::delay(200);
	chassis.moveDistance(40_in);
	pros::delay(200);

	//flip back cap
	gyroPID(-960);
	chassis.setMaxVelocity(80);
	flipper.moveVelocity(100);
	pros::delay(600);
	flipper.moveVelocity(0);
	chassis.moveDistance(-7_in);
	gyroPID(-960);

	//move back from cap
	flipper.moveVelocity(-100);
	pros::delay(300);
	flipper.moveVelocity(0);
	chassis.moveDistance(7_in);

	//move to back red tile
	gyroPID(-70);
	flipper.moveVelocity(-100);
	pros::delay(600);
	flipper.moveVelocity(0);
	chassis.setMaxVelocity(100);
	chassis.moveDistance(-40_in);
	gyroPID(-1000);

	//get in place to shoot first ball
	chassis.moveDistance(30_in);
	pros::delay(200);
	gyroPID(-1010);

	//shoot first ball
	indexer.moveVelocity(100);
	pros::delay(350);
	indexer.moveVelocity(0);

	//move second ball up
	pros::delay(100);
	indexer.moveVelocity(100);
	pros::delay(200);
	indexer.moveVelocity(0);

	//get in place to shoot second ball
	chassis.moveDistance(39_in);
	pros::delay(200);
	gyroPID(-1010);

	//shoot second ball
	indexer.moveVelocity(100);
	pros::delay(700);
	indexer.moveVelocity(0);

	//hit low flag
	chassis.setMaxVelocity(150);
	gyroPID(-1170);
	chassis.moveDistance(28_in);
	pros::delay(200);

	//move back to front red tile
	chassis.moveDistance(-44_in);
	pros::delay(200);
	gyroPID(-70);

	//wall allign
	chassis.setMaxVelocity(70);
	chassis.moveDistance(-8_in);
	chassis.setMaxVelocity(150);

	//get ball
	pros::delay(200);
	chassis.moveDistance(47_in); //43
	pros::delay(200);

	//middle column (low + middle) flag
	gyroPID(-950);
	pros::delay(200);
	chassis.setMaxVelocity(80);
	chassis.moveDistance(16_in);
	pros::delay(200);
	indexer.moveVelocity(100);
	pros::delay(750);
	indexer.moveVelocity(0);
	gyroPID(-1100);	//-1100
	chassis.setMaxVelocity(150);
	chassis.moveDistance(32_in);
	pros::delay(200);

	//flip front cap
	chassis.moveDistance(-22_in);
	FW.target = 0;
	flywheelToggle = 0;
	flipper.moveVelocity(100);
	pros::delay(600);
	flipper.moveVelocity(0);
	gyroPID(-200);
	chassis.setMaxVelocity(80);
	chassis.moveDistance(-3_in);
	flipper.moveVelocity(-80);	//-100
	pros::delay(1000);
	flipper.moveVelocity(0);

	//move between red tiles
	gyroPID(1250);
	chassis.setMaxVelocity(130);	//150
	chassis.moveDistance(67_in);
	pros::delay(200);
	gyroPID(-100);
	chassis.moveDistance(82_in);
}

/*______________________________________________________________________________________________________________________________________________________________________________*/

bool selected = false;

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
		if (lcdCounter > 5)
		{
			lcdCounter = 5;
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
		return "Blue Back";
	case 4:
		return "Red Back";
	case 5:
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
	gyro.reset();
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