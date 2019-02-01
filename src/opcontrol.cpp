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
okapi::Motor flipper(5, true, okapi::AbstractMotor::gearset::red);
okapi::ADIGyro gyro('B', 1);
okapi::Controller controller;
auto chassis = okapi::ChassisControllerFactory::create({1, 11}, {-10, -20}, okapi::AbstractMotor::gearset::green, {4.125, 10});

void flywheelTask(void *param);
void gyroPID(int rotation);
void movePID(int distance, int ms);
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
			FW.speed = -.75;	//-2
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
void movePID(int distance, int ms) {
	int target = distance * 360 / (2 * 3.1415 * (4.125 / 2));
	auto drivePIDL = okapi::IterativeControllerFactory::posPID(0.00295, 0, 0.0015);	//0.00275
	auto drivePIDR = okapi::IterativeControllerFactory::posPID(0.00265, 0, 0.0015);	//0.00275
	chassis.resetSensors();

	int timer = 0;
	double errorL;
	double errorR;
	double powerL;
	double powerR;
	double multiplier = 1;
	while(timer < ms){
		if(target < 0 && timer < 400) multiplier = 0.3;	//0.3
		else {
			if(multiplier < 1){
				multiplier += 0.1;
			}
		}

		errorL = target - chassis.getSensorVals()[0];
		errorR = target - chassis.getSensorVals()[1];
		powerL = drivePIDL.step(errorL);
		powerR = drivePIDR.step(errorR);
		
		if(powerL > 1) powerL = 1;
		if(powerR > 1) powerL = 1;
		if(powerL < -1) powerL = -1;
		if(powerR < -1) powerR = -1;

		powerL *= multiplier;
		powerR *= multiplier;

		chassis.tank(-powerL, -powerR);
		std::cout << powerL << " " << powerR << std::endl;

		pros::delay(20);

		timer += 20;
	}
}

/*______________________________________________________________________________________________________________________________________________________________________________*/

void blueFront();
void redFront();
void blueBack();
void redBack();
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
		blueBack();
		break;
	case 4:
		redBack();
		break;
	case 5:
		blueBackPark();
		break;
	case 6:
		redBackPark();
		break;
	case 7:
		progSkills();
		break;
	}
}

void blueFront()
{
	gyro.reset();
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;
	pros::delay(100);

	//get ball
	movePID(34, 2000);
	movePID(-10, 2000);

	//get front cap
	gyroPID(-960);
	
	flipper.moveVelocity(100);
	pros::delay(500);
	flipper.moveVelocity(0);
	movePID(-7, 2000);
	flipper.moveVelocity(-80);
	pros::delay(300);
	flipper.moveVelocity(0);

	//move in place to shoot first ball
	
	gyroPID(-1300);
	flipper.moveVelocity(-100);
	movePID(36, 2000);
	flipper.moveVelocity(0);
	gyroPID(900);

	//shoot first ball
	indexer.moveVelocity(100);
	pros::delay(250);
	indexer.moveVelocity(0);

	//move second ball up
	indexer.moveVelocity(100);
	pros::delay(250);
	indexer.moveVelocity(0);

	//get in place to shoot second ball
	movePID(53, 2000);
	gyroPID(900);

	//shoot second ball
	indexer.moveVelocity(100);
	pros::delay(500);
	FW.target = 0;
	flywheelToggle = 0;

	//hit low flag
	
	gyroPID(1070);
	movePID(32, 2000);
}
void redFront()
{
	gyro.reset();
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;
	pros::delay(100);

	//get ball
	movePID(34, 1750);
	movePID(-14, 1500);

	//get front cap
	gyroPID(860);
	
	flipper.moveVelocity(100);
	pros::delay(500);
	flipper.moveVelocity(0);
	movePID(-7, 1000);
	flipper.moveVelocity(-80);
	pros::delay(300);
	flipper.moveVelocity(0);

	//move in place to shoot first ball
	gyroPID(1400);
	flipper.moveVelocity(-100);
	movePID(32, 2000);
	flipper.moveVelocity(0);
	gyroPID(-950);

	//shoot first ball
	indexer.moveVelocity(100);
	pros::delay(250);
	indexer.moveVelocity(0);

	//move second ball up
	indexer.moveVelocity(100);
	pros::delay(250);
	indexer.moveVelocity(0);

	//get in place to shoot second ball
	movePID(40, 1000);
	gyroPID(-950);

	//shoot second ball
	indexer.moveVelocity(100);
	pros::delay(500);
	FW.target = 0;
	flywheelToggle = 0;

	//hit low flag
	
	gyroPID(-1150);
	movePID(32, 2000);
}
void blueBack()
{
	gyro.reset();
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;
	pros::delay(100);

	//get ball
	movePID(34, 2000);
	movePID(-5, 2000);

	//get back cap
	gyroPID(1250);
	
	flipper.moveVelocity(100);
	pros::delay(500);
	flipper.moveVelocity(0);
	movePID(-11, 2000);
	flipper.moveVelocity(-100);
	pros::delay(300);
	flipper.moveVelocity(0);

	//move to between spawns
	
	gyroPID(1550);
	movePID(47, 2000);
	gyroPID(900);

	//shoot first ball
	indexer.moveVelocity(100);
	pros::delay(250);
	indexer.moveVelocity(0);

	//move second ball up
	indexer.moveVelocity(100);
	pros::delay(250);
	indexer.moveVelocity(0);

	//get in place to shoot second ball
	movePID(53, 2000);
	gyroPID(900);

	//shoot second ball
	indexer.moveVelocity(100);
	pros::delay(450);
	FW.target = 0;
	flywheelToggle = 0;

	//hit low flag
	
	gyroPID(1050);
	movePID(32, 2000);

}
void redBack()
{
	gyro.reset();
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;
	pros::delay(100);

	//get ball
	movePID(34, 1000);
	movePID(-5, 500);

	//get back cap
	gyroPID(-1340);
	
	flipper.moveVelocity(100);
	pros::delay(400);
	flipper.moveVelocity(0);
	movePID(-13, 2000);
	flipper.moveVelocity(-100);
	pros::delay(300);
	flipper.moveVelocity(0);

	//move to between spawns
	
	gyroPID(-1550);
	movePID(47, 2000);

	//rotate to face flags
	gyroPID(-995);

	//shoot first ball
	indexer.moveVelocity(100);
	pros::delay(250);
	indexer.moveVelocity(0);

	//move second ball up
	indexer.moveVelocity(100);
	pros::delay(250);
	indexer.moveVelocity(0);

	//get in place to shoot second ball
	movePID(51, 2000);	//47
	gyroPID(-970);

	//shoot second ball
	indexer.moveVelocity(100);
	pros::delay(450);
	FW.target = 0;
	flywheelToggle = 0;

	//hit low flag
	
	gyroPID(-1100);
	movePID(32, 2000);
	
}
void blueBackPark() 
{
	gyro.reset();
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;
	pros::delay(100);

	//get ball
	movePID(34, 2000);
	movePID(-5, 2000);

	//get back cap
	gyroPID(1250);
	
	flipper.moveVelocity(100);
	pros::delay(500);
	flipper.moveVelocity(0);
	movePID(-11, 2000);
	flipper.moveVelocity(-100);
	pros::delay(300);
	flipper.moveVelocity(0);

	//park
	gyroPID(960);
	
	movePID(47, 2000);
}
void redBackPark() 
{
	gyro.reset();
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;
	pros::delay(100);

	//get ball
	movePID(37, 1700);
	movePID(-9, 800);
	FW.target = 0;
	flywheelToggle = 0;

	//get back cap
	gyroPID(-1340);
	
	flipper.moveVelocity(100);
	pros::delay(400);
	flipper.moveVelocity(0);
	movePID(-15, 1300);
	flipper.moveVelocity(-100);
	pros::delay(300);
	flipper.moveVelocity(0);

	//park
	movePID(10,1100);
	gyroPID(-960);
	
	movePID(37, 2000);
}
void progSkills()
{
	gyro.reset();
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;
	pros::delay(100);

	//get ball
	pros::delay(200);
	movePID(40, 2000);
	pros::delay(200);

	//flip back cap
	gyroPID(-960);
	
	flipper.moveVelocity(100);
	pros::delay(600);
	flipper.moveVelocity(0);
	movePID(-7, 2000);
	gyroPID(-960);

	//move back from cap
	flipper.moveVelocity(-100);
	pros::delay(300);
	flipper.moveVelocity(0);
	movePID(7, 2000);

	//move to back red tile
	gyroPID(-70);
	flipper.moveVelocity(-100);
	pros::delay(600);
	flipper.moveVelocity(0);
	
	movePID(-40, 2000);
	gyroPID(-1030);	//-1000

	//get in place to shoot first ball
	movePID(30, 2000);
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
	movePID(39, 2000);
	pros::delay(200);
	gyroPID(-1050);

	//shoot second ball
	indexer.moveVelocity(100);
	pros::delay(700);
	indexer.moveVelocity(0);

	//hit low flag
	
	gyroPID(-1170);
	movePID(28, 2000);
	pros::delay(200);

	//move back to front red tile
	movePID(-39, 2000);
	pros::delay(200);
	gyroPID(-70);

	//"wall allign"
	
	movePID(-8, 2000);
	

	//get ball
	pros::delay(200);
	movePID(47, 2000);
	pros::delay(200);

	//middle column (low + middle) flag
	movePID(-4, 2000);
	gyroPID(-920);
	pros::delay(200);
	
	movePID(16, 2000);
	pros::delay(200);
	indexer.moveVelocity(100);
	pros::delay(750);
	indexer.moveVelocity(0);
	gyroPID(-1100);	//-1100
	
	movePID(32, 2000);
	pros::delay(200);

	//flip front cap
	movePID(-22, 2000);
	FW.target = 0;
	flywheelToggle = 0;
	flipper.moveVelocity(100);
	pros::delay(600);
	flipper.moveVelocity(0);
	gyroPID(-200);
	
	movePID(-6, 2000);
	pros::delay(300);			
	flipper.moveVelocity(-80);
	pros::delay(1000);
	flipper.moveVelocity(0);
	movePID(3, 2000);		

	//move between red tiles
	gyroPID(1250);
	
	movePID(67, 2000);
	pros::delay(200);

	//park
	gyroPID(-100);
	FW.target = 2000;
	flywheelToggle = 1;
	movePID(82, 2000);
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
		if (lcdCounter > 7)
		{
			lcdCounter = 7;
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
		return "Blue Back Park";
	case 6:
		return "Red Back Park";
	case 7:
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