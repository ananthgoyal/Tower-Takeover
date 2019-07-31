#include "main.h"
using namespace okapi;
struct PID
{
	float kP;
	float kI;
	float kD;
	float integral;
	float derivative;
	float error;
	float previous_error;
	float speed;
	float target;
	float sensor;
};
//hello world
typedef struct PID pid;
//pid LT;
//others

//pros::ADIPotentiometer potCollector('E'); 
okapi::Controller controller;
okapi::Motor backLift(-18);
okapi::Motor armLift(19);
//okapi::Motor leftRoller(3, false, okapi::AbstractMotor::gearset::green);
//okapi::Motor rightRoller(10, true, okapi::AbstractMotor::gearset::green);
okapi::MotorGroup rollers({3,-10});

auto chassis = okapi::ChassisControllerFactory::create({2, 13}, {-9, -20}, okapi::AbstractMotor::gearset::green, {4.125, 10});
//auto motorGroup = okapi::ChassisControllerFactory::create({3,-10}, okapi::AbstractMotor::gearset::green,{4.125,10});
//NEED PORT auto lift = okapi::ChassisControllerFactory::create()
//Position Tracking start
//float dTheta; //= (renc - lenc)/chaswidth -> radians
//float sector; //sector = (renc+lence)/2
//float radius; //radius = sector/dTheta
//End

void opcontrol() {
	while (true)
	{
		//std::cout << intakeLS.get_value() << " " << indexerLS.get_value() << " " << hoodLS.get_value() << " " << intakeBall << " " << indexerBall << " " << hoodBall << std::endl;
		chassis.arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
		rollers.moveVelocity(200 * controller.getDigital(ControllerDigital::L1) - 200 * controller.getDigital(ControllerDigital::L2));
		backLift.moveVelocity(200 * controller.getDigital(ControllerDigital::R1) - 200 * controller.getDigital(ControllerDigital::R2));
		armLift.moveVelocity(200 * controller.getDigital(ControllerDigital::up) -  200 * controller.getDigital(ControllerDigital::down));
		//motorGroup.arcade(200 * controller.getDigital(ControllerDigital::L1));
		//leftRoller.moveVelocity(200 * controller.getDigital(ControllerDigital::L1));
		//rightRoller.moveVelocity(200 * controller.getDigital(ControllerDigital::R1));
		//indexer.moveVelocity(200 * controller.getDigital(ControllerDigital::L1) - 200 * controller.getDigital(ControllerDigital::L2));
		//flipper.moveVelocity(200 * controller.getDigital(ControllerDigital::up) - 200 * controller.getDigital(ControllerDigital::down));
		
		pros::delay(20);
	}
	/*pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(2);
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;
		pros::delay(20);
	}*/
}

/*void liftPID(double height){
	LT.target = height; 
	LT.integral = 0;  
	int timer = 0; 

	while(timer < 50){ //or while(LT.sensor - error <= 10)
		LT.kP = 0;//tuned
		LT.kD = 0; //tuned
		LT.kI = 0; //tuned
		LT.sensor = potCollector.get_value(); // = sensor.getValue || post setup
		LT.error = LT.target - LT.sensor;
		LT.derivative = LT.error - LT.previous_error; 
		LT.integral += LT.error; 
		LT.speed = (LT.kP * LT.error + LT.kD * LT.derivative + LT.kI * LT.integral)
		//lift.moveVelocity(LT.speed);
		//fill
		timer++; 
		pros::delay(20); 
	}
	//action PID || return func
}*/

/*void movePID(double distanceL, double distanceR, int ms){
	double targetL = distanceL * 360 /(2 * 3.1415  * (4.125 / 2));
	double targetR = distanceR * 360 /(2 * 3.1415  * (4.125 / 2));
	auto drivePIDL = okapi::IterativeControllerFactory::posPID(0.00275, 0.001, 0.0015); //= data
	auto drivePIDR = okapi::IterativeControllerFactory::posPID(0.00257, 0.001, 0.0015);
	chassis.resetSensors(); 

	int timer = 0; 
	double errorL;
	double errorR;
	double powerL; 
	double powerR;

	while(timer < ms){
		errorL = targetL - chassis.getSensorVals()[0]; 
		errorR = targetR - chassis.getSensorVals()[1];
		powerL = drivePIDL.step(errorL);
		powerR = drivePIDR.step(errorR);
		chassis.tank(-powerL, -powerR);

		pros::delay(10);
		timer+=10;
	}

	chassis.tank(0,0); 

}*/

/*void positionTracking(double x, double y){
	int rect = 0; 
	int length = 0;
	int chaswidth = 0; 
}*/

//Autonomous

/*void autonmous(){
	switch(lcdCounter){
		//fill
	}
}*/
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

