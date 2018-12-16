#include "main.h"
// #include "autism.h"

void blueFront();
void redFront();
void blueBack();
void redBack();

void autonomous() {
  switch (lcdCounter) {
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
  }
}

void blueFront () {
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
void redFront() {}
void blueBack() {
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	chassis.setMaxVelocity(150);
	pros::Task flywheelTaskHandle(flywheelTask);
	FW.target = 3000;
	flywheelToggle = 2;

	pros::delay(3000);

	chassis.moveDistance(36_in);//Blue back support with T
	pros::delay(500);
	chassis.moveDistance(-41_in);

	gyroPID(930);
	chassis.moveDistance(37_in);

	pros::delay(50);

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
	gyroPID(-120);
	chassis.moveDistance(17_in);

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
void redBack() {}
