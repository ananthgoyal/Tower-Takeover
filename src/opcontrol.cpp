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
typedef struct PID pid;
pid LT;
pid TL;
//others
 
//Initializing Sensors and Motors
pros::ADIPotentiometer trayPot('H');
pros::ADIPotentiometer rollerLiftPot('F');
pros::ADIEncoder trackingWheel('B', 'C');
okapi::Controller controller;
 
okapi::Motor trayLift(-16);
okapi::Motor rollerLift(9);
 
okapi::MotorGroup rollers({-5, 8});
okapi::Motor rollerOne(-5);
okapi::Motor rollerTwo(8);
/*
case 0:
        break;
    case 1:
        red();
        break;
    case 2:
        push();
        break;
    case 3:
        blue();
        break;
    case 4:
        redBig();
        break;
*/
//Initializing global variables
int lcdCounter = 4;
int buttonCount = 0;
bool isPressed = false;
 
double slowTraySpeed = 27.5;
double fastTraySpeed = 200;
 
bool holdTray = false;
int trayPosition = 400;
bool holdRollerLift = false;
int rollerLiftToggle = 0;
int rollerLiftPosition;
 
double slowMoveKP = 0.0005; //0.001
double fastMoveKP = 0.002;
int holdToggle = 0;
 
//Initializing chassis
auto chassis = okapi::ChassisControllerFactory::create({20, 19}, {-11, -3}, okapi::AbstractMotor::gearset::green, {4.125, 10});
 
//Initializing tasks
void rollerLiftToggleTask(void *param);
void trayPIDTask(void *param);
void rollerLiftPIDTask(void *param);
void trayToggleTask(void *param);
 
void opcontrol()
{
    //Starting Tasks
    pros::Task rollerLiftToggleTaskHandle(rollerLiftToggleTask);
    pros::Task trayPIDTaskHandle(trayPIDTask);
    pros::Task rollerLiftPIDTaskHandle(rollerLiftPIDTask);
    pros::Task trayToggleTaskHandle(trayToggleTask);
    rollerLiftPosition = 400;
 
    while (true)
    {
        //Drive Mode:Arcade 
        chassis.arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
        //Controls roller intake and its direction
        rollers.moveVelocity(200 * controller.getDigital(ControllerDigital::L1) - 
            200 * controller.getDigital(ControllerDigital::Y) - 50 * controller.getDigital(ControllerDigital::L2));
        
        pros::delay(20);
    }
}
 
void trayLiftPID(double value)
{
    TL.target = value;
    TL.integral = 0;
    TL.sensor = trayLift.get_position();
    TL.error = TL.target - TL.sensor;
    int timer = 0;
 
    while ((abs(TL.error) >= 30))                       
    {                           
        TL.kP = 0.4;                     
        TL.kD = 0;                  
        TL.kI = 0;                       
        TL.sensor = trayLift.get_position(); 
        TL.error = TL.target - TL.sensor;
        TL.derivative = TL.error - TL.previous_error;
        TL.integral += TL.error;
        TL.previous_error = TL.error;
        TL.speed = (TL.kP * TL.error + TL.kD * TL.derivative + TL.kI * TL.integral);
        trayLift.moveVelocity(TL.speed);
        //std::cout << "\nPot Value:" << trayLift.get_position();
        timer++;
        pros::delay(20);
    }
}
 
void rollerLiftPID(double degrees)
{
    LT.target = degrees;
    LT.integral = 0;
    LT.sensor = rollerLift.getPosition();
    LT.error = LT.target - LT.sensor;
    int timer = 0;
 
    while ((abs(LT.error) >= 40))//
    {                                         //or while(timer < 50){
        LT.kP = 0.15;                         //need tuning
        LT.kD = 0.1;                          //need tuning
        LT.kI = 0;                            //need tuning
        LT.sensor = rollerLift.getPosition(); // = sensor.getValue || post setup
        LT.error = LT.target - LT.sensor;
        LT.derivative = LT.error - LT.previous_error;
        LT.integral += LT.error;
        LT.speed = (LT.kP * LT.error + LT.kD * LT.derivative + LT.kI * LT.integral);
        rollerLift.moveVelocity(LT.speed);
        //fill
        timer += 20;
        std::cout << "\n"
                  << rollerLift.getPosition();
        pros::delay(20);
    }
}
 
void movePID(double distanceL, double distanceR, double speedkP, int ms)
{
    double targetL = distanceL * 360 / (2 * 3.1415 * (4.125 / 2));
    double targetR = distanceR * 360 / (2 * 3.1415 * (4.125 / 2));
    auto drivePIDL = okapi::IterativeControllerFactory::posPID(speedkP, 0.001, 0.0015); //= data
    auto drivePIDR = okapi::IterativeControllerFactory::posPID(speedkP, 0.001, 0.0015);
    chassis.resetSensors();
 
    int timer = 0;
    double errorL;
    double errorR;
    double powerL;
    double powerR;
 
    while (timer < ms)
    {
        errorL = targetL - chassis.getSensorVals()[0];
        errorR = targetR - chassis.getSensorVals()[1];
        powerL = drivePIDL.step(errorL);
        powerR = drivePIDR.step(errorR);
        chassis.tank(-powerL, -powerR);
 
        pros::delay(10);
        timer += 10;
    }
 
    chassis.tank(0, 0);
}
 
//Toggle for the tray
void trayToggleTask(void *)
{
    while (true)
    {
        //Buttton Pressed --> hold lifts or unhold lifts
        if (controller[ControllerDigital::A].changedToPressed())
        {
            holdTray = !holdTray;
            pros::delay(100);
 
        }
 
        if (holdTray)
        {
            //lifts tray enough to allow for lift to move
            trayPosition = 400;
        }
        else
        {
            //driver control tray movement
            //mutliple speeds for precise stacking
            trayLift.moveVelocity(slowTraySpeed * controller.getDigital(ControllerDigital::R1) +
                                  fastTraySpeed * controller.getDigital(ControllerDigital::left) - fastTraySpeed * controller.getDigital(ControllerDigital::R2));
        }
        pros::delay(25);
    }
}
 
void trayPIDTask(void *)
{
    while (true)
    {
        //std::cout << "\nPot Value:" << holdTray;
        TL.target = trayPosition;
        TL.integral = 0;
        TL.sensor = trayLift.get_position();
        TL.error = TL.target - TL.sensor;
        int timer = 0;
 
        while (holdTray)                         //(abs(TL.error) >= 40)
        {                                        //or while(timer < 50){
            TL.kP = 0.4;                         //need tuning
            TL.kD = 0;                           //need tuning
            TL.kI = 0;                           //need tuning
            TL.sensor = trayLift.get_position(); // = sensor.getValue || post setup
            TL.error = TL.target - TL.sensor;
            TL.derivative = TL.error - TL.previous_error;
            TL.integral += TL.error;
            TL.previous_error = TL.error;
            TL.speed = (TL.kP * TL.error + TL.kD * TL.derivative + TL.kI * TL.integral);
            trayLift.moveVelocity(TL.speed);
            //fill
            timer += 20;
            //std::cout << "\nPot Value:" << trayLift.get_position();
            pros::delay(20);
        }
    }
}
 
void rollerLiftToggleTask(void *)
{
    while (true)
    {
        //Buttton Pressed --> hold lifts or unhold lifts
        if (controller[ControllerDigital::up].changedToPressed())
        {
            rollerLiftToggle = 2;
            pros::delay(200);
        }
        if (controller[ControllerDigital::right].changedToPressed())
        {
            rollerLiftToggle = 1;
            pros::delay(200);
        }
        if (controller[ControllerDigital::down].changedToPressed())
        {
            rollerLiftToggle = 3;
            pros::delay(200);
        }
        if (controller[ControllerDigital::B].changedToPressed())
        {
            rollerLiftToggle = 4;
            pros::delay(200);
        }
        if (controller[ControllerDigital::X].changedToPressed())
        {
            rollerLiftToggle = 4;
            pros::delay(200);
        }
 
        //whenever lift is held, ensures tray is held too
        if (rollerLiftToggle == 2 || rollerLiftToggle == 1){
            holdTray = true;
        }
 
        //small tower
        if (rollerLiftToggle == 1)
        {
            rollerLiftPosition = 400;
            holdRollerLift = true;
        }
        //medium tower
        else if (rollerLiftToggle == 2)
        {
            rollerLiftPosition = 560;
            holdRollerLift = true;
        }
        else if (rollerLiftToggle == 3)
        {
            holdRollerLift = false;
            rollerLift.move_velocity(-5);
        }
        else
        {
            holdRollerLift = false;
            rollerLift.controllerSet(controller.getDigital(ControllerDigital::X) - controller.getDigital(ControllerDigital::B));
        }
 
        pros::delay(25);
    }
}
 
void rollerLiftPIDTask(void *)
{
    while (true)
    {
        //LT.target = rollerLiftPosition;
        LT.integral = 0;
        LT.sensor = rollerLift.getPosition();
        LT.error = LT.target - LT.sensor;
        int timer = 0;
 
        while (holdRollerLift)
        {                                         //or while(timer < 50){
            LT.kP = 0.4;                          //need tuning
            LT.kD = 0.3;                          //need tuning
            LT.kI = 0;                            //need tuning
            LT.sensor = rollerLift.getPosition(); // = sensor.getValue || post setup
            LT.error = rollerLiftPosition - LT.sensor;
            LT.derivative = LT.error - LT.previous_error;
            LT.integral += LT.error;
            LT.speed = (LT.kP * LT.error + LT.kD * LT.derivative + LT.kI * LT.integral);
            rollerLift.moveVelocity(LT.speed);
            //fill
            timer += 20;
            
            pros::delay(20);
        }
    }
}
 
//Autonomous
 
void red()
{
    
    //movePID(10, 10, slowMoveKP, 1500);
    rollers.moveVelocity(-200);
    //rollerLift.moveVelocity(200);
    pros::delay(800);
    rollers.moveVelocity(0);
    //rollerLift.moveVelocity(-200);
    rollerLiftPID(0);
    pros::delay(750);
    trayLift.moveVelocity(-200);
    rollerLift.moveVelocity(-20);
 
    //Flip out
    rollers.moveVelocity(200);
    //Pick up the cubes
 
    pros::delay(500);
    trayLift.moveVelocity(0);
    movePID(45, 45, slowMoveKP*0.2, 1500);
    
    //Move back to wall align
    //movePID(-43, -43, slowMoveKP, 2000);
    pros::delay(900);
    movePID(31, 31, slowMoveKP*0.1, 1500);
    pros::delay(1000);
    
    movePID(-49, -49, slowMoveKP, 1000);
    rollers.moveVelocity(0);
    //pros::delay(2000);
    pros::delay(200); 
    movePID(26.5 , -26.5, slowMoveKP, 800);
    pros::delay(500);
    movePID(27, 27, slowMoveKP, 1000);
    pros::delay(500);
    //movePID(25, 25, slowMoveKP, 1000);
 
    //Get bottom cube in position to stack
    rollerLift.moveVelocity(0);
 
    rollers.moveVelocity(-100);
    pros::delay(400);
    rollers.moveVelocity(0);
 
    trayLiftPID(1000);
    rollers.moveVelocity(-100);
    pros::delay(500);
 
    //Straighten up the tray and align bottom
    //std::cout << "\nPot Value:" << trayPot.get_value();
    //trayLift.moveVelocity(200);
    //pros::delay(2000);
    // holdTray=true;
    // trayPosition = 950;
    // pros::delay(4000);
    // holdTray=false;
    // //pros::delay(100000);
    //backLiftPID(900);
    //holdTray2=false;
    movePID(-20,-20, slowMoveKP, 1000);
}
 
void progskills()
{
    rollerLiftPID(400);
    movePID(10, 10, slowMoveKP, 1500);
    rollerLiftPID(0);
    pros::delay(500);
    movePID(-10, -10, slowMoveKP, 1500);
    rollers.moveVelocity(200);
    movePID(15, 15, slowMoveKP*0.2, 1500);
    movePID(30, 30, slowMoveKP*0.2, 1500);
    movePID(40, 40, slowMoveKP*0.2, 1500);
    movePID(8, -8, slowMoveKP*0.2, 1500);
    movePID(30, 30, slowMoveKP*0.2, 1500);
    movePID(15, 15, slowMoveKP*0.2, 1500);
    movePID(-8, 8, slowMoveKP*0.2, 1500);
    movePID(30, 30, slowMoveKP*0.2, 1500);
    movePID(30, 30, slowMoveKP*0.2, 1500);
    movePID(30, 30, slowMoveKP*0.2, 1500);
    movePID(25, 25, slowMoveKP*0.2, 1500);
    pros::delay(500);
    rollers.moveVelocity(20); 
    movePID(-30, -30, slowMoveKP*0.2, 1500);
    pros::delay(300);
    movePID(10, -10, slowMoveKP, 1500);
    pros::delay(300);
    movePID(31, 31, slowMoveKP*0.2, 1500);
    pros::delay(2000);
    rollers.moveVelocity(10);
    trayLift.move_velocity(200);
    pros::delay(700);
    rollers.moveVelocity(0);
    trayLift.move_velocity(50);
    pros::delay(3200);
    movePID(10,10,slowMoveKP, 1500);
    trayLift.move_velocity(-50);
    pros::delay(200);
    rollers.moveVelocity(-50);
    movePID(-20, -20, slowMoveKP*0.2, 1500);
    movePID(-30,30, slowMoveKP*0.2, 1500);
    movePID(-35, -35, slowMoveKP*0.2, 1500); 
    pros::delay(1000);
    rollers.moveVelocity(100); 
    movePID(55, 55, slowMoveKP*0.2, 1500); 
    movePID(30, 30, slowMoveKP*0.2, 1500);
    //rollers.moveVelocity(-100); 
    pros::delay(3000); 
    rollers.moveVelocity(-50);
    pros::delay(1500);
    rollers.moveVelocity(0); 
    movePID(-15, -15, slowMoveKP*0.2, 1500);
    
    //pros::Task rollerLiftToggleTaskHandle(rollerLiftToggleTask);
    pros::Task trayPIDTaskHandle(trayPIDTask);
    pros::Task rollerLiftPIDTaskHandle(rollerLiftPIDTask);
    //pros::Task trayToggleTaskHandle(trayToggleTask);
    rollerLiftPosition = 560;
    holdRollerLift = true;
    holdTray = true;
    trayPosition = 400;
    movePID(15, 15, slowMoveKP*0.2, 1500);
    rollers.moveVelocity(-75);
    pros::delay(1000);
    rollers.moveVelocity(0);
    movePID(-10, -10, slowMoveKP, 1500);
    holdRollerLift = false;
    rollerLift.moveVelocity(-50);
    pros::delay(2000); 
    holdTray = false;
    trayLift.moveVelocity(0);
    rollerLift.moveVelocity(0);
    movePID(-20, -20, slowMoveKP*0.2, 1500);
    movePID(35, -35, slowMoveKP*0.2, 1500);
    pros::delay(1000);
    movePID(-30, -30, slowMoveKP*0.2, 1500);
    rollers.moveVelocity(100);
    movePID(40, 40, slowMoveKP*0.2, 1500);
    movePID(20, 20, slowMoveKP*0.2, 1500);
    pros::delay(3000); 
    rollers.moveVelocity(-50);
    pros::delay(1500);
    rollers.moveVelocity(0); 
    movePID(-10, -10, slowMoveKP*0.2, 1500);
    rollerLiftPosition = 400;
    holdRollerLift = true;
    holdTray = true;
    trayPosition = 400;
    movePID(15, 15, slowMoveKP*0.2, 1500);
    rollers.moveVelocity(-75);
    pros::delay(1000);
    rollers.moveVelocity(0);
    movePID(-10, -10, slowMoveKP, 1500);
    holdRollerLift = false;
    rollerLift.moveVelocity(-50);
    pros::delay(2000); 
    holdTray = false;
    trayLift.moveVelocity(0);
    rollerLift.moveVelocity(0);
    movePID(-20, -20, slowMoveKP*0.2, 1500);

}
 
void push()
{
    movePID(30, 30, slowMoveKP, 1500);
    movePID(-30, -30, slowMoveKP, 1500);
}
 
void blue()
{
    //pros::delay(1000);
    rollerLiftPID(400);
    //movePID(10, 10, slowMoveKP, 1500);
    rollers.moveVelocity(-200);
    //rollerLift.moveVelocity(200);
    pros::delay(800);
    rollers.moveVelocity(0);
    //rollerLift.moveVelocity(-200);
    rollerLiftPID(0);
    pros::delay(500);
    trayLift.moveVelocity(-200);
    rollerLift.moveVelocity(-10);
 
    //Flip out
    rollers.moveVelocity(200);
    //Pick up the cubes
 
    pros::delay(500);
    trayLift.moveVelocity(0);
    movePID(45, 45, slowMoveKP*0.2, 1500);
    
    //Move back to wall align
    //movePID(-43, -43, slowMoveKP, 2000);
    pros::delay(900);
    movePID(31, 31, slowMoveKP*0.1, 1500);
    pros::delay(1000);
    
    movePID(-49, -49, slowMoveKP, 1000);
    rollers.moveVelocity(0);
    //pros::delay(2000);
    pros::delay(200); 
    movePID(-27 , 27, slowMoveKP, 800);
    pros::delay(500);
    movePID(27, 27, slowMoveKP, 1000);
    pros::delay(500);
    //movePID(25, 25, slowMoveKP, 1000);
 
    //Get bottom cube in position to stack
    rollerLift.moveVelocity(0);
 
    rollers.moveVelocity(-100);
    pros::delay(400);
    rollers.moveVelocity(0);
 
    trayLiftPID(1000);
    rollers.moveVelocity(-100);
    pros::delay(500);
 
    //Straighten up the tray and align bottom
    //std::cout << "\nPot Value:" << trayPot.get_value();
    //trayLift.moveVelocity(200);
    //pros::delay(2000);
    // holdTray=true;
    // trayPosition = 950;
    // pros::delay(4000);
    // holdTray=false;
    // //pros::delay(100000);
    //backLiftPID(900);
    //holdTray2=false;
    movePID(-20,-20, slowMoveKP, 1000);
}
 
void autonomous()
{
    switch (lcdCounter)
    {
    case 0:
        break;
    case 1:
        red();
        break;
    case 2:
        push();
        break;
    case 3:
        blue();
        break;
    case 4:
        progskills();
        break;
    }
}
 
bool selected = true; //TODO: false
 
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
        if (lcdCounter > 4)
        {
            lcdCounter = 4;
        }
    }
}
std::string convert(int arg)
{
    switch (arg)
    {
    case 0:
        return "No Auton";
    case 1:
        return "Red";
    case 2:
        return "Push";
    case 3:
        return "Blue";
    case 4:
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
    //pros::lcd::print(0, "Test Temperature", pros::trayLift::get_temperature());
 
    //intakeLS.calibrate();
    //rollers.calibrate();
    //indexerLS.calibrate();
    //hoodLS.calibrate();
 
    while (!selected)
    {
        pros::lcd::set_text(0, convert(lcdCounter));
        pros::delay(20);
    }
 
    pros::lcd::set_text(0, convert(lcdCounter) + " (SELECTED)");
 
    //pros::Task trayTaskHandle(trayTask);
    //pros::Task armTaskHandle(armTask);
    //pros::Task trayLiftTaskHandle(trayLiftTask);
}
 
//void disabled() {}
 
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
 
//void competitionitialize() {}

