#include "main.h"
//for std::abs
#include<cstdlib>
#include"sensors.hpp"
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
pros::vision_signature_s_t B_FLAG = pros::Vision::signature_from_utility(2, -3469, -2519, -2994, 11367, 14763, 13064, 5.9, 0);
pros::vision_signature_s_t R_FLAG = pros::Vision::signature_from_utility(1, 7505, 11963, 9734, -541, 1, -270, 3.5, 0);

using namespace okapi;

//load forever until stopped with stopLoader() or manually.
 static void loadf()
 {
   LoadServ.move(127);
   LoadServ2.move(127);
 }

 //load forever reversed, same rules as loadf()
 static void loadfr()
 {
   LoadServ.move(-127);
   LoadServ2.move(-127);
 }
 //load reversed for a set ammount of miliseconds, then stop the loader.
 static void loadR(std::uint32_t mili)
 {
   LoadServ.move(-127);
   LoadServ2.move(-127);
   pros::Task::delay(mili);
   LoadServ.move(0);
   LoadServ2.move(0);
 }
 //stop the loading system.
 static void stopLoader()
 {
   LoadServ.move_velocity(0);
   LoadServ2.move_velocity(0);
 }
 //load normally for a set ammount of miliseconds, then stop the loader.
 static void load(std::uint32_t mili)
 {
	 LoadServ.move(127);
	 LoadServ2.move(127);
	 pros::Task::delay(mili);
	 stopLoader();
 }
 //stop the fly wheel
  static void stopShooter()
  {
    fly.move(0);
    fly2.move(0);
  }
  //turn on the shooter for a set amount of seconds.
 static void shoot(std::uint32_t mili)
 {
   fly.move_velocity(600);
   fly2.move_velocity(600);
   pros::Task::delay(mili);
 }

 static void shootS(std::uint32_t mili, int speed)
 {
   fly.move_velocity(speed);
   fly2.move_velocity(speed);
   pros::Task::delay(mili);
 }

//the robot chassis, okapi lib uses this data for the Chassis functions like moveDistance
 static auto Chassis = ChassisControllerFactory::create
 (
   //left drive train
   {1, 2},
   //right drive train, reversed hence the -
   {-11, -12},
   //the motors are using the green gearset.
   AbstractMotor::gearset::green,
   //the wheel diameter is 4 in, the chassis from middle of wheel to middle of wheel horizontally is 14.5 in
   {4_in, 14.5_in}
 );


static void FAR_RED()
{
  shootS(2500, 520);
  load(2000);
}


static void ALT_BLUE_C()
{
  vision_READ(R_FLAG, -15, 5);
  shoot(1700);
  pros::Task::delay(500);
	//shoot top top left flag
  load(200);
	//realign after shooting from an angle
	Chassis.turnAngle(8_deg);
	//speed up the chassis again, we only have a minute after all.
	Chassis.setMaxVelocity(150);
  Chassis.turnAngle(-100_deg);
	//toggle the bottom left flag after shooting.

  loadf();
	Chassis.moveDistance(38_in);
  stopLoader();
  Chassis.moveDistance(-39_in);

  Chassis.turnAngle(101_deg);

  vision_READ(R_FLAG, -20, 5);

  Chassis.setMaxVelocity(120);

  Chassis.moveDistance(28_in);
  load(1000);

  Chassis.turnAngle(17_deg);
  Chassis.moveDistance(20_in);
}

//start at red and move to gray, move forward to back of next tick
static void ALT_RED_C()
{
	vision_READ(B_FLAG, -5, 25);
  shoot(1700);
  pros::Task::delay(500);
	//shoot top top left flag
  load(200);
	//realign after shooting from an angle
	Chassis.turnAngle(-8_deg);
	//speed up the chassis again, we only have a minute after all.
	Chassis.setMaxVelocity(150);
  Chassis.turnAngle(102_deg);
	//toggle the bottom left flag after shooting.

  loadf();
	Chassis.moveDistance(39_in);
  stopLoader();
  Chassis.moveDistance(-39_in);

  Chassis.turnAngle(-101_deg);

  vision_READ(B_FLAG, -5, 25);

  Chassis.setMaxVelocity(120);

  Chassis.moveDistance(28_in);
  load(1000);

  Chassis.turnAngle(-15_deg);
  Chassis.moveDistance(20_in);
}

//start front of first tick, farthest from flag

//back of second tick
static void skillz()
{
  //shoot top top left flag
  shoot(2000);
  loadf();
  pros::Task::delay(1250);
  stopShooter();
  stopLoader();

  //slow down the chassis to get more accurate turns.
  Chassis.setMaxVelocity(50);
  //realign after shooting from an angle
  Chassis.turnAngle(-10);
  //speed up the chassis again, we only have a minute after all.
  Chassis.setMaxVelocity(125);
  //toggle the bottom left flag after shooting.
  Chassis.moveDistance(48_in);

  //get out of flag and turn towards cap(first ball)
  Chassis.moveDistance(-44_in);
  Chassis.turnAngle(102_deg);
  //turn on the loader to get the ball and drive for it.
  loadf();
  Chassis.moveDistance(38_in);
  //give time to load
  pros::Task::delay(100);
  stopLoader();

  //turn towards flag and move back a bit to get base allignment.
  Chassis.turnAngle(-79_deg);
  Chassis.moveDistance(-2_in);
  //allign with the biggest green object on flag on the x coord.
//  readV();

  //reverse to make sure we didn't load to much and the ball will get caught in the fly wheel
  loadR(100);
  //shoot and stop the loader and shooter
  shoot(2000);
  load(1400);
  stopLoader();
  stopShooter();

  //hit the bottom middle flag
  Chassis.moveDistance(28_in);
  Chassis.turnAngle(-17_deg);
  Chassis.setMaxVelocity(140);
  Chassis.moveDistance(24_in);
  Chassis.setMaxVelocity(125);

  //get out of flags
  Chassis.moveDistance(-24_in);

  //turn towards cap to flip and turn on the loader reversed so the robot doesn't eat the cap, then go for it.
  Chassis.turnAngle(-97_deg);
  loadfr();
  Chassis.moveDistance(21_in);

  Chassis.turnAngle(195_deg);
  Chassis.moveDistance(62_in);
  Chassis.moveDistance(-3_in);

  Chassis.turnAngle(97_deg);
  Chassis.moveDistance(20_in);
  Chassis.turnAngle(95_deg);
  loadf();
  Chassis.moveDistance(12_in);
  pros::Task::delay(100);
  Chassis.turnAngle(148_deg);
  loadR(100);
//  readV();
  shoot(2000);
  load(1400);

  stopShooter();
  Chassis.moveDistance(24_in);
  Chassis.turnAngle(-12);
  Chassis.moveDistance(24_in);
  Chassis.turnAngle(-15_deg);
  Chassis.moveDistance(8_in);

  Chassis.moveDistance(-24_in);
  Chassis.turnAngle(-97_deg);
  Chassis.moveDistance(78_in);
  Chassis.turnAngle(-97_deg);
  Chassis.moveDistance(44_in);
  Chassis.turnAngle(100_deg);
  Chassis.setMaxVelocity(200);
  Chassis.moveDistance(-6_ft);
}
void autonomous()
{
	FAR_RED();
}
