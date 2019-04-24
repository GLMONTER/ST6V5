#include "main.h"
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
  //could optimize
   LoadServ.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
   LoadServ2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
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

 static void shoot(std::uint32_t mili, int vol)
 {
   fly.move(127);
   fly2.move(127);
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
   {11, 12},
   //right drive train, reversed hence the -
   {-20, -19},
   //the motors are using the green gearset.
   AbstractMotor::gearset::green,
   //the wheel diameter is 4.125 in, the chassis from middle of wheel to middle of wheel horizontally is 14.5 in
   {4.125_in, 14_in}
 );

 static void FAR_BLUE()
 {
   Chassis.setMaxVelocity(150);

   //allignBackH(270, 20);
   loadf();
   Chassis.moveDistance(37.5_in);
   pros::Task::delay(50);
   stopLoader();
   Chassis.moveDistance(6_in);

   Chassis.turnAngle(-77_deg);

   LoadServ2.move(-127);
   Chassis.setMaxVelocity(150);
   Chassis.moveDistance(17_in);
   LoadServ2.move(0);

   Chassis.setMaxVelocity(150);
   Chassis.moveDistance(-52_in);

 }


static void FAR_RED()
{
  Chassis.setMaxVelocity(150);

  //allignBackH(270, 20);
  loadf();
  Chassis.moveDistance(37.5_in);
  pros::Task::delay(50);
  stopLoader();
  Chassis.moveDistance(6_in);

  Chassis.turnAngle(92_deg);

  LoadServ2.move(-127);
  Chassis.setMaxVelocity(150);
  Chassis.moveDistance(17_in);
  LoadServ2.move(0);

  Chassis.setMaxVelocity(150);
  Chassis.moveDistance(-52_in);
}


static void CLOSE_BLUE()
{
  shoot(1);
  leftMotF.move(50);
  leftMotB.move(50);
  rightMotF.move(50);
  rightMotB.move(50);
  pros::Task::delay(200);
  leftMotF.move(0);
  leftMotB.move(0);
  rightMotF.move(0);
  rightMotB.move(0);
  pros::Task::delay(1500);
  vision_READ(R_FLAG, -35, -5, true);
  pros::Task::delay(500);
  //shoot top top left flag
  load(300);

  leftMotF.move(-50);
  leftMotB.move(-50);
  rightMotF.move(-50);
  rightMotB.move(-50);
  pros::Task::delay(200);
  leftMotF.move(0);
  leftMotB.move(0);
  rightMotF.move(0);
  rightMotB.move(0);

	//realign after shooting from an angle
	Chassis.turnAngle(8_deg);
	//speed up the chassis again, we only have a minute after all.
	Chassis.setMaxVelocity(150);
  Chassis.turnAngle(-103_deg);
	//toggle the bottom left flag after shooting.

  loadf();
	Chassis.moveDistance(36.5_in);
  stopLoader();
  Chassis.moveDistance(-39_in);

  Chassis.turnAngle(101_deg);

  vision_READ(R_FLAG, -35, -5, true);

  Chassis.setMaxVelocity(120);

  Chassis.moveDistance(28_in);
  load(1000);

  Chassis.turnAngle(16_deg);
  Chassis.moveDistance(20_in);
}

//start at red and move to gray, move forward to back of next tick
static void CLOSE_RED()
{
  shootS(1, 480);

  Chassis.setMaxVelocity(130);
  Chassis.moveDistance(44_in);
  Chassis.moveDistance(-42_in);

  vision_READ(B_FLAG, 0, 20, true);

  load(500);

  Chassis.moveDistance(-8_in);
  Chassis.turnAngle(80_deg);
  loadf();
  Chassis.moveDistance(42_in);
  stopLoader();

  Chassis.turnAngle(-70_deg);
  Chassis.moveDistance(14_in);
  vision_READ(B_FLAG, 0, 20, true);
  load(1000);
 // Chassis.turnAngle(-10_deg);
  Chassis.moveDistance(38_in);

}

//start front of first tick, farthest from flag

//back of second tick
static void skillz()
{
  //shoot top top left flag
  shoot(1);
  vision_READ(B_FLAG, -5, 20, true);
  pros::Task::delay(2000);
  loadf();
  pros::Task::delay(1250);
  stopShooter();
  stopLoader();

  //slow down the chassis to get more accurate turns.
  Chassis.setMaxVelocity(50);
  //realign after shooting from an angle
  Chassis.turnAngle(-8_deg);
  //speed up the chassis again, we only have a minute after all.
  Chassis.setMaxVelocity(125);
  //toggle the bottom left flag after shooting.
  Chassis.moveDistance(48_in);

  //get out of flag and turn towards cap(first ball)
  Chassis.moveDistance(-46_in);
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
  vision_READ(B_FLAG, -5, 20, true);
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
  Chassis.turnAngle(-98_deg);
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
  vision_READ(B_FLAG, -5, 20, true);
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
  Chassis.moveDistance(-6.5_ft);
}


void autonomous()
{
	CLOSE_RED();
}
