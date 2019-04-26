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
 //left of 4, back of 5
 static void FAR_BLUE()
 {
   shootS(4000, 475);

   load(500);
   Chassis.setMaxVelocity(150);
   Chassis.turnAngle(-43_deg);
   loadf();
   Chassis.moveDistance(40_in);
   stopLoader();

   Chassis.moveDistance(-42_in);
   Chassis.turnAngle(90_deg);

   Chassis.moveDistance(24_in);

   Chassis.turnAngle(90_deg);
Chassis.setMaxVelocity(150);
   Chassis.moveDistance(-44_in);
 }


static void FAR_RED()
{
  shootS(4000, 460);

  load(500);
  Chassis.setMaxVelocity(150);
  Chassis.turnAngle(60_deg);
  loadf();
  Chassis.moveDistance(40_in);
  stopLoader();

  Chassis.moveDistance(-42_in);
  Chassis.turnAngle(-90_deg);

  Chassis.moveDistance(26_in);

  Chassis.turnAngle(-90_deg);
  Chassis.setMaxVelocity(150);
  Chassis.moveDistance(-46_in);
}


static void CLOSE_BLUE()
{
  shootS(1, 500);

  Chassis.setMaxVelocity(130);
  Chassis.moveDistance(44_in);
  Chassis.moveDistance(-48_in);
  Chassis.turnAngle(-8_deg);
  //vision_READ(R_FLAG, -20, -5, true);


  load(500);
Chassis.moveDistance(6_in);
  Chassis.moveDistance(-8_in);
  Chassis.turnAngle(-80_deg);
  loadf();
  Chassis.moveDistance(42_in);
  stopLoader();

  Chassis.turnAngle(70_deg);
  Chassis.moveDistance(26_in);
  vision_READ(R_FLAG, -20, -5, true);
  load(1000);
  Chassis.turnAngle(-5_deg);
  Chassis.moveDistance(38_in);
}

//start at red and move to gray, move forward to back of next tick
static void CLOSE_RED()
{
  shootS(1, 500);

  Chassis.setMaxVelocity(130);
  Chassis.moveDistance(44_in);
  Chassis.moveDistance(-48_in);

  vision_READ(B_FLAG, 0, 20, true);


  load(500);
Chassis.moveDistance(6_in);
  Chassis.moveDistance(-8_in);
  Chassis.turnAngle(82_deg);
  loadf();
  Chassis.moveDistance(42_in);
  stopLoader();

  Chassis.turnAngle(-70_deg);
  Chassis.moveDistance(26_in);
  vision_READ(B_FLAG, 0, 20, true);
  load(1000);
  Chassis.turnAngle(-8_deg);
  Chassis.moveDistance(38_in);
}

//start front of first tick, farthest from flag

//back of second tick
static void skillz()
{
  shootS(1, 500);

  Chassis.setMaxVelocity(130);
  Chassis.moveDistance(44_in);
  Chassis.moveDistance(-48_in);

  vision_READ(B_FLAG, 0, 20, true);


  load(500);
Chassis.moveDistance(6_in);
  Chassis.moveDistance(-8_in);
  Chassis.turnAngle(85_deg);
  loadf();
  Chassis.moveDistance(42_in);
  stopLoader();

  Chassis.turnAngle(-70_deg);
  Chassis.moveDistance(30_in);
  vision_READ(B_FLAG, 0, 20, true);
  load(1000);
  Chassis.turnAngle(-10_deg);
  Chassis.moveDistance(38_in);

  Chassis.moveDistance(-48_in);

  Chassis.turnAngle(-90_deg);
  Chassis.moveDistance(40_in);
  Chassis.turnAngle(-90_deg);
  Chassis.moveDistance(26_in);

  Chassis.turnAngle(90_deg);

  Chassis.moveDistance(-38_in);
}


void autonomous()
{
	skillz();
}
