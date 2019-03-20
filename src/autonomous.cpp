#include "main.h"
//for std::abs
#include<cstdlib>
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


using namespace okapi;

//for checking the error state of errno fo debugging.
extern int errno;

//the actaull signature data from the vision sensor utility.
pros::vision_signature_s_t B_FLAG = pros::Vision::signature_from_utility(2, -3121, 85, -1518, 2743, 12577, 7660, 1.2, 0);
pros::vision_signature_s_t R_FLAG;
pros::vision_signature_s_t GR = pros::Vision::signature_from_utility(1, -2591, -1797, -2194, -5113, -3723, -4418, 3, 0);

pros::ADIUltrasonic frontR (F_R_O, F_R_Y);
pros::ADIUltrasonic frontL(F_L_O, F_L_Y);
//a function that alligns with the green part of the flag using the vision sensor, finds the largest green object on flags.

static void vision_READ(pros::vision_signature_s_t signature, int MAX_LEFT, int MAX_RIGHT)
{
  //basically resetting the vision sensor.
	vSensor.clear_led();

  //set the blue signature to an index that can be referenced later.
  vSensor.set_signature(0, &signature);

  //infinate loop so we can update the position of the vision object we find, for allignment.
	while(true)
	{
		//get the largest object(0), based on the signature passed in.
		//we call this every update to get the new position of the object
		pros::vision_object_s_t rtn = vSensor.get_by_sig(0, 1);

		//if it is within range, stop the motors.
		if(!(rtn.x_middle_coord < MAX_LEFT) && !(rtn.x_middle_coord > MAX_RIGHT))
		{
			rightMotB.move(0);
			leftMotB.move(0);

			rightMotF.move(0);
			leftMotF.move(0);
			break;
		}
		//if the object is to the left, then turn the robot left.
		if(rtn.x_middle_coord < MAX_LEFT)
		{
			rightMotB.move(50);
			leftMotB.move(-50);

			rightMotF.move(50);
			leftMotF.move(-50);
		}
		else
		{
			//if the object is to the right, then turn the robot right.
			if(rtn.x_middle_coord > MAX_RIGHT)
			{
				rightMotB.move(-60);
				leftMotB.move(60);

				rightMotF.move(-60);
				leftMotF.move(60);
			}
		}
		//so we don't starv other tasks like updating the LCD
		pros::Task::delay(10);
	}
}
static int getDif()
{
	int ret = 0;

	pros::Task::delay(50);
	ret += frontR.get_value();
	pros::Task::delay(50);
	ret -= frontL.get_value();


	return std::abs(ret);

}

static int getLeft()
{
	pros::Task::delay(50);
	return frontL.get_value();
}

static int getRight()
{
	pros::Task::delay(50);
	return frontR.get_value();
}
//a test function for using the ultrasonic sensor to line the robot up without hitting a wall.
static void allignFront(int dis)
{
  while(true)
  {
    if(getDif() < 50)
    {
        break;
    }
    else
    {
      if(getRight() > getLeft())
      {
        rightMotB.move(50);
        leftMotB.move(-50);
      }
      else
      {
        rightMotB.move(-50);
        leftMotB.move(50);
      }
    }
    pros::Task::delay(10);
  }
  leftMotB.move(0);
  leftMotF.move(0);

  rightMotB.move(0);
  rightMotF.move(0);


	int temp;
  while(std::abs(temp - dis) > 100)
  {
    temp = ((getRight() + getLeft()) / 2);

    if(temp > dis)
    {
      leftMotB.move(40);
      leftMotF.move(40);

      rightMotB.move(40);
      rightMotF.move(40);
    }
    else
    {
      leftMotB.move(-40);
      leftMotF.move(-40);

      rightMotB.move(-40);
      rightMotF.move(-40);
    }
    pros::Task::delay(10);
  }
  leftMotB.move(0);
  leftMotF.move(0);

  rightMotB.move(0);
  rightMotF.move(0);
}

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
 //load normally for a set ammount of miliseconds, then stop the loader.
 static void load(std::uint32_t mili)
 {
   LoadServ.move(127);
   LoadServ2.move(127);
   pros::Task::delay(mili);
   LoadServ.move(0);
   LoadServ2.move(0);
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
   LoadServ.move(0);
   LoadServ2.move(0);
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
   fly.move_velocity(530);
   fly2.move_velocity(530);
   pros::Task::delay(mili);
 }
//the robot chassis
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
//front of second tick, far from flag.
static void farRed()
{
  Chassis.moveDistance(1_ft);
  Chassis.turnAngle(10_deg);
  shoot(2500);
  loadf();
  pros::Task::delay(1500);
  Chassis.moveDistance(-1_ft);
  Chassis.turnAngle(70_deg);
  Chassis.moveDistance(40_in);
  Chassis.moveDistance(-4_in);
  Chassis.turnAngle(97_deg);
  Chassis.moveDistance(-3_ft);
  /*
  Chassis.setMaxVelocity(100);
  Chassis.moveDistance(24_in);
  Chassis.turnAngle(90_deg);
  Chassis.setMaxVelocity(200);
  Chassis.moveDistance(3.5_ft);
  */
}


static void ALT_BLUE_C()
{
	//shoot top top left flag
	shoot(1500);
	loadf();
	pros::Task::delay(200);
	stopLoader();

	//slow down the chassis to get more accurate turns.
	Chassis.setMaxVelocity(50);
	//realign after shooting from an angle
	Chassis.turnAngle(7_deg);
	//speed up the chassis again, we only have a minute after all.
	Chassis.setMaxVelocity(160);
	//toggle the bottom left flag after shooting.
	Chassis.moveDistance(47_in);

	//get out of flag and turn towards cap(first ball)
	Chassis.moveDistance(-44_in);
	Chassis.turnAngle(-102_deg);
	Chassis.moveDistance(-10_in);
	//turn on the loader to get the ball and drive for it.
	loadf();
	Chassis.moveDistance(44_in);
	//give time to load
	stopLoader();

	//turn towards flag and move back a bit to get base allignment.
	Chassis.turnAngle(87_deg);
//  Chassis.moveDistance(-2_in);
	Chassis.moveDistance(3_ft);
	//allign with the biggest green object on flag on the x coord.
	//readVB();

	//reverse to make sure we didn't load to much and the ball will get caught in the fly wheel
	loadR(100);
	//shoot and stop the loader and shooter
	load(1000);
	stopLoader();
	stopShooter();

		Chassis.turnAngle(10_deg);

		Chassis.setMaxVelocity(200);
	//hit the bottom middle flag
	Chassis.moveDistance(28_in);
}


//start at red and move to gray, move forward to back of next tick
static void ALT_RED_C()
{
	//shoot top top left flag
	shoot(1500);
	loadf();
	pros::Task::delay(200);
	stopLoader();

	//slow down the chassis to get more accurate turns.
	Chassis.setMaxVelocity(50);
	//realign after shooting from an angle
	Chassis.turnAngle(-5_deg);
	//speed up the chassis again, we only have a minute after all.
	Chassis.setMaxVelocity(160);
	//toggle the bottom left flag after shooting.
	Chassis.moveDistance(47_in);

	//get out of flag and turn towards cap(first ball)
	Chassis.moveDistance(-46_in);
	Chassis.turnAngle(102_deg);
	Chassis.moveDistance(-11_in);

	//turn on the loader to get the ball and drive for it.
	loadf();
	Chassis.moveDistance(43_in);

	stopLoader();

	//turn towards flag and move back a bit to get base allignment.
	Chassis.turnAngle(-76_deg);
//  Chassis.moveDistance(-2_in);
	Chassis.moveDistance(3_ft);
	//allign with the biggest green object on flag on the x coord.
	//readV();

	//shoot and stop the loader and shooter
	load(1000);
	stopLoader();
	stopShooter();

		Chassis.turnAngle(-15_deg);

		Chassis.setMaxVelocity(200);
	//hit the bottom middle flag
	Chassis.moveDistance(28_in);
}

static void farBlue()
{
  shoot(2500);
  loadf();
  pros::Task::delay(2000);
  /*
  Chassis.setMaxVelocity(100);
  Chassis.moveDistance(24_in);
  Chassis.turnAngle(-90_deg);
  Chassis.setMaxVelocity(200);
  Chassis.moveDistance(3.5_ft);
  */
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
	while(true)
	{
		vision_READ(GR, -35, 15);
	}
}
