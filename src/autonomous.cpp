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

//the actaull signature data from the vision sensor utility.
pros::vision_signature_s_t B_FLAG = pros::Vision::signature_from_utility(2, -3469, -2519, -2994, 11367, 14763, 13064, 5.9, 0);
pros::vision_signature_s_t R_FLAG = pros::Vision::signature_from_utility(1, 7505, 11963, 9734, -541, 1, -270, 3.5, 0);

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
		pros::vision_object_s_t rtn = vSensor.get_by_sig(0, 0);

		std::cout<<"value : "<<rtn.x_middle_coord<<std::endl;
		//if it is within range, stop the motors.
		if(!(rtn.x_middle_coord < MAX_LEFT) && !(rtn.x_middle_coord > MAX_RIGHT))
		{
			rightMotB.move_velocity(0);
			leftMotB.move_velocity(0);

			rightMotF.move_velocity(0);
			leftMotF.move_velocity(0);
			break;
		}
		//if the object is to the left, then turn the robot left.
		if(rtn.x_middle_coord < MAX_LEFT)
		{
			rightMotB.move(40);
			leftMotB.move(-40);

			rightMotF.move(40);
			leftMotF.move(-40);
		}
		else
		{
			//if the object is to the right, then turn the robot right.
			if(rtn.x_middle_coord > MAX_RIGHT)
			{
				rightMotB.move(-40);
				leftMotB.move(40);

				rightMotF.move(-40);
				leftMotF.move(40);
			}
		}
		//so we don't starv other tasks like updating the LCD
		pros::Task::delay(10);
	}
}
//finds the differnce between either the back or front 2 ultrasonic sensors
static int getDif(int side)
{
	int ret = 0;

	if(side == 0)
	{
		ret += frontR.get_value();
		//added a delay which seemed to fix some problems getting the value with the ultrasonic sensors..
		pros::Task::delay(50);
		ret -= frontL.get_value();
	}
	else
	{
		ret += backR.get_value();
		//added a delay which seemed to fix some problems getting the value with the ultrasonic sensors..
		pros::Task::delay(50);
		ret -= backL.get_value();
	}
	return std::abs(ret);
}

//gets the left ultrasonic sensor value either on back or front
static int getLeft(enum Sides side)
{
	//added a delay which seemed to fix some problems getting the value with the ultrasonic sensors..
	pros::Task::delay(50);
	if(side == 0)
	{
		return frontL.get_value();
	}
	else
	{
		return backL.get_value();
	}
}
//gets the right ultrasonic sensor value either on back or front
static int getRight(int side)
{
	//added a delay which seemed to fix some problems getting the value with the ultrasonic sensors..
	pros::Task::delay(50);
	if(side == 0)
	{
		return frontR.get_value();
	}
	else
	{
		return backR.get_value();
	}

}
//allign with the ultrasonic sensors facing the back
static void allignBackH()
{
	while(true)
  {
		std::cout<<"left : "<<getLeft(Sides::Back)<<std::endl;
		std::cout<<"right : "<<getRight(Sides::Back)<<std::endl;
    if(getDif(1) < 40)
    {
        break;
    }
    else
    {
      if(getRight(Sides::Back) > getLeft(Sides::Back))
      {
        rightMotB.move(-50);
        leftMotB.move(50);
      }
      else
      {
        rightMotB.move(50);
        leftMotB.move(-50);
      }
    }
    pros::Task::delay(10);
  }
  leftMotB.move(0);
  leftMotF.move(0);

  rightMotB.move(0);
  rightMotF.move(0);
}

//a test function for using the ultrasonic sensor to line the robot up without hitting a wall.
static void allignFront(int dis)
{
  while(true)
  {
    if(getDif(Sides::Front) < 50)
    {
        break;
    }
    else
    {
      if(getRight(Sides::Front) > getLeft(Sides::Front))
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
    temp = ((getRight(Sides::Front) + getLeft(Sides::Front)) / 2);

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
	vision_READ(R_FLAG, -5, 45);
	//shoot top top left flag
	shoot(1800);
	load(200);

	//slow down the chassis to get more accurate turns.
	Chassis.setMaxVelocity(50);
	//realign after shooting from an angle
	Chassis.turnAngle(7_deg);
	//speed up the chassis again, we only have a minute after all.
	Chassis.setMaxVelocity(170);
	//toggle the bottom left flag after shooting.
	Chassis.moveDistance(47_in);

	//get out of flag and turn towards cap(first ball)
	Chassis.moveDistance(-46_in);
	Chassis.turnAngle(-102_deg);
	allignBackH();
	Chassis.moveDistance(-11_in);


	//turn on the loader to get the ball and drive for it.
	loadf();
	Chassis.moveDistance(44_in);

	stopLoader();

	//turn towards flag and move back a bit to get base allignment.
	Chassis.turnAngle(76_deg);
//  Chassis.moveDistance(-2_in);
	Chassis.moveDistance(3_ft);
	//allign with the biggest green object on flag on the x coord.

	//shoot and stop the loader and shooter
	load(800);
	stopLoader();
	stopShooter();

	Chassis.turnAngle(15_deg);

	Chassis.setMaxVelocity(200);
	//hit the bottom middle flag
	Chassis.moveDistance(28_in);
}


//start at red and move to gray, move forward to back of next tick
static void ALT_RED_C()
{
	vision_READ(B_FLAG, -45, 5);
	//shoot top top left flag
	shoot(1800);
	load(200);

	//slow down the chassis to get more accurate turns.
	Chassis.setMaxVelocity(50);
	//realign after shooting from an angle
	Chassis.turnAngle(-7_deg);
	//speed up the chassis again, we only have a minute after all.
	Chassis.setMaxVelocity(170);
	//toggle the bottom left flag after shooting.
	Chassis.moveDistance(47_in);

	//get out of flag and turn towards cap(first ball)
	Chassis.moveDistance(-46_in);
	Chassis.turnAngle(102_deg);
	allignBackH();
	Chassis.moveDistance(-11_in);


	//turn on the loader to get the ball and drive for it.
	loadf();
	Chassis.moveDistance(44_in);

	stopLoader();

	//turn towards flag and move back a bit to get base allignment.
	Chassis.turnAngle(-76_deg);
//  Chassis.moveDistance(-2_in);
	Chassis.moveDistance(3_ft);
	//allign with the biggest green object on flag on the x coord.
	vision_READ(B_FLAG, -45, 5);

	//shoot and stop the loader and shooter
	load(800);
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
	ALT_RED_C();
}
