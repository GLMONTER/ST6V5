#include"sensors.hpp"
#define BLUE_SIG 0
#define RED_SIG 1

//a function that alligns with the green part of the flag using the vision sensor, finds the object that matches the signature passed in.
 void vision_READ(pros::vision_signature_s_t sig, int MAX_LEFT, int MAX_RIGHT)
{
  //basically resetting the vision sensor.
	vSensor.clear_led();
  //set the blue signature to an index that can be referenced later.

  vSensor.set_signature(RED_SIG, &sig);

  //infinate loop so we can update the position of the vision object we find, for allignment.
	while(true)
	{
		//get the largest object(0), based on the signature passed in.
		//we call this every update to get the new position of the object
		pros::vision_object_s_t rtn = vSensor.get_by_sig(0, RED_SIG);

    //for driver level vision sensing.
    if(mController.get_digital(pros::E_CONTROLLER_DIGITAL_A))
    {
      break;
    }
		//std::cout<<"value : "<<rtn.x_middle_coord<<std::endl;
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
 int getDif(int side)
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
 int getLeft(enum Sides side)
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
 int getRight(int side)
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
 void allignBackH()
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
 void allignFront(int dis)
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
