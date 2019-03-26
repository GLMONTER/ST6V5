#include "main.h"
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

//toggle and press bool for forward motion of drum
int buttonToggleF = 0;
int buttonPressedF = 0;

//toggle and press bool for backward motion of drum
int buttonToggleR = 0;
int buttonPressedR = 0;

//the toggle and press variables for the fly wheel
int flyToggle = 0;
int flyPressed = 0;

//the reverse drive variables for toggle and press
int reverseToggle = 0;
int reversePressed = 0;

//a function that polls the controls for the fly wheel the acts on the fly wheel based on the controls.
void pollTFly()
{
	//the toggle for the fly wheel.
	if(mController.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
	{
		if(!flyPressed)
		{
			flyToggle = 1 - flyToggle;

			flyPressed = 1;
		}
	}
	else
		flyPressed = 0;

	//if the fly wheel toggle is true, set the shooter to 600 RPM, else turn it off
	if(flyToggle)
	{
		fly.move_velocity(600);
    fly2.move_velocity(600);
	}
	else
  {
		if(!flyToggle)
		{
			fly.move(0);
      fly2.move(0);
		}
  }
}

//function for polling the input for the loader, and acting on the loader.
void pollToggles()
{
	//go forward with drum
	if(mController.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
	{
		//if the forward button toggle isn't on then continute
		if(!buttonPressedF)
		{
			//actaully flip the toggle, this is why the type has to be int
			buttonToggleF = 1 - buttonToggleF;
			//changed button pressed to true
			buttonPressedF = 1;
			//change the backward toggle to false so we don't try to go backwards and forwards
			buttonToggleR = false;
		}
	}
	//switch back to normal buttton state but leave toggle on if button isn't pressed.
	else
		buttonPressedF = 0;

//if our forward toggle is on, then eat the balls :D
if(buttonToggleF == true)
{
	LoadServ.move(90);
  LoadServ2.move(90);
}
//check if other toggle is on if we need to really stop the motor
else
{
	if(!buttonToggleR && !buttonToggleF)
	{
		LoadServ.move(0);
	  LoadServ2.move(0);
	}
}
//go backwards with drum
if(mController.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
{
	//if we haven't pressed the button then toggle the button
	if(!buttonPressedR)
	{
		//actually toggle the button, this is why the type is int
		buttonToggleR = 1 - buttonToggleR;

		buttonPressedR = 1;

		//so we stop going forward.
		buttonToggleF = false;
	}
}
//else, then turn button pressed to false
else
	buttonPressedR = 0;

//if backward button toggle is on, then start the motor backward
if(buttonToggleR == true)
{
	LoadServ.move(-90);
  LoadServ2.move(-90);
}
	//else, check if the forward toggle is off, then stop.
	else
	{
		if(!buttonToggleF && !buttonToggleR)
		{
			LoadServ.move(0);
	  	LoadServ2.move(0);
		}
	}
}

bool rev = false;
//the driving code
void driveControl()
{
//infinite loop to keep the driving code going.
 while(true)
 {
	 //the toggle for reversing the drive train
	 if(mController.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
	 {
		 if(!reversePressed)
		 {
			 reverseToggle = 1 - reverseToggle;

			 reversePressed = 1;
		 }
	 }
	 else
		 reversePressed = 0;

	 if(reverseToggle)
	 {
		 rev = true;
	 }
	 else
	 {
		 if(!reverseToggle)
		 {
			 rev = false;
		 }
	 }
	 //end toggle code

	 //if we are not reversed, then drive normally, else drive reversed.
	 if(!rev)
	 {
		 	leftMotB.move(mController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
			leftMotF.move(mController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));

			rightMotB.move(mController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		 	rightMotF.move(mController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
	 }
	 else
	 {
		 leftMotB.move(-mController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		 leftMotF.move(-mController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

		 rightMotB.move(-mController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		 rightMotF.move(-mController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
	 }

	 //poll the shooter and loader
	 pollTFly();
	 pollToggles();
 }
}

//operator control entry point
void opcontrol()
{
	//actually set the signature from the data we collected from the vision utility.
	//set the data to index of 1 to reference later.
  driveControl();
}
