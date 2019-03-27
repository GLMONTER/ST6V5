#include "main.h"
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
pros::ADIUltrasonic backR (B_R_O, B_R_Y);
pros::ADIUltrasonic backL(B_L_O, B_L_Y);

pros::ADIUltrasonic frontR (F_R_O, F_R_Y);
pros::ADIUltrasonic frontL(F_L_O, F_L_Y);

//left drive train, both not reversed
pros::Motor leftMotB(1, pros::E_MOTOR_GEARSET_18, false);
pros::Motor leftMotF(2, pros::E_MOTOR_GEARSET_18, false);

//the right drive train, both reversed
pros::Motor rightMotB(11, pros::E_MOTOR_GEARSET_18, true);
pros::Motor rightMotF(12, pros::E_MOTOR_GEARSET_18, true);

//the fly wheel, with the high speed gearset, one reversed one not.
pros::Motor fly(20, pros::E_MOTOR_GEARSET_06, true);
pros::Motor fly2(10, pros::E_MOTOR_GEARSET_06, false);

//both of the loaders, one reversed one not.
pros::Motor LoadServ(19, pros::E_MOTOR_GEARSET_18, true);
pros::Motor LoadServ2(9, pros::E_MOTOR_GEARSET_18, false);

//the vision sensor, telling the sensor to put the origin in the center, (0,0).
pros::Vision vSensor(8, pros::E_VISION_ZERO_CENTER);

//the main controller variable.
pros::Controller mController(pros::E_CONTROLLER_MASTER);

void initialize()
{
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
