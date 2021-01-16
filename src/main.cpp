#include "main.h"
#include "Robot.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    Robot::initialize();
    lcd::initialize();
}

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
void autonomous()
{
    Task trackPos(Robot::trackPosition, "Tracking");
    Task displayPos(Robot::displayPosition, "Display");
    // Robot::moveTo(Robot::Position{12, 20, 90}, 20, 1);
    // Robot::move_to({12, 12, 0}, {1, 1, 1}, {3, 3, 3});
}

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
void opcontrol()
{
    while (true)
    {
        Robot::drive();
        if (Robot::controller.get_digital(E_CONTROLLER_DIGITAL_R1) == 1)
        {
            Robot::intakes(127);
        }
        else if (Robot::controller.get_digital(E_CONTROLLER_DIGITAL_L1) == 1)
        {
            Robot::intakes(-127);
        }
        else
        {
            Robot::intakes(0);
        }

        if (Robot::controller.get_digital(E_CONTROLLER_DIGITAL_R2) == 1)
        {
            Robot::rollers(127);
        }
        else if (Robot::controller.get_digital(E_CONTROLLER_DIGITAL_L2) == 1)
        {
            Robot::rollers(-127);
        }
        else
        {
            Robot::rollers(0);
        }

        lcd::print(0, "Right Enc: %d", Robot::RightEncoder.get_value());
        lcd::print(1, "Left Enc: %d", Robot::LeftEncoder.get_value());
        lcd::print(2, "Back Enc: %d", Robot::BackEncoder.get_value());
        lcd::print(3, "Inertial: %0.2f", Robot::Inertial.get_heading());
        delay(20);
    }
}
