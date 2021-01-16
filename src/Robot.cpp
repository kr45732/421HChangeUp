#include "Robot.h"
#include <vector>
using namespace pros;

/* Initializing motors, sensors, controller */
Controller Robot::controller(E_CONTROLLER_MASTER);
Motor Robot::FrontLeft(14);
Motor Robot::FrontRight(8, true);
Motor Robot::BackLeft(14);
Motor Robot::BackRight(6, true);

Motor Robot::LeftIntake(4);
Motor Robot::RightIntake(19, true);

Motor Robot::BottomRoller(12, true);
Motor Robot::TopRoller(7);

ADIEncoder Robot::LeftEncoder(1, 2);
ADIEncoder Robot::RightEncoder(7, 8, true);
ADIEncoder Robot::BackEncoder(1, 2);

Imu Robot::Inertial(5);

/* Odometry presets */
Robot::Position Robot::robotPos = {0, 0, 0};
Robot::EncoderValues Robot::prevEncValues = {LeftEncoder.get_value(), RightEncoder.get_value(), BackEncoder.get_value()};

double Robot::LEFT_OFFSET = 7.5;
double Robot::RIGHT_OFFSET = 7.5;
double Robot::BACK_OFFSET = 7.25;

double Robot::LEFT_WHEEL = 4 * M_PI / 360;
double Robot::RIGHT_WHEEL = 4 * M_PI / 360;
double Robot::BACK_WHEEL = 2.75 * M_PI / 360;

/* Functions */
void Robot::initialize()
{
    Inertial.reset();
}

void Robot::trackPosition()
{
    while (true)
    {
        int leftEnc = LeftEncoder.get_value();
        int rightEnc = RightEncoder.get_value();
        int backEnc = BackEncoder.get_value();

        double leftMove = (leftEnc - prevEncValues.prevLeft) * LEFT_WHEEL;     // The amount the left side of the robot moved
        double rightMove = (rightEnc - prevEncValues.prevRight) * RIGHT_WHEEL; // The amount the right side of the robot moved
        double backMove = (backEnc - prevEncValues.prevBack) * BACK_WHEEL;     // The amount the back side of the robot moved

        // Update the last values
        prevEncValues.prevLeft = leftEnc;
        prevEncValues.prevRight = rightEnc;
        prevEncValues.prevBack = backEnc;

        double h;                                                         // The hypotenuse of the triangle formed by the middle of the robot on the starting position and ending position and the middle of the circle it travels around
        double i;                                                         // Half on the angle that I've traveled
        double h2;                                                        // The same as h but using the back instead of the side wheels
        double a = (leftMove - rightMove) / (LEFT_OFFSET + RIGHT_OFFSET); // The angle that I've traveled
        if (a)
        {
            double r = rightMove / a; // The radius of the circle the robot travel's around with the right side of the robot
            i = a / 2.0;
            double sinI = sin(i);
            h = ((r + RIGHT_OFFSET) * sinI) * 2.0;

            double r2 = backMove / a; // The radius of the circle the robot travel's around with the back of the robot
            h2 = ((r2 + BACK_OFFSET) * sinI) * 2.0;
        }
        else
        {
            h = rightMove;
            i = 0;

            h2 = backMove;
        }
        double p = i + a; // The global ending angle of the robot
        double cosP = cos(p);
        double sinP = sin(p);

        // Update the global position
        robotPos.y += h * cosP;
        robotPos.x += h * sinP;

        robotPos.y += h2 * -sinP; // -sin(x) = sin(-x)
        robotPos.x += h2 * cosP;  // cos(x) = cos(-x)

        robotPos.angle += a;
        delay(5);
    }
}

/*
void Robot::moveTo(Position targetPosition, double speed, double distanceError)
{
    double distanceToX = targetPosition.x - robotPos.x;
    double distanceToY = targetPosition.y - robotPos.y;

    double distance = hypot(distanceToX, distanceToY);
    // double distance = targetPosition.angle - robotPos.angle;
    while (distance > distanceError)
    {

        distanceToX = targetPosition.x - robotPos.x;
        distanceToY = targetPosition.y - robotPos.y;

        double angleError = toDegrees(atan2(distanceToX, distanceToY));

        double moveX = calculateX(angleError, speed);
        double moveY = calculateY(angleError, speed);
        double pivotCorrection = targetPosition.angle - robotPos.angle;
        pivotCorrection = (pivotCorrection > 40) ? 40 : pivotCorrection;

        lcd::print(4, "Distance: %0.2f", distance);
        lcd::print(5, "Turn: %0.2f", pivotCorrection);

        moveChassis(moveY, moveX, pivotCorrection);

        distance = hypot(distanceToX, distanceToY);
        // distance = targetPosition.angle - robotPos.angle;

        delay(10);
    }
    moveChassis(0, 0, 0);
    lcd::print(6, "DONE");
}

void Robot::move_to(std::vector<double> pose, std::vector<double> margin, std::vector<double> speeds)
{
    double new_y = pose[0];
    double new_x = pose[1];
    double heading = pose[2];

    double y_error = new_y - robotPos.y;
    double x_error = -(new_x - robotPos.x);

    double heading2 = (heading < 0) ? heading + 360 : heading - 360;
    heading = (abs(Inertial.get_rotation() - heading) < abs(Inertial.get_rotation() - heading2)) ? heading : heading2;
    double imu_error = -(Inertial.get_rotation() - heading);
    //Calculate inverse headings (i.e. 1 deg = -359 deg), then find which heading is closer to current heading. For
	//example, moving to -358 deg would require almost a full 360 degree turn from 1 degree, but from its equivalent of -359
	//deg, it only takes a minor shift in position 

    while (abs(y_error) > margin[0] || abs(x_error) > margin[1] || abs(imu_error) > margin[2])
    { // while Robot::y, Robot::x and IMU heading are all more than the specified margin away from the target
        double phi = toRadians(Inertial.get_rotation());
        double power = (y_error * std::cos(phi) + x_error * std::sin(phi)) * speeds[0];
        double strafe = (x_error * std::cos(phi) - y_error * std::sin(phi)) * speeds[1];
        // if (strafe > 20)
        // {
        //     strafe = 20;
        // }
        // else if (strafe < 20)
        // {
        //     strafe = -20;
        // }
        double turn = (imu_error)*1.5 * speeds[2];
        lcd::print(4, "Y: %0.2f  X: %0.2f  R: %0.2f", y_error, x_error, imu_error);
        lcd::print(5, "P: %0.2f  S: %0.2f  T: %0.2f", power, strafe, turn);
        moveChassis(power, strafe, turn);
        //Using our PD objects we use the error on each of our degrees of freedom (axial, lateral, and turning movement)
		// to obtain speeds to input into Robot::mecanum. We perform a rotation matrix calculation to translate our y and x
		// error to the same coordinate plane as Robot::y and Robot::x to ensure that the errors we are using are indeed
		// proportional/compatible with Robot::y and Robot::x 

        imu_error = -(Inertial.get_rotation() - heading);
        y_error = new_y - robotPos.y;
        x_error = -(new_x - robotPos.x);
        // Recalculating our error by subtracting components of our current position vector from target position vector 

        delay(5);
    }
    lcd::print(6, "DONE");
    moveChassis(0, 0, 0);
}
*/

void Robot::drive()
{
    int power = controller.get_analog(ANALOG_LEFT_Y);
    int strafe = controller.get_analog(ANALOG_LEFT_X);
    int turn = controller.get_analog(ANALOG_RIGHT_X);

    moveChassis(power, strafe, turn);

    delay(10);
}

double Robot::calculateX(double desiredAngle, double speed)
{
    return sin(toRadians(desiredAngle)) * speed;
}

double Robot::calculateY(double desiredAngle, double speed)
{
    return cos(toRadians(desiredAngle)) * speed;
}

double Robot::toRadians(double degrees)
{
    return degrees * M_PI / 180;
}

double Robot::toDegrees(double radians)
{
    return radians * 180 / M_PI;
}

void Robot::moveChassis(double power, double strafe, double turn)
{
    FrontLeft.move(power + strafe + turn);
    FrontRight.move(power - strafe - turn);
    BackLeft.move(power - strafe + turn);
    BackRight.move(power + strafe - turn);
}

void Robot::displayPosition()
{
    while (true)
    {
        lcd::print(0, "X: %0.2f", Robot::robotPos.x);
        lcd::print(1, "Y: %0.2f", Robot::robotPos.y);
        // lcd::print(2, "Angle: %0.2f", (Robot::robotPos.angle * 180 / M_PI));
        lcd::print(2, "Inertial: %0.2f", (Robot::Inertial.get_rotation()));
        delay(20);
    }
}

void Robot::intakes(int speed)
{
    LeftIntake.move(speed);
    RightIntake.move(speed);
}
void Robot::rollers(int speed)
{
    TopRoller.move(speed);
    BottomRoller.move(speed);
}