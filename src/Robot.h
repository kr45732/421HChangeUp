#include "main.h"
#include <vector>
using namespace pros;

class Robot
{
public:
    /* -- Initializing and constants --  */
    /* Controller */
    static Controller controller;

    /* Chassis */
    static Motor FrontLeft;
    static Motor FrontRight;
    static Motor BackLeft;
    static Motor BackRight;

    /* Intakes */
    static Motor RightIntake;
    static Motor LeftIntake;

    /* Rollers */
    static Motor BottomRoller;
    static Motor TopRoller;

    /* Encoders */
    static ADIEncoder LeftEncoder;
    static ADIEncoder RightEncoder;
    static ADIEncoder BackEncoder;

    /* Inertial */
    static Imu Inertial;

    /* Robot Position */
    struct Position
    {
        double x;
        double y;
        double angle;

    } static robotPos;

    struct EncoderValues
    {
        int prevLeft;
        int prevRight;
        int prevBack;
    } static prevEncValues;

    /* Odometry constants */
    static double LEFT_OFFSET;
    static double RIGHT_OFFSET;
    static double BACK_OFFSET;

    static double LEFT_WHEEL;
    static double RIGHT_WHEEL;
    static double BACK_WHEEL;

    /* -- Functions --  */
    /* Initialization */
    static void initialize();

    /* Autonomous Control */
    static void trackPosition();
    static void moveTo(Position targetPosition, double speed, double distanceError);

    /* Driver Control */
    static void drive();

    /* Helper Functions */
    static double toRadians(double degrees);
    static double toDegrees(double radians);
    static double calculateX(double desiredAngle, double speed);
    static double calculateY(double desiredAngle, double speed);
    static void moveChassis(double power, double strafe, double turn);
    static void displayPosition();
    static void intakes(int speed = 127);
    static void move_to(std::vector<double> pose, std::vector<double> margin, std::vector<double> speeds);
    static void rollers(int speed = 127);
};