#include "config.hpp"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drivetrain motors
pros::MotorGroup rightMotors({15, 16, -17}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({-18, -19, 20}, pros::MotorGearset::blue);

// Indexing/intake motors
pros::Motor bottomrollers(-14, pros::MotorGearset::green);
pros::Motor toprollers(-13, pros::MotorGearset::green);

// Sensors
pros::Imu imu(8);
pros::adi::DigitalOut blocker(1);
pros::adi::DigitalOut scraper(2);

// Drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              14, // 14 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 3.25" omnis
                              343, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// Lateral motion controller
lemlib::ControllerSettings linearController(12, // proportional gain (kP)
                                            0.1, // integral gain (kI)
                                            3.5, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            13 // maximum acceleration (slew)
);

// Angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0.1, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// Sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// Input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// Input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// Create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// Turn to heading function
void setheading(float degrees, int maxspeed, int timeout) {
    if (timeout > 0) {
        // turn to specified heading with a timeout
        chassis.turnToHeading(degrees, timeout, {.maxSpeed = maxspeed}, false);
    } else {
        // Turn to the specified heading without a timeout
        chassis.turnToHeading(degrees, 0, {.maxSpeed = maxspeed}, false);

        // Wait until the chassis has stopped moving
        chassis.waitUntilDone();
    }
}

// Initialize all hardware components
void initializeHardware() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    
    // Initialize optical sensors
    optical1.set_led_pwm(100);
    optical2.set_led_pwm(100);
    optical3.set_led_pwm(100);
    
    printf("Hardware initialization complete\n");
}