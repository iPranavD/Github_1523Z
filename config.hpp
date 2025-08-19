#pragma once
q
#include "lemlib/api.hpp"

// External declarations for hardware components

// Controller
extern pros::Controller controller;

// Motor groups
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup leftMotors;

// Individual motors
extern pros::Motor toprollers;
extern pros::Motor bottomrollers;

// Sensors
extern pros::Imu imu;

// Pneumatics
extern pros::adi::DigitalOut blocker;
extern pros::adi::DigitalOut scraper;
extern pros::adi::DigitalOut park;
// LemLib chassis
extern lemlib::Chassis chassis;

// Configuration functions
void initializeHardware();
void setheading(float degrees, int maxspeed, int timeout = 0);