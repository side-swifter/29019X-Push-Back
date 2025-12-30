#pragma once

#include "EZ-Template/piston.hpp"
#include "pros/adi.h"
#include "pros/motor_group.hpp"
#include "pros/vision.h"
#pragma once

#include "pros/motors.hpp"
#include "EZ-Template/drive/drive.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examplesb
inline pros::MotorGroup intake({7,-8,-10});  // Negative port will reverse the motor
inline pros::Motor SS(-8); 
inline pros::Motor TS(-10); 

inline ez::Piston scraper('E');     
inline ez::Piston switcher('H');
inline ez::Piston hood('G');
inline ez::Piston descore('A');



// sensors
inline pros::Optical CS(4);
