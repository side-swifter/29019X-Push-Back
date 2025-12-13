#pragma once

#include "EZ-Template/piston.hpp"
#include "pros/adi.h"
#include "pros/motor_group.hpp"
#pragma once

#include "pros/motors.hpp"
#include "EZ-Template/drive/drive.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::MotorGroup intake({7,-8,9,-10});  // Negative port will reverse the motor 

inline ez::Piston scraper('H');     
inline ez::Piston switcher('G');
inline ez::Piston hood('F');
inline ez::Piston descore('A');
