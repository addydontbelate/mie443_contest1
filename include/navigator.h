#ifndef INCLUDE_NAVIGATOR_H
#define INCLUDE_NAVIGATOR_H

#include <ros/ros.h>
#include <cmath>

#define OPEN_ENV_VEL 0.25 // [m/s]
#define OBST_DET_VEL 0.1 // [m/s]
#define MAX_ANG_VEL M_PI/6 // [rad/s]

class Navigator
{
 private:
    float angular_vel; // <= M_PI/6 [rad/s]
    float linear_vel; // <= 0.25 [m/s]

 public:
    // accessors
    float get_angular_vel() {return angular_vel;}
    float get_linear_vel() {return linear_vel;}
    
    // commands
    void stop() {angular_vel = 0.0; linear_vel = 0.0;}

    // constructor and destructor
    Navigator() {angular_vel = 0.0; linear_vel = 0.0;}
    ~Navigator() {};
};

#endif  // INCLUDE_NAVIGATOR_H