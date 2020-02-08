#ifndef INCLUDE_CSRW_H
#define INCLUDE_CSRW_H

#include <ros/ros.h>
#include <ros/console.h>
#include <cmath>
#include <chrono>

// global robot state variables
extern float rob_yaw;
extern float rob_pos_x;
extern float rob_pos_y;
extern float goal_pos_x;
extern float goal_pos_y;
extern bool bumper_hit;
extern float front_laser_dist;
extern float left_laser_dist;
extern float right_laser_dist;
extern uint8_t bumper[NUM_BUMPER];

class CSRW // corner-seeking random walk
{
 private:
    // state vars

 public:
    // commands
    
    // constructor and destructor
    ~CSRW() {};
};

#endif  // INCLUDE_CSRW_H