#ifndef INCLUDE_CSRW_H
#define INCLUDE_CSRW_H

#include <ros/ros.h>
#include <ros/console.h>
#include <cmath>
#include <chrono>
#include <array>
#include "navigator.h"

#define TOP_BOUNDARY 0
#define TOP_RIGHT_CRNR 1
#define TOP_LEFT_CRNR 2
#define BOT_LEFT_CRNR 3
#define BOT_RIGHT_CRNR 4
#define RENAV_TOP_RIGHT_CRNR 5
#define CNTR 6
#define RW 7

#define EXTRM_DIST 10.0 // [m] extreme distance

// global robot state variables
extern float rob_yaw;
extern float rob_pos_x;
extern float rob_pos_y;
extern float min_pos_x;
extern float max_pos_x;
extern float min_pos_y;
extern float max_pos_y;
extern bool bumper_hit;
extern float front_laser_dist;
extern float left_laser_dist;
extern float right_laser_dist;
extern uint8_t bumper[NUM_BUMPER];

class CSRW // corner-seeking random walk
{
 private:
    Navigator nav;
    std::array<float, 2> top_right_crnr;
    std::array<float, 2> top_left_crnr;
    std::array<float, 2> bot_right_crnr;
    std::array<float, 2> bot_left_crnr;
    std::array<float, 2> cntr;

    float goal_pos_x;
    float goal_pos_y;
    bool go_to_cntr_flag;
    uint8_t goal_stage; // strategy state

    // TODO: add time limit to stages
    // TODO: add 360 rot to move_to...

 public:
    // commands
    void set_goal();
    void go_to_goal() { nav.move_to(goal_pos_x, goal_pos_y); }
    void rotate() { nav.rotate(DEG2RAD(360), MAX_ANG_VEL, CW); }
    bool crnrs_complete() { return go_to_cntr_flag; }
    void do_rw();
    void pause() { nav.stop(); }
    void respond_to_bump();

    // constructor and destructor
    CSRW(ros::NodeHandle* nh);
    ~CSRW() {};
};

#endif  // INCLUDE_CSRW_H