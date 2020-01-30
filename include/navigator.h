#ifndef INCLUDE_NAVIGATOR_H
#define INCLUDE_NAVIGATOR_H

#include <ros/ros.h>
#include <cmath>
#include "wavefront_detector.h"

#define OPEN_ENV_VEL 0.25 // [m/s]
#define OBST_DET_VEL 0.1 // [m/s]
#define MAX_ANG_VEL M_PI/6 // [rad/s]

// global variables
extern int rob_state;
extern float rob_pos_x;
extern float rob_pos_y;
extern float goal_pos_x;
extern float goal_pos_y;
extern float yaw;
extern float min_laser_dist; 
extern int32_t n_lasers;
extern int32_t desired_n_lasers; 
extern int32_t view_angle; // +-5 deg from heading angle
extern bool detect_frontier; // wfd enable flag
extern bool bumper_hit; // recovery mode flag
extern uint8_t bumper[3];

// objects
extern Wavefront_Detector wfd;

class Navigator
{
 private:

    float angular_vel; // <= M_PI/6 [rad/s]
    float linear_vel; // <= 0.25 [m/s]
    ros::Publisher;

 public:

    // Accessors
    float get_angular_vel() {return angular_vel;}
    float get_linear_vel() {return linear_vel;}
    
    // Commands
    void stop() {angular_vel = 0.0; linear_vel = 0.0;}
    void move_to_goal_point(float goal_x, float goal_y);

    // Constructor and destructor
    Navigator();
    ~Navigator() {};
};

extern Navigator nav;

#endif  // INCLUDE_NAVIGATOR_H