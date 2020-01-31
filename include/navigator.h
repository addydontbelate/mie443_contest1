#ifndef INCLUDE_NAVIGATOR_H
#define INCLUDE_NAVIGATOR_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <chrono>

// angle conversion macros
#define RAD2DEG(rad) ((rad) * 180./M_PI)
#define DEG2RAD(deg) ((deg) * M_PI/180.)

// velocity limits
#define FREE_ENV_VEL 0.25   // [m/s]
#define OBST_DET_VEL 0.1    // [m/s]
#define MAX_ANG_VEL M_PI/6  // [rad/s] 

// direction macros
#define FWD true
#define BCK false
#define CW true
#define CCW false

// global variables
extern nav_msgs::Odometry rob_pose;
extern float rob_yaw;
extern float rob_pos_x;
extern float rob_pos_y;
extern float goal_pos_x;
extern float goal_pos_y;
extern bool bumper_hit;
extern geometry_msgs::Twist rob_vel;
extern ros::Publisher vel_pub;
extern ros::Subscriber odom_sub;

class Navigator
{
 private:
    float angular_vel;  // <= M_PI/6 [rad/s]
    float linear_vel;   // <= 0.25 [m/s]
    void publish_move();

 public:
    // accessors
    float get_angular_vel() {return angular_vel;}
    float get_linear_vel() {return linear_vel;}
    
    // TODO: add bumper/obstacle avoidance to these functions!
    // commands
    void stop();
    void move_to(float goal_x, float goal_y);
    void move_straight(float dist, float linear_speed, bool forward);
    void rotate(float rad, float angular_speed, bool clockwise);

    // constructor and destructor
    Navigator() {angular_vel = 0.0; linear_vel = 0.0;};
    ~Navigator() {};
};

#endif  // INCLUDE_NAVIGATOR_H