#ifndef INCLUDE_NAVIGATOR_H
#define INCLUDE_NAVIGATOR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <kobuki_msgs/BumperEvent.h>
#include <cmath>
#include <chrono>

#define NUM_BUMPER 3 // LEFT, CENTER, RIGHT

// angle conversion macros
#define RAD2DEG(rad) ((rad) * 180./M_PI)
#define DEG2RAD(deg) ((deg) * M_PI/180.)

// velocity and movement limits
#define FREE_ENV_VEL 0.25   // [m/s]
#define OBST_DET_VEL 0.1    // [m/s]
#define MAX_ANG_VEL M_PI/6  // [rad/s]
#define OBST_HIT_DIST 0.2   // [m] 

// direction macros
#define FWD true
#define BCK false
#define CW true
#define CCW false

// global robot state variables
extern float rob_yaw;
extern float rob_pos_x;
extern float rob_pos_y;
extern float goal_pos_x;
extern float goal_pos_y;
extern bool bumper_hit;
extern uint8_t bumper[NUM_BUMPER];

class Navigator
{
 private:
  float angular_vel;  // <= M_PI/6 [rad/s]
  float linear_vel;   // <= 0.25 [m/s] 
  
  // robot velocity publisher
  ros::Publisher vel_pub;
  geometry_msgs::Twist rob_vel;
  void publish_move();
  void move_straight(float dist, float linear_speed, bool forward);
  
 public:
  // TODO: add bumper and obstacle avoidance to move_straight() & rotate() 
  // (as a part of nav public functions)

  // commands
  void stop();
  void move_to(float goal_x, float goal_y);
  void move_right(float dist, float linear_speed, float angular_speed);
  void move_left(float dist, float linear_speed, float angular_speed);
  void rotate(float rad, float angular_speed, bool clockwise);
  void rotate_right(float angular_speed);
  void rotate_left(float angular_speed);
  void respond_to_bump();

   // constructor and destructor
   Navigator(ros::NodeHandle* nh);
   ~Navigator() {};
};

#endif  // INCLUDE_NAVIGATOR_H