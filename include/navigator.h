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

// timer macros
#define TIME std::chrono::time_point<std::chrono::system_clock>
#define CLOCK std::chrono::system_clock
#define TIME_S std::chrono::duration_cast<std::chrono::seconds>
#define TIME_LIMIT 480 // seconds
#define addydontbelate(time_elapsed) ((time_elapsed <= TIME_LIMIT) ? true : false) 

#define NUM_BUMPER 3 // LEFT, CENTER, RIGHT

// angle conversion macros
#define RAD2DEG(rad) ((rad) * 180./M_PI)
#define DEG2RAD(deg) ((deg) * M_PI/180.)

// velocity and movement limits
#define FREE_ENV_VEL 0.25   // [m/s]
#define OBST_DET_VEL 0.1    // [m/s]
#define MAX_ANG_VEL M_PI/6  // [rad/s]
#define OBST_HIT_DIST 0.2   // [m] 
#define OBST_DIST_THRESH 0.5// [m]
#define GOAL_REACH_DIST 0.15// [m]
#define SF 1.15             // num; safety factor
#define NUM_REPLANS 3       // num
#define OBST_RESPONSE_LIM 5 // num

// direction macros
#define FWD true
#define BCK false
#define CW true
#define CCW false

// reactive navigation macros
#define DISABLE_REACTIVE_NAV false
#define ENABLE_REACTIVE_NAV true

// bug 2 algorithm tolerance
#define BUG_TOL 0.1
#define BUG_STEP 0.25

// macro for goal within reach
#define GOAL_IN_REACH(goal_x, goal_y) (fabs(rob_pos_x - goal_x) < GOAL_REACH_DIST || fabs(rob_pos_y - goal_y) < GOAL_REACH_DIST)

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

class Navigator
{
 private:
   float angular_vel;  // <= M_PI/6 [rad/s]
   float linear_vel;   // <= 0.25 [m/s]
   float obst_pos_x;
   float obst_pos_y;
   uint64_t seconds_elapsed;
   TIME start;
   uint8_t num_obst_response; // OBST_RESPONSE_LIM

  
   // robot velocity publisher
   ros::Publisher vel_pub;
   geometry_msgs::Twist rob_vel;
   void publish_move();
   void move_straight(float dist, float linear_speed, bool forward, bool reactive_nav_enabled = ENABLE_REACTIVE_NAV);
   void move_right(float dist, float linear_speed, float angular_speed);
   void move_left(float dist, float linear_speed, float angular_speed);
   void respond_to_obst();
   void bug_nav(float goal_x, float goal_y);
   void update_global_extremes();
   float orient_to(float goal_x, float goal_y);
   void follow_obst();
   bool leave_obst(float m_angle, float goal_x, float goal_y);
   void update_time();

 public:
   // commands
   void init(ros::NodeHandle* nh);
   void stop();
   void move_to(float goal_x, float goal_y);
   void rotate(float rad, float angular_speed, bool clockwise);
   void rotate_right(float angular_speed);
   void rotate_left(float angular_speed);
   void respond_to_bump();  
   void set_response_limit() { num_obst_response = OBST_RESPONSE_LIM; }

   // destructor
   Navigator();
   ~Navigator() {};
};

#endif  // INCLUDE_NAVIGATOR_H