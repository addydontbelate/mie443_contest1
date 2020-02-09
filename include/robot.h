#include <ros/ros.h>
#include <ros/console.h>
#include <cmath>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include "info.h"

class Robot
{
 private:

   float angular_vel = 0.0;  // <= M_PI/6 [rad/s]
   float linear_vel = 0.0;   // <= 0.25 [m/s]
   int32_t view_angle = 10;  // +-10 deg from heading angle

   // States
   float yaw = 0.0;
   float pos[2] = {0.0, 0.0};
   bool bumper_hit = false;
   float front_laser_dist = std::numeric_limits<float>::infinity();
   float left_laser_dist = std::numeric_limits<float>::infinity();
   float right_laser_dist = std::numeric_limits<float>::infinity();
   uint8_t bumper[3] = {
     kobuki_msgs::BumperEvent::RELEASED, 
     kobuki_msgs::BumperEvent::RELEASED, 
     kobuki_msgs::BumperEvent::RELEASED
     };
  
   // robot velocity publisher
   ros::Publisher vel_pub;
   geometry_msgs::Twist rob_vel;
   ros::Rate loop_rate(10);
   
   void publish_move();
   void bumper_callback();
   void pos_callback();
   void laser_callback();

 public:

   void init(ros::NodeHandle* nh);
   void stop();
   void move_straight(float dist, float linear_speed, bool forward);
   void move_right(float dist, float linear_speed);
   void move_left(float dist, float linear_speed);
  //  void move_to(float goal_x, float goal_y);
   void rotate(float rad, bool clockwise);
   void rotate_right();
   void rotate_left();

   // destructor
   Robot();
   ~Robot() {};
};