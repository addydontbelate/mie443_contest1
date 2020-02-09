#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include "info.h"
#include "robot.h"

// timer macros
#define TIME std::chrono::time_point<std::chrono::system_clock>
#define CLOCK std::chrono::system_clock
#define TIME_S std::chrono::duration_cast<std::chrono::seconds>
#define TIME_LIMIT 480 // seconds
#define time_check(time_elapsed) ((time_elapsed <= TIME_LIMIT) ? true : false) 

void setup_ros(Robot robot, string name)
{
    ros::init(argc, argv, name);
    ros::NodeHandle nh;
    
    // init subscribers
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &robot.bumper_callback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &robot.laser_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &robot.pos_callback);
}

int main(int argc, char **argv)
{
    // Init Robot + ROS
    Robot robot;
    setup_ros(robot, "my_little_pony")

    // Setup timer limit 
    TIME start = CLOCK::now();
    uint64_t seconds_elapsed = 0;

    // Start loop
    while(ros::ok() && time_check(seconds_elapsed)) 
    {
        // Update robot state vars
        ros::spinOnce();
        ROS_INFO("[MAIN] Position: (%f, %f) || Yaw: %f deg || Laser Distance: (Front: %f; Right: %f; Left: %f)", 
            rob_pos_x, rob_pos_y, RAD2DEG(rob_yaw), front_laser_dist, right_laser_dist, left_laser_dist);

        // Do magic


        // Update the timer
        seconds_elapsed = TIME_S(CLOCK::now()-start).count();
        robot.loop_rate.sleep();
    }

    // time up: stop
    ROS_INFO("[MAIN] Time up! Stopping.");
    robot.stop();
    return 0;
}