#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include "wavefront_detector.h"


// defintiions and global constants
#define N_BUMPER (3) // LEFT, CENTER, RIGHT
#define RAD2DEG(rad) ((rad) * 180./M_PI)
#define DEG2RAD(deg) ((deg) * M_PI/180.)


// global variables
float pos_x = 0.0;
float pos_y = 0.0;
float yaw = 0.0;
float min_laser_dist = std::numeric_limits<float>::infinity(); 
int32_t n_lasers = 0;
int32_t desired_n_lasers = 0; 
int32_t view_angle = 5; // +-5 deg from heading angle
bool bumper_hit = false; // recovery mode flag
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, 
                    kobuki_msgs::BumperEvent::RELEASED, 
                    kobuki_msgs::BumperEvent::RELEASED};


// callback functions
void bumper_callback (const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper] = msg->state;
    
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
        ROS_INFO("%s bumper hit!", (msg->bumper == kobuki_msgs::BumperEvent::LEFT) ? "LEFT" : 
            (msg->bumper == kobuki_msgs::BumperEvent::CENTER) ? "CENTER" : "RIGHT" );
        bumper_hit = true; // set flag
    }
    else
        bumper_hit = false; // reset flag
}

void laser_callback (const sensor_msgs::LaserScan::ConstPtr& msg)
{
	min_laser_dist = std::numeric_limits<float>::infinity(); 
    n_lasers = (msg->angle_max - msg->angle_min)/msg->angle_increment; 
    desired_n_lasers = DEG2RAD(view_angle)/msg->angle_increment;

    if (DEG2RAD(view_angle) < msg->angle_max && -DEG2RAD(view_angle) > msg->angle_min) 
    {
        // find min_laser_dist over view_angle
        for (uint32_t laser_idx = (n_lasers/2 - desired_n_lasers); laser_idx < (n_lasers/2 + desired_n_lasers); ++laser_idx)
            min_laser_dist = std::min(min_laser_dist, msg->ranges[laser_idx]);
    } 
    else 
    {
        // use full range if view angle > angle range
        for (uint32_t laser_idx = 0; laser_idx < n_lasers; ++laser_idx) 
            min_laser_dist = std::min(min_laser_dist, msg->ranges[laser_idx]);
    }
}

void odom_callback (const nav_msgs::Odometry::ConstPtr& msg)
{
    pos_x = msg->pose.pose.position.x; 
    pos_y = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

void map_callback (const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    float resolution = map->info.resolution;
    float map_x = map->info.origin.position.x/resolution;
    float map_y = map->info.origin.position.y/resolution;
    // float x = 0. - map_x;
    // float y = 0. - map_y;

    // add (occupancy grid) map data
    // for (int i = 0; i < map->info.height; i++) 
    //     for (int j = 0; j < map->info.width; j++)
    //         map_2d[i][j] = (int) map->data[j + i * map.info.width];
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "addydontbelate_explorer");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumper_callback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laser_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &odom_callback);
    ros::Subscriber map_sub = nh.subscribe("map", 1, &map_callback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t seconds_elapsed = 0;

    float angular_vel = 0.0; // <= M_PI/6 [rad/s]
    float linear_vel = 0.0; // <= 0.25 [m/s]

    while(ros::ok() && seconds_elapsed <= 480) {
        ros::spinOnce();
        ROS_INFO("Position: (%f, %f); Orientation: %f deg; Min Laser Dist: %f;", 
            pos_x, pos_y, RAD2DEG(yaw), min_laser_dist);

        if (bumper_hit)
        {
            ; // initiate recovery mode.
        }

        // publish next move
        vel.angular.z = angular_vel;
        vel.linear.x = linear_vel;
        vel_pub.publish(vel);

        // update the timer.
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}