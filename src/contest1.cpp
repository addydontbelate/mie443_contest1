#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include "wavefront_detector.h"
#include "navigator.h"
#include "visualizer.h"

// defintiions and global constants
#define NUM_BUMPER (3) // LEFT, CENTER, RIGHT
#define RAD2DEG(rad) ((rad) * 180./M_PI)
#define DEG2RAD(deg) ((deg) * M_PI/180.)
#define TIME_LIMIT 480 // seconds
#define addydontbelate(time_elapsed) ((time_elapsed <= TIME_LIMIT) ? true : false) 

// fsm states
#define _INIT_ 0
#define _RECOVERY_ 1
#define _GET_NEW_FRONTIER_ 2
#define _NAV_TO_FRONTIER_ 3

// global variables
int rob_state = _INIT_;
float rob_pos_x = 0.0;
float rob_pos_y = 0.0;
float goal_pos_x = rob_pos_x;
float goal_pos_y = rob_pos_y;
float yaw = 0.0;
float min_laser_dist = std::numeric_limits<float>::infinity(); 
int32_t n_lasers = 0;
int32_t desired_n_lasers = 0; 
int32_t view_angle = 5; // +-5 deg from heading angle
bool detect_frontier = false; // wfd enable flag
bool bumper_hit = false; // recovery mode flag
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, 
                    kobuki_msgs::BumperEvent::RELEASED, 
                    kobuki_msgs::BumperEvent::RELEASED};
geometry_msgs::Twist rob_vel;
ros::Publisher vel_pub;

// Objects

Wavefront_Detector wfd;
Visualizer viz;

// Callback functions

/**
 * ROS callback to globally record bumper hit.
 */

void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper] = msg->state;
    
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
        ROS_INFO("%s bumper hit!", (msg->bumper == kobuki_msgs::BumperEvent::LEFT) ? "LEFT" : 
            (msg->bumper == kobuki_msgs::BumperEvent::CENTER) ? "CENTER" : "RIGHT" );
        bumper_hit = true; // set flag
    }
}

/**
 * ROS callback to set global laser distance given readings.
 */

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
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

/**
 * ROS callback to set the position of the robot's global vairables.
 */

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    
    rob_pos_x = msg->pose.pose.position.x; 
    rob_pos_y = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

/**
 * ROS callback to globally update frontier detector
 */

void map_callback(const nav_msgs::OccupancyGrid& map)
{
    float resolution = map.info.resolution;
    float map_x = map.info.origin.position.x/resolution;
    float map_y = map.info.origin.position.y/resolution;
    float x = 0. - map_x;
    float y = 0. - map_y;

    if (detect_frontier)
    {
        // get frontiers
        std::vector<std::vector<int>> frontiers = wfd.frontiers(map, map.info.height, 
            map.info.width, x + (y * map.info.width));
        ROS_INFO("Found %d frontiers", static_cast<int>(frontiers.size()));

        // visualize frontiers
        // std::vector<std::vector<int>> map_2d(map->info.height, std::vector<int>(map->info.width, 0)); // print to file

        // get frontier medians
        std::vector<int> frontier_median;
        for (int i = 0; i < frontiers.size(); i++)
            frontier_median.push_back(wfd.frontier_median(frontiers[i]));

        // corresponding position of median on map
        std::vector<geometry_msgs::Point> frontier_pos(frontier_median.size());
        
        for (int i = 0; i < frontier_median.size(); i++)
        {
            frontier_pos[i].x = ((frontier_median[i] % map.info.width) + map_x)*resolution;
            frontier_pos[i].y = ((frontier_median[i] / map.info.width) + map_y)*resolution;
            frontier_pos[i].z = 0;
        }

        // get nearest frontier
        int nf_idx = wfd.nearest_frontier_idx(frontier_pos);
        ROS_INFO("Nearest frontier index is: %d", nf_idx);

        //  update goal position
        goal_pos_x = frontier_pos[nf_idx].x;
        goal_pos_y = frontier_pos[nf_idx].y;
        ROS_INFO("Goal position set to: (%f, %f)", goal_pos_x, goal_pos_y);

        detect_frontier = false; // set to true when needing a new frontier
        rob_state = _NAV_TO_FRONTIER_;
    }
}

/* Initialize ROS */

ros::Publisher setup_ros()
{
    ros::init (argc, argv, "auto_explorer");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumper_callback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laser_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &odom_callback);
    ros::Subscriber map_sub = nh.subscribe("map", 1, &map_callback);
<<<<<<< HEAD

    vel_pub = nh.advertise<geometry_msgs::Twist> ("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);
=======
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist> ("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel;

    return vel_pub;
}

int main(int argc, char **argv)
{   
    // Setup ROS + Navigator
    vel_pub = setup_ros();
    Navigator Nav(vel_pub);
>>>>>>> 7cb2e13da8d87148b79dcd5b27dce97c2a9ac33a

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t seconds_elapsed = 0;

    // Begin robot processing loop

    while(ros::ok() && addydontbelate(seconds_elapsed)) 
    {
        ros::spinOnce();
        ROS_INFO("Position: (%f, %f); Orientation: %f deg; Min Laser Dist: %f;", 
            rob_pos_x, rob_pos_y, RAD2DEG(yaw), min_laser_dist);

        // unexpected hit
        if (bumper_hit)
        {
            ; // initiate recovery mode.

            rob_state = _RECOVERY_;
            bumper_hit = false; // reset flag
        }

        // robot fsm
        if (rob_state == _INIT_)
        {
            ROS_INFO("Robot in INIT state");
<<<<<<< HEAD
            nav.rotate_once();
            rob_state = _GET_NEW_FRONTIER_;
            // detect_frontier = true;
=======
            ; 
            // rot 360;
            detect_frontier = true;

            Nav.move_to_goal_point(1,1);
>>>>>>> 7cb2e13da8d87148b79dcd5b27dce97c2a9ac33a
        }
        else if (rob_state == _RECOVERY_)
        {
            ROS_INFO("Robot in RECOVERY state");
            rob_state = _INIT_; // also make robot move away from obst
        }
        else if (rob_state == _GET_NEW_FRONTIER_)
        {
            ROS_INFO("Robot in GET_NEW_FRONTIER state");
            ; //
            rob_state = _NAV_TO_FRONTIER_;
        }
        else if (rob_state == _NAV_TO_FRONTIER_)
        {
            ROS_INFO("Robot in NAV_TO_FRONTIER state");
            // nav.move_to_goal_point(goal_pos_x, goal_pos_y); //
            nav.move_to_goal_point(1, 1); //
            return 0;
        }
        else // invalid state stored
        {
            ROS_INFO("Robot in INVALID state! Starting RECOVERY");
            rob_state = _RECOVERY_;
            Nav.stop(); // stop robot movement
        }

        // publish next move
<<<<<<< HEAD
        // rob_vel.angular.z = nav.get_angular_vel();
        // rob_vel.linear.x = nav.get_linear_vel();
        // vel_pub.publish(vel);
=======
        // move_time = move_time
        vel.angular.z = Nav.get_angular_vel();
        vel.linear.x = Nav.get_linear_vel();
        vel_pub.publish(vel);
>>>>>>> 7cb2e13da8d87148b79dcd5b27dce97c2a9ac33a

        // update the timer.
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}