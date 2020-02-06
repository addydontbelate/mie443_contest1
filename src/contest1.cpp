#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include "wavefront_detector.h"
#include "navigator.h"
#include "visualizer.h"

// timer macros
#define TIME std::chrono::time_point<std::chrono::system_clock>
#define CLOCK std::chrono::system_clock
#define TIME_S std::chrono::duration_cast<std::chrono::seconds>
#define TIME_LIMIT 480 // seconds
#define addydontbelate(time_elapsed) ((time_elapsed <= TIME_LIMIT) ? true : false) 

// fsm states
#define _INIT_ 0
#define _RECOVERY_ 1
#define _GET_NEW_FRONTIER_ 2
#define _NAV_TO_FRONTIER_ 3

// global robot state variables
int rob_state = _INIT_;
float rob_yaw = 0.0;
float rob_pos_x = 0.0;
float rob_pos_y = 0.0;
float goal_pos_x = rob_pos_x;
float goal_pos_y = rob_pos_y;
float front_laser_dist = std::numeric_limits<float>::infinity();
float left_laser_dist = std::numeric_limits<float>::infinity();
float right_laser_dist = std::numeric_limits<float>::infinity();
int32_t n_lasers = 0;
int32_t desired_n_lasers = 0;
int32_t view_angle = 10;        // +-10 deg from heading angle
bool detect_frontier = false;   // wfd enable flag
bool bumper_hit = false;        // recovery mode flag
uint8_t bumper[NUM_BUMPER] = {kobuki_msgs::BumperEvent::RELEASED, 
    kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

// global object for custom visualization
Visualizer viz;

/**
 * ROS callback to record bumper hit.
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
 * ROS callback to set minimum distance from laser over view angle.
 */
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	front_laser_dist = std::numeric_limits<float>::infinity();
    left_laser_dist = std::numeric_limits<float>::infinity();
    right_laser_dist = std::numeric_limits<float>::infinity();
    n_lasers = (msg->angle_max - msg->angle_min)/msg->angle_increment; 
    desired_n_lasers = DEG2RAD(view_angle)/msg->angle_increment;

    // find front_laser_dist over view_angle
    for (uint32_t laser_idx = (n_lasers/2 - desired_n_lasers); laser_idx < (n_lasers/2 + desired_n_lasers); ++laser_idx)
        front_laser_dist = std::min(front_laser_dist, msg->ranges[laser_idx]);
    
    // find left_laser_dist
    for (uint32_t laser_idx = 0; laser_idx < (n_lasers/2 - desired_n_lasers) - 1; ++laser_idx)
        left_laser_dist = std::min(front_laser_dist, msg->ranges[laser_idx]);
    
    // find right_laser_dist
    for (uint32_t laser_idx = (n_lasers/2 + desired_n_lasers) + 1; laser_idx < msg->ranges.size(); ++laser_idx)
        right_laser_dist = std::min(front_laser_dist, msg->ranges[laser_idx]);
}

/**
 * ROS callback to update robot pose.
 */
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    rob_pos_x = msg->pose.pose.position.x; 
    rob_pos_y = msg->pose.pose.position.y;
    rob_yaw = tf::getYaw(msg->pose.pose.orientation);
}

/**
 * ROS callback to update map and get desirable frontier's median.
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
        // init detector object
        Wavefront_Detector wfd;

        // get frontiers
        std::vector<std::vector<int>> frontiers = wfd.frontiers(map, map.info.height, 
            map.info.width, x + (y * map.info.width));
        ROS_INFO("[WFD] Found %d frontiers", static_cast<int>(frontiers.size()));

        // visualize frontiers
        viz.visualize_frontier();

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
        ROS_INFO("[WFD] Nearest frontier index is: %d", nf_idx);

        //  update goal position
        if (!frontier_pos.empty())
        {
            goal_pos_x = frontier_pos[nf_idx].x;
            goal_pos_y = frontier_pos[nf_idx].y;
            ROS_INFO("[WFD] Goal position set to: (%f, %f)", goal_pos_x, goal_pos_y);
        }
        else
        {
            goal_pos_x = rob_pos_x;
            goal_pos_y = rob_pos_y;
            ROS_INFO("[WFD] Frontier not found! Goal set to: (%f, %f)", goal_pos_x, goal_pos_y);
        }

        detect_frontier = false; // set to true when needing a new frontier
        rob_state = _NAV_TO_FRONTIER_;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "addydontbelate_explorer");
    ros::NodeHandle nh;
    
    // init subscribers
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumper_callback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laser_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &odom_callback);
    ros::Subscriber map_sub = nh.subscribe("map", 1, &map_callback);

    // init navigator object and visualizer topic
    Navigator nav(&nh);
    viz.init(&nh);

    // init loop rate
    ros::Rate loop_rate(10);

    // contest count down timer
    TIME start = CLOCK::now();
    uint64_t seconds_elapsed = 0;
    srand(time(0)); // random walk

    // robot loop
    while(ros::ok() && addydontbelate(seconds_elapsed)) 
    {
        // update robot state vars
        ros::spinOnce();
        ROS_INFO("[MAIN] Position: (%f, %f) || Yaw: %f deg || Laser Distance: (Front: %f; Right: %f; Left: %f)", 
            rob_pos_x, rob_pos_y, RAD2DEG(rob_yaw), front_laser_dist, right_laser_dist, left_laser_dist);

        // unexpected hit: move away from hit
        if (bumper_hit)
        {   
            bumper_hit = false; // reset flag
            nav.respond_to_bump();

            // initiate recovery mode
            rob_state = _RECOVERY_;
        }

        // robot fsm
        if (rob_state == _INIT_)
        {
            ROS_INFO("[MAIN] Robot in INIT state");
            nav.rotate(DEG2RAD(360), MAX_ANG_VEL, CW);
            rob_state = _GET_NEW_FRONTIER_;
            // detect_frontier = true;
        }
        else if (rob_state == _RECOVERY_)
        {
            ROS_INFO("[MAIN] Robot in RECOVERY state");
            rob_state = _INIT_;
        }
        else if (rob_state == _GET_NEW_FRONTIER_)
        {
            ROS_INFO("[MAIN] Robot in GET_NEW_FRONTIER state");
            rob_state = _NAV_TO_FRONTIER_;
        }
        else if (rob_state == _NAV_TO_FRONTIER_)
        {
            ROS_INFO("[MAIN] Robot in NAV_TO_FRONTIER state");

            // gen rand nums
            int delta_x = 0, delta_y = 0;
            while (delta_x == 0 && delta_y == 0)
            {
                delta_x  = (int(rand()%3) - 1);
                delta_y  = (int(rand()%3) - 1);
            }

            nav.move_to(rob_pos_x + delta_x, rob_pos_y + delta_y);

            // nav.move_to(goal_pos_x, goal_pos_y);
            rob_state = _INIT_; // repeat process
        }
        else // invalid state stored
        {
            ROS_INFO("[MAIN] Robot in INVALID state! Starting RECOVERY");
            rob_state = _RECOVERY_;
            nav.stop(); // stop robot movement
        }

        // update the timer
        seconds_elapsed = TIME_S(CLOCK::now()-start).count();
        loop_rate.sleep();
    }

    // time up: stop
    ROS_INFO("[MAIN] Time up! Stopping.");
    nav.stop();
    return 0;
}