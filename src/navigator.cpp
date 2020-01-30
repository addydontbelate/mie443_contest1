#include "navigator.h"

void Navigator::rotate_once()
{
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t seconds_elapsed = 0;
    
    while (seconds_elapsed < 10000/*2*M_PI/MAX_ANG_VEL*/)
    {
        angular_vel = MAX_ANG_VEL;
        rob_vel.angular.z = angular_vel;
        rob_vel.linear.x = linear_vel;
        vel_pub.publish(rob_vel);
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
    }

    stop();
}

void Navigator::Navigator(ros::Publisher vel_pub)
{   
    // Default values
    angular_vel = 0.0; 
    linear_vel = 0.0;
    vel_pub = vel_pub;
}

void Navigator::move_to_goal_point(float goal_x, float goal_y) 
{
    //  count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t seconds_elapsed = 0;

    // Loop until the robot is close to the desired x & y coordinates
    while (seconds_elapsed < 10000 && abs(goal_x-rob_pos_x) > 0.1 && abs(goal_y-rob_pos_y) > 0.1) 
    {
        // Calculate the x & y increments and required angle of rotation
        float inc_x = goal_x - rob_pos_x;
        float inc_y = goal_y - rob_pos_y;
        float goal_yaw = atan2(inc_y, inc_x);

        // Orient then move
        if (abs(goal_yaw-yaw) > 0.1) 
        {
            angular_vel = MAX_ANG_VEL;
            linear_vel = 0.0;
        }
        else 
        {
            angular_vel = 0;
            linear_vel = OPEN_ENV_VEL;
        }

        rob_vel.angular.z = angular_vel;
        rob_vel.linear.x = linear_vel;
        vel_pub.publish(rob_vel);

        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
    }
}
