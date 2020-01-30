#include "navigator.h"

void Navigator::move_to_goal_point(float goal_x, float goal_y) 
{
    //  count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t seconds_elapsed = 0;

    // Loop until the robot is close to the desired x & y coordinates
    while (seconds_elapsed < 10 && abs(goal_x-rob_pos_x) > 0.1 && abs(goal_y-rob_pos_y) > 0.1) 
    {
        // Calculate the x & y increments and required angle of rotation
        float inc_x = goal_x - rob_pos_x;
        float inc_y = goal_y - rob_pos_y;
        float goal_yaw = atan2(inc_y, inc_x);

        // Orient then move
        if (abs(goal_yaw-yaw) > 0.1) 
        {
            angular_vel = MAX_ANG_VEL;
            linear_vel = 0;
        }
        else 
        {
            angular_vel = 0;
            linear_vel = OPEN_ENV_VEL;
        }

        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
    }
}
