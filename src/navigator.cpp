#include "navigator.h"

void Navigator::init(ros::NodeHandle* nh)
{
    rob_vel.angular.z = angular_vel;
    rob_vel.linear.x = linear_vel;
    
    // init velocity publisher
    vel_pub = nh->advertise<geometry_msgs::Twist> ("cmd_vel_mux/input/teleop", 1);
}

Navigator::Navigator()
{
    // init velocities
    angular_vel = 0.0;
    linear_vel = 0.0;
    num_obst_response = OBST_RESPONSE_LIM;

    // controller for bug2
    reset_pid();
}

void Navigator::rotate(float rad, float angular_speed, bool clockwise)
{
    // ROS_INFO("[NAV] Currently at (%f, %f) @ %f deg;", rob_pos_x, rob_pos_y, RAD2DEG(rob_yaw));
    float initial_yaw = rob_yaw;

    if (clockwise)
        angular_vel = -fabs(angular_speed);
    else
        angular_vel = fabs(angular_speed);

    linear_vel = 0.0;

	float angle_turned = 0.0;
    ros::Rate loop_rate(10);
	
    if (rad < M_PI)
        while (angle_turned < rad && ros::ok() && seconds_elapsed < TIME_LIMIT)
        {
            initial_yaw = rob_yaw;
            ros::spinOnce();
            publish_move();
            loop_rate.sleep();
            
            angle_turned += fabs(rob_yaw - initial_yaw);
        }
    else
    {
        while (angle_turned < M_PI && ros::ok() && seconds_elapsed < TIME_LIMIT)
        {
            initial_yaw = rob_yaw;
            ros::spinOnce();
            publish_move();
            loop_rate.sleep();
            
            angle_turned += fabs(rob_yaw - initial_yaw);
        }
        rotate(rad-M_PI, angular_speed, clockwise);
    }
    
    stop();
    // ROS_INFO("[NAV] Rotated to (%f, %f) @ %f deg;", rob_pos_x, rob_pos_y, RAD2DEG(rob_yaw));
}

void Navigator::move_straight(float dist, float linear_speed, bool forward, bool reactive_nav_enabled)
{
	// initial pose before moving
	float initial_pos_x = rob_pos_x;
    float initial_pos_y = rob_pos_y;

	if (forward)
		linear_vel = fabs(linear_speed); // positive for forward
	else
		linear_vel = -fabs(linear_speed); // nagtive for backwards
	
    angular_vel = 0.0;

	float dist_moved = 0.0;
	ros::Rate loop_rate(10);

	while (dist_moved < dist && ros::ok() && seconds_elapsed < TIME_LIMIT)
    {
        publish_move();
		ros::spinOnce();

        // update global position extremes
        update_global_extremes();

        if (bumper_hit)
        {   
            ROS_INFO("[BUMP_HIT] Detected hit while moving straight!");
            bumper_hit = false; // reset flag
            respond_to_bump();
            return; // recalculate move
        }
        else if (front_laser_dist < OBST_DIST_THRESH + BUG_TOL || 
                right_laser_dist < OBST_DIST_THRESH + BUG_TOL || 
                left_laser_dist < OBST_DIST_THRESH + BUG_TOL)
        {
            // reactive navigation
            if (reactive_nav_enabled)
            {
                ROS_INFO("[NAV] Too close to the walls, moving away!");
                respond_to_obst();
            }
            else
                stop();
            return;
        }

		loop_rate.sleep();
		
        dist_moved = sqrt(pow((rob_pos_x - initial_pos_x), 2) +
			pow((rob_pos_y - initial_pos_y), 2));
	}

	stop();
}

void Navigator::move_to(float goal_x, float goal_y) 
{
    ROS_INFO("[NAV] Currently at (%f, %f); Moving to (%f, %f);", rob_pos_x, rob_pos_y, goal_x, goal_y);
    uint8_t num_tries = 0;
    float initial_pos_x = rob_pos_x;
    float initial_pos_y = rob_pos_y;

    while (!GOAL_IN_REACH(goal_x, goal_y) && num_tries < NUM_REPLANS && seconds_elapsed < TIME_LIMIT) 
    {
        num_obst_response = OBST_RESPONSE_LIM;

        // orient towards goal
        orient_to(goal_x, goal_y);

        // move straight to goal
        float dist = sqrt(pow((rob_pos_x - goal_x), 2) + pow((rob_pos_y - goal_y), 2));
        move_straight(dist, FREE_ENV_VEL, FWD);

        // increment move
        num_tries++;
        ROS_INFO("[NAV] Required %d replans so far", num_tries);
    }

    // reset num_tries for bug_nav
    num_tries = 0;

    // if still not reached goal and exceeded replan limit: initiate bug 2 navigation algorithm
    while (!GOAL_IN_REACH(goal_x, goal_y) && num_tries < NUM_REPLANS && seconds_elapsed < TIME_LIMIT)
    {
        bug_nav(goal_x, goal_y);
        num_tries++;
        ROS_INFO("[NAV] Required %d replans so far", num_tries + NUM_REPLANS);
    }

    ROS_INFO("[NAV] Moved to (%f, %f);", rob_pos_x, rob_pos_y);
}

void Navigator::move_right(float dist, float linear_speed, float angular_speed)
{
    // ROS_INFO("[NAV] Currently at (%f, %f);", rob_pos_x, rob_pos_y);
    rotate_right(angular_speed);
    move_straight(dist, linear_speed, FWD);
    // ROS_INFO("[NAV] Moved to (%f, %f);", rob_pos_x, rob_pos_y);
}

void Navigator::move_left(float dist, float linear_speed, float angular_speed)
{
    // ROS_INFO("[NAV] Currently at (%f, %f);", rob_pos_x, rob_pos_y);
    rotate_left(angular_speed);
    move_straight(dist, linear_speed, FWD);
    // ROS_INFO("[NAV] Moved to (%f, %f);", rob_pos_x, rob_pos_y);
}

void Navigator::rotate_right(float angular_speed)
{
    rotate(DEG2RAD(90), angular_speed, CW);
}

void Navigator::rotate_left(float angular_speed)
{
    rotate(DEG2RAD(90), angular_speed, CCW);
}

void Navigator::respond_to_bump()
{
    // decrement available responses
    (num_obst_response > 0) ? (num_obst_response--) : (num_obst_response = 0);

    // get number of hits
    uint8_t num_hits = 0;
    for (uint8_t i = 0; i < NUM_BUMPER; ++i)
        if (bumper[i] == kobuki_msgs::BumperEvent::PRESSED)
            num_hits++;
    
    ROS_INFO("[BUMP_HIT] Detected %d hits!", num_hits);
    if (num_obst_response > 0)
    {
        if (num_hits == 1) // one hit only
        {
            // left: move right
            if (bumper[0] == kobuki_msgs::BumperEvent::PRESSED)
            {
                ROS_INFO("[BUMP_HIT] Left bumper hit. Moving right!");
                move_right(SF*OBST_DIST_THRESH, OBST_DET_VEL, MAX_ANG_VEL);
            }
            // center: move back
            else if (bumper[1] == kobuki_msgs::BumperEvent::PRESSED)
            {
                ROS_INFO("[BUMP_HIT] Center bumper hit. Moving back!");
                move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, BCK);
            }
            // right: move left
            else
            {
                ROS_INFO("[BUMP_HIT] Right bumper hit. Moving left!");
                move_left(SF*OBST_DIST_THRESH, OBST_DET_VEL, MAX_ANG_VEL);
            }
        }
        else if (num_hits == 2) // two hits
        {
            // left + center: move back towards right
            if (bumper[0] == kobuki_msgs::BumperEvent::PRESSED  &&
                bumper[1] == kobuki_msgs::BumperEvent::PRESSED)
            {
                ROS_INFO("[BUMP_HIT] Left + Center bumpers hit. Moving back towards right!");
                rotate(DEG2RAD(45), MAX_ANG_VEL, CCW);
                move_straight(SF*OBST_DIST_THRESH/2, OBST_DET_VEL, BCK);
                rotate(DEG2RAD(45), MAX_ANG_VEL, CCW);
                move_straight(SF*OBST_DIST_THRESH/2, OBST_DET_VEL, BCK);
            }
            // right + center: move back towards left
            else if (bumper[2] == kobuki_msgs::BumperEvent::PRESSED  &&
                    bumper[1] == kobuki_msgs::BumperEvent::PRESSED)
            {
                ROS_INFO("[BUMP_HIT] Right + Center bumpers hit. Moving back towards left!");
                rotate(DEG2RAD(45), MAX_ANG_VEL, CW);
                move_straight(SF*OBST_DIST_THRESH/2, OBST_DET_VEL, BCK);
                rotate(DEG2RAD(45), MAX_ANG_VEL, CW);
                move_straight(SF*OBST_DIST_THRESH/2, OBST_DET_VEL, BCK);
            }
            // right + left (unlikely): move back
            else
            {
                ROS_INFO("[BUMP_HIT] Left + Right bumpers hit. Moving back!");
                move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, BCK);
            }
        }
        else // all bumpers are hit: move back
        {
            ROS_INFO("[BUMP_HIT] All bumpers hit. Moving back!");
            move_straight(SF*OBST_DIST_THRESH, OBST_HIT_DIST, BCK);
        }
    }
}

void Navigator::respond_to_obst()
{
    // decrement available responses
    (num_obst_response > 0) ? (num_obst_response--) : (num_obst_response = 0);

    ROS_INFO("[NAV_OBST] Front Dist: %f; Right Dist: %f; Left Dist: %f;", front_laser_dist, right_laser_dist, left_laser_dist);

    if (num_obst_response > 0)
    {
        if (front_laser_dist > OBST_DIST_THRESH && left_laser_dist > OBST_DIST_THRESH && 
            right_laser_dist > OBST_DIST_THRESH)
        {
            // no obstacle (unlikely)
            // ROS_INFO("[NAV_OBST] No obstacles around!");
        }
        else if (front_laser_dist < OBST_DIST_THRESH && left_laser_dist < OBST_DIST_THRESH && 
            right_laser_dist < OBST_DIST_THRESH)
        {
            // obstacles on all sides: blocked
            // ROS_INFO("[NAV_OBST] Blocked, rotating and moving back!");
            rotate(DEG2RAD(180), MAX_ANG_VEL, CW);
            move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
        }
        else if (front_laser_dist < OBST_DIST_THRESH && left_laser_dist > OBST_DIST_THRESH && 
            right_laser_dist > OBST_DIST_THRESH)
        {
            // obstacle at the front only
            // ROS_INFO("[NAV_OBST] Obstacle in front, moving around!");
            
            // obstacle to right farther than left 
            if (left_laser_dist < right_laser_dist)
            {
                // rotate right relative to the distance from the obstacle
                rotate(DEG2RAD((45/(OBST_DIST_THRESH - OBST_HIT_DIST))*(OBST_DIST_THRESH - front_laser_dist) + 45),
                    MAX_ANG_VEL, CW);
                move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
            }
            else
            {
                // rotate left relative to the distance from the obstacle
                rotate(DEG2RAD((45/(OBST_DIST_THRESH - OBST_HIT_DIST))*(OBST_DIST_THRESH - front_laser_dist) + 45),
                    MAX_ANG_VEL, CCW);
                move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
            }
        }
        else if (front_laser_dist > OBST_DIST_THRESH && left_laser_dist > OBST_DIST_THRESH && 
            right_laser_dist < OBST_DIST_THRESH)
        {
            // obstacle on the right only
            // ROS_INFO("[NAV_OBST] Obstacle at the right, moving towards left!");
            
            // obstacle to front farther than left 
            if (left_laser_dist < front_laser_dist)
            {
                // rotate left relative to the distance from the obstacle
                rotate(DEG2RAD((15/(OBST_DIST_THRESH - OBST_HIT_DIST))*(OBST_DIST_THRESH - right_laser_dist) + 30),
                    MAX_ANG_VEL, CCW);
                move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
            }
            else
            {
                // rotate left relative to the distance from the obstacle
                rotate(DEG2RAD((45/(OBST_DIST_THRESH - OBST_HIT_DIST))*(OBST_DIST_THRESH - right_laser_dist) + 45),
                    MAX_ANG_VEL, CCW);
                move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
            }
        }
        else if (front_laser_dist > OBST_DIST_THRESH && left_laser_dist < OBST_DIST_THRESH && 
            right_laser_dist > OBST_DIST_THRESH)
        {
            // obstacle on the left only
            // ROS_INFO("[NAV_OBST] Obstacle at the left, moving towards right!");

            // obstacle to front farther than right 
            if (right_laser_dist < front_laser_dist)
            {
                // rotate right relative to the distance from the obstacle
                rotate(DEG2RAD((15/(OBST_DIST_THRESH - OBST_HIT_DIST))*(OBST_DIST_THRESH - left_laser_dist) + 30),
                    MAX_ANG_VEL, CW);
                move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
            }
            else
            {
                // rotate right relative to the distance from the obstacle
                rotate(DEG2RAD((45/(OBST_DIST_THRESH - OBST_HIT_DIST))*(OBST_DIST_THRESH - left_laser_dist) + 45),
                    MAX_ANG_VEL, CW);
                move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
            }
        }
        else if (front_laser_dist < OBST_DIST_THRESH && left_laser_dist > OBST_DIST_THRESH && 
            right_laser_dist < OBST_DIST_THRESH)
        {
            // obstacles on the front and right only
            // ROS_INFO("[NAV_OBST] Obstacles on front and right, moving towards left!");
            move_left(SF*OBST_DIST_THRESH, OBST_DET_VEL, MAX_ANG_VEL);
        }
        else if (front_laser_dist < OBST_DIST_THRESH && left_laser_dist < OBST_DIST_THRESH && 
            right_laser_dist > OBST_DIST_THRESH)
        {
            // obstacles on the front and left only
            // ROS_INFO("[NAV_OBST] Obstacles on front and left, moving towards right!");
            move_right(SF*OBST_DIST_THRESH, OBST_DET_VEL, MAX_ANG_VEL);
        }
        else if (front_laser_dist > OBST_DIST_THRESH && left_laser_dist < OBST_DIST_THRESH && 
            right_laser_dist < OBST_DIST_THRESH)
        {
            // obstacles on the right and left only
            // ROS_INFO("[NAV_OBST] Obstacles on front and left, moving towards right!");
            move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
        }
        else 
            stop(); // unknown state   
    }
}

void Navigator::bug_nav(float goal_x, float goal_y)
{
	ros::Rate loop_rate(10);
    ROS_INFO("[BUG_NAV] Started bug 2 algorithm!");
    reset_pid();

    // bug timer
    TIME bug_start = CLOCK::now();
    uint64_t bug_time = 0.0;

    while (!GOAL_IN_REACH(goal_x, goal_y) && bug_time < BUG_TIMER && seconds_elapsed < TIME_LIMIT)
    {
        // orient to goal
        float m_angle = orient_to(goal_x, goal_y);

        // move towards the goal; stop at the first obstacle
        float dist = sqrt(pow((rob_pos_x - goal_x), 2) + pow((rob_pos_y - goal_y), 2));
        move_straight(dist, FREE_ENV_VEL, FWD, DISABLE_REACTIVE_NAV);

        // update obstacle encounter position
        obst_pos_x = rob_pos_x;
        obst_pos_y = rob_pos_y;

        // turn to the right
        rotate_right(BUG_ANG_VEL); // rotate right [default bias]

        // follow the obstacle
        while(!leave_obst(m_angle, goal_x, goal_y) && ros::ok() && 
            seconds_elapsed < TIME_LIMIT && bug_time < BUG_TIMER)
        {
            if (bumper_hit)
            {
                bumper_hit = false; // reset flag
                num_obst_response = OBST_RESPONSE_LIM;
                respond_to_bump();
                stop();
                return;
            }

            ros::spinOnce();
            follow_obst();
            publish_move();
            loop_rate.sleep();
            bug_time = TIME_S(CLOCK::now()-bug_start).count();
        }

        stop();
    }
}

void Navigator::follow_obst()
{
    // get angular speed from controller
    float cntrl_ang_vel = -pid.calculate(OBST_DIST_THRESH, left_laser_dist); // negative for turning right

    // make sure that we are not within hit dist on front and right
    if (front_laser_dist < OBST_DIST_THRESH + BUG_TOL)
        rotate_right(BUG_ANG_VEL);
    else if (fabs(left_laser_dist - OBST_DIST_THRESH) < BUG_TOL)
    {
        // suppress extra rotation: stability
        angular_vel = 0;
        linear_vel = OBST_DET_VEL;
    }
    else if (right_laser_dist < OBST_DIST_THRESH - BUG_TOL)
    {
        ROS_INFO("[BUG_NAV] Too close on right, turning around!");
        angular_vel = fabs(cntrl_ang_vel); // turn left
        linear_vel = 0.0;
    }
    else
    {
        // turn left or right with controller anguler velocity
        angular_vel = cntrl_ang_vel;
        linear_vel = OBST_DET_VEL;
    }

    // get rid of the gazebo error
    if (std::isnan(angular_vel))
        angular_vel = -BUG_ANG_VEL;
}

bool Navigator::leave_obst(float m_angle, float goal_x, float goal_y)
{
    // if just encountered obstacle, return false
    if (rob_pos_x == obst_pos_x && rob_pos_y == obst_pos_y)
        return false;

    float curr_angle = atan2f(goal_y-rob_pos_y, goal_x-rob_pos_x);
    
    float prev_dist = sqrt(pow(obst_pos_y-goal_x, 2) + pow(obst_pos_y-goal_y, 2));
    float curr_dist = sqrt(pow(rob_pos_x-goal_x, 2) +  pow(rob_pos_y-goal_y, 2));

    // if not near the initial obstacle encounter coordinates and on the m_line, leave
    if (fabs(curr_angle - m_angle) < BUG_TOL && !GOAL_IN_REACH(obst_pos_x, obst_pos_y))
        if (curr_dist < prev_dist)
        {
            ROS_INFO("[BUG_NAV] Leaving obstacle!");
            return true;
        }
    return false;
}

float Navigator::orient_to(float goal_x, float goal_y)
{
    // rotate towards goal
    float m_angle = atan2f(goal_y - rob_pos_y, goal_x - rob_pos_x);

    if (goal_y > 0) // goal ccw
    {  
        if (rob_yaw > 0)
            (rob_yaw - m_angle > 0) ? rotate(rob_yaw - m_angle, MAX_ANG_VEL, CW) : 
                rotate(m_angle - rob_yaw, MAX_ANG_VEL, CCW);
        else
            rotate(m_angle + rob_yaw, MAX_ANG_VEL, CCW);
    }
    else // goal cw
    {
        if (rob_yaw < 0)
            (rob_yaw - m_angle < 0) ? rotate(fabs(rob_yaw - m_angle), MAX_ANG_VEL, CCW) : 
                rotate(fabs(m_angle - rob_yaw), MAX_ANG_VEL, CW);
        else
            rotate(m_angle + rob_yaw, MAX_ANG_VEL, CW);
    }

    return m_angle;
}

void Navigator::stop()
{
    rob_vel.angular.z = 0.0;
    rob_vel.linear.x = 0.0;
    publish_move();
}

void Navigator::publish_move() 
{
    rob_vel.angular.z = angular_vel;
    rob_vel.linear.x = linear_vel;
    vel_pub.publish(rob_vel);
}

void Navigator::update_global_extremes()
{
    if (rob_pos_x > max_pos_x) 
        max_pos_x = rob_pos_x;
    else if (rob_pos_x < min_pos_x) 
        min_pos_x = rob_pos_x;
    else if (rob_pos_y > max_pos_y) 
        max_pos_y = rob_pos_y;
    else if (rob_pos_y < min_pos_y) 
        min_pos_y = rob_pos_y;
}

void Navigator::reset_pid()
{
    // PID at 10Hz for bug2 navigation algorithm
    pid = PID(0.1, BUG_ANG_VEL, -BUG_ANG_VEL, 6.0, 1.5, 2.0);
}