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
        else if (front_laser_dist < OBST_DIST_THRESH || right_laser_dist < OBST_DIST_THRESH || 
            left_laser_dist < OBST_DIST_THRESH)
        {
            // reactive navigation
            ROS_INFO("[NAV] Too close to the walls, moving away!");
            if (reactive_nav_enabled)
                respond_to_obst();
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
        ROS_INFO("[NAV] Required %d replans so far", num_tries);
        num_tries++;
    }

    // if still not reached goal and exceeded replan limit: initiate bug 2 navigation algorithm
    if (!GOAL_IN_REACH(goal_x, goal_y) && num_tries >= NUM_REPLANS)
        bug_nav(goal_x, goal_y);
    
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

    // limit both loops to run at 10Hz for stable behavior
    while (!GOAL_IN_REACH(goal_x, goal_y) && ros::ok() && seconds_elapsed < TIME_LIMIT)
    {
        ROS_INFO("[BUG_NAV] Started bug 2 algorithm!");

        // orient to goal
        float m_angle = orient_to(goal_x, goal_y);

        // move towards the goal; stop at the first obstacle
        float dist = sqrt(pow((rob_pos_x - goal_x), 2) + pow((rob_pos_y - goal_y), 2));
        move_straight(dist, FREE_ENV_VEL, FWD, DISABLE_REACTIVE_NAV);

        // update obstacle encounter position
        obst_pos_x = rob_pos_x;
        obst_pos_y = rob_pos_y;

        // turn to the right
        rotate(BUG_STEP/2, MAX_ANG_VEL, CW); // rotate right [default bias]

        // follow the obstacle
        while(!leave_obst(m_angle, goal_x, goal_y) && ros::ok() && seconds_elapsed < TIME_LIMIT)
        {
            ros::spinOnce();
            follow_obst();
            
            loop_rate.sleep();
        }

        loop_rate.sleep();
    }
}

void Navigator::follow_obst()
{
    // TODO: maybe set a tiny fwd velocity while all of this is happening?
    // TODO: have an immediate response if the robot is too close to walls;
    // go away then
    if (front_laser_dist < OBST_DIST_THRESH)
    {
        rotate(BUG_STEP/3, MAX_ANG_VEL, CW); // rotate right
        // nudge();
    }
    else if (fabs(left_laser_dist - OBST_DIST_THRESH) < BUG_TOL)
    {   
        nudge_fwd(BUG_STEP);  
    }
    else if (left_laser_dist > OBST_DIST_THRESH + BUG_TOL)
    {    
        rotate(BUG_STEP/3, MAX_ANG_VEL, CCW); // rotate left
        nudge_fwd(BUG_STEP/3); // TODO: this could be the bug in the bug!!!
    }
    else
    {
        rotate(BUG_STEP/3, MAX_ANG_VEL, CW); // rotate right
        nudge_fwd(BUG_STEP/3);
    }
}

void Navigator::nudge_fwd(float dist)
{
    // move straight by BUG_STEP
    float initial_pos_x = rob_pos_x;
    float initial_pos_y = rob_pos_y;

    linear_vel = OBST_DET_VEL;        
    angular_vel = 0.0;

    float dist_moved = 0.0;
    ros::Rate loop_rate(10);

    while (dist_moved < dist && ros::ok())
    {
        publish_move();
        ros::spinOnce();

        // update global position extremes
        update_global_extremes();

        loop_rate.sleep();
        
        dist_moved = sqrt(pow((rob_pos_x - initial_pos_x), 2) +
            pow((rob_pos_y - initial_pos_y), 2));
    }

    stop();
}

// TODO: what if the bumper is hit? recall move_to? -- not a good idea. check for the dist...
// TODO: time limit per call.
// TODO: the move straight function stops when either of the laser ranges go down below thresh, this will
// make the robot follow random obstacles! -- but at the end (should still get to the goal)?
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
            return true;

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