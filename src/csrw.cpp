#include "csrw.h"

CSRW::CSRW(ros::NodeHandle* nh) 
{ 
    // default shallow copy constructor
    nav.init(nh);

    // initialize coordinates
    top_right_crnr = {0.0, 0.0};
    top_left_crnr = {0.0, 0.0};
    bot_right_crnr = {0.0, 0.0};
    bot_left_crnr = {0.0, 0.0};
    cntr = {0.0, 0.0};
    goal_pos_x = 0.0;
    goal_pos_y = 0.0;
    go_to_cntr_flag = false;
    goal_stage = TOP_BOUNDARY;
}

void CSRW::set_goal()
{
    // update the corner coordinates according to the stage
    // and set goal coordinates for the navigator
    if (goal_stage == TOP_BOUNDARY)
    {
        ROS_INFO("[CSRW] Heading to TOP_BOUNDARY");
        goal_pos_x = EXTRM_DIST;
        goal_pos_y = 0.0;
        goal_stage = TOP_RIGHT_CRNR;
    }
    else if (goal_stage == TOP_RIGHT_CRNR)
    {
        ROS_INFO("[CSRW] Heading to TOP_RIGHT_CORNER");
        top_left_crnr = {max_pos_x, max_pos_y}; // top left crnr estimate
        goal_pos_x = EXTRM_DIST;//top_left_crnr[0];
        goal_pos_y = -EXTRM_DIST;//top_left_crnr[1] - EXTRM_DIST; // head right
        goal_stage = TOP_LEFT_CRNR;
    }
    else if (goal_stage == TOP_LEFT_CRNR)
    {
        ROS_INFO("[CSRW] Heading to TOP_LEFT_CORNER");
        top_right_crnr = {max_pos_x, min_pos_y}; // top right crnr estimate
        goal_pos_x = EXTRM_DIST;//top_right_crnr[0];
        goal_pos_y = EXTRM_DIST;//top_right_crnr[1] + EXTRM_DIST; // head left (back)
        goal_stage = BOT_LEFT_CRNR;
    }
    else if(goal_stage == BOT_LEFT_CRNR)
    {
        ROS_INFO("[CSRW] Heading to BOTTOM_LEFT_CORNER");
        top_left_crnr = {max_pos_x, max_pos_y}; // re-estimate top left crnr
        goal_pos_x = -EXTRM_DIST;//top_left_crnr[0] - EXTRM_DIST; // head down
        goal_pos_y = EXTRM_DIST;//top_left_crnr[1];
        goal_stage = BOT_RIGHT_CRNR;
    }
    else if(goal_stage == BOT_RIGHT_CRNR)
    {
        ROS_INFO("[CSRW] Heading to BOTTOM_RIGHT_CORNER");
        bot_left_crnr = {min_pos_x, max_pos_y}; // bottom left crnr estimate
        goal_pos_x = -EXTRM_DIST;//bot_left_crnr[0];
        goal_pos_y = -EXTRM_DIST;//bot_left_crnr[1] - EXTRM_DIST; // head right
        goal_stage = RENAV_TOP_RIGHT_CRNR;
    }
    else if (goal_stage == RENAV_TOP_RIGHT_CRNR)
    {
        bot_right_crnr = {min_pos_x, min_pos_y}; // bottom right crnr estimate
        goal_pos_x = EXTRM_DIST;//bot_left_crnr[0] + EXTRM_DIST; // head up
        goal_pos_y = -EXTRM_DIST;//bot_left_crnr[1];
        goal_stage = CNTR;
    }
    else if(goal_stage == CNTR)
    {
        top_right_crnr = {max_pos_x, min_pos_y}; // re-estimate top right crnr

        // estimate map center
        cntr[0] = (top_right_crnr[0] + top_left_crnr[0] + bot_left_crnr[0] + bot_right_crnr[0])/4.0;
        cntr[1] = (top_right_crnr[1] + top_left_crnr[1] + bot_left_crnr[1] + bot_right_crnr[1])/4.0;
        
        // set goal coordinates
        goal_pos_x = cntr[0];
        goal_pos_y = cntr[1];

        // now ready to navigate to the center
        go_to_cntr_flag = true;
        goal_stage = RW;
    }
    else if (goal_stage == RW)
    {
        // if coming from recovery state, 
        // maintain current position
        goal_pos_x = rob_pos_x;
        goal_pos_y = rob_pos_y;
    }
}

void CSRW::do_rw()
{
    // gen rand nums
    int delta_x = 0, delta_y = 0;
    while (delta_x == 0 && delta_y == 0)
    {
        delta_x  = (int(rand()%3) - 1);
        delta_y  = (int(rand()%3) - 1);
    }
    
    // random walk
    nav.move_to(rob_pos_x + delta_x, rob_pos_y + delta_y);
}

void CSRW::respond_to_bump()
{
    nav.set_response_limit();
    nav.respond_to_bump();
}