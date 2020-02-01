#include "wavefront_detector.h"

void Wavefront_Detector::pos_neighbors(int nbor[], int pos, int map_width)
{
    nbor[0] = pos - map_width - 1;
    nbor[1] = pos - map_width;
    nbor[2] = pos - map_width + 1;
    nbor[3] = pos - 1;
    nbor[4] = pos + 1;
    nbor[5] = pos + map_width - 1;
    nbor[6] = pos + map_width;
    nbor[7] = pos + map_width + 1;
}

std::vector<std::vector<int>> Wavefront_Detector::frontiers(const nav_msgs::OccupancyGrid& map, 
    int map_height, int map_width, int pose)
{
    std::vector<std::vector<int>> frontiers;
    int map_size = map_height * map_width;
    std::map<int, int> cell_state;
    std::queue<int> q_m;

    q_m.push(pose);
    cell_state[pose] = MAP_OPEN_LIST;
    
    int adj_nbors[NUM_NBORS];
    int v_nbors[NUM_NBORS];

    // exterior bfs
    while (!q_m.empty()) 
    {
        int cur_pos = q_m.front();
        q_m.pop();

        if (cell_state[cur_pos] == MAP_CLOSE_LIST)
            continue;

        if (is_frontier_point(map, cur_pos, map_size, map_width)) 
        {
            std::queue<int> q_f;
            std::vector<int> new_frontier;
            
            q_f.push(cur_pos);
            cell_state[cur_pos] = FRONTIER_OPEN_LIST;
            
            //  interior bfs
            while (!q_f.empty()) 
            {
                int n_cell = q_f.front();
                q_f.pop();

                if (cell_state[n_cell] == MAP_CLOSE_LIST || cell_state[n_cell] == FRONTIER_CLOSE_LIST)
                    continue;

                if (is_frontier_point(map, n_cell, map_size, map_width)) 
                {
                    new_frontier.push_back(n_cell);
                    pos_neighbors(adj_nbors, cur_pos, map_width);

                    for (int i = 0; i < NUM_NBORS; i++)
                        if (adj_nbors[i] < map_size && adj_nbors[i] >= 0) 
                            if (cell_state[adj_nbors[i]] != FRONTIER_OPEN_LIST &&
                                cell_state[adj_nbors[i]] != FRONTIER_CLOSE_LIST &&
                                cell_state[adj_nbors[i]] != MAP_CLOSE_LIST)
                                if (map.data[adj_nbors[i]] != OCCUPIED)
                                {
                                    q_f.push(adj_nbors[i]);
                                    cell_state[adj_nbors[i]] = FRONTIER_OPEN_LIST;
                                }
                }

                cell_state[n_cell] = FRONTIER_CLOSE_LIST;
            }

            if (new_frontier.size() > MIN_FRONTIER_SIZE)
            {
                frontiers.push_back(new_frontier);
                ROS_INFO("Added frontier of size %d", static_cast<int>(new_frontier.size()));
            }

            for (int i = 0; i < new_frontier.size(); i++) 
                cell_state[new_frontier[i]] = MAP_CLOSE_LIST;
        }

        pos_neighbors(adj_nbors, cur_pos, map_width);
        
        for (int i = 0; i < NUM_NBORS; ++i)
            if (adj_nbors[i] < map_size && adj_nbors[i] >= 0) 
                if (cell_state[adj_nbors[i]] != MAP_OPEN_LIST &&  
                    cell_state[adj_nbors[i]] != MAP_CLOSE_LIST) 
                {
                    pos_neighbors(v_nbors, adj_nbors[i], map_width);
                    bool map_open_neighbor = false;

                    for (int j = 0; j < NUM_NBORS; j++)
                        if (v_nbors[j] < map_size && v_nbors[j] >= 0)
                            if (map.data[v_nbors[j]] < OCC_THRESHOLD && map.data[v_nbors[j]] >= 0) 
                            {
                                map_open_neighbor = true;
                                break;
                            }

                    if (map_open_neighbor) 
                    {
                        q_m.push(adj_nbors[i]);
                        cell_state[adj_nbors[i]] = MAP_OPEN_LIST;
                    }
                }

        cell_state[cur_pos] = MAP_CLOSE_LIST;
    }

    return frontiers;
}

bool Wavefront_Detector::is_frontier_point(const nav_msgs::OccupancyGrid& map, 
    int pos, int map_size, int map_width)
{
    // point must not be in known region
    if (map.data[pos] != UNKNOWN) 
        return false;
    
    int nbors[NUM_NBORS];
    pos_neighbors(nbors, pos, map_width);
    
    for (int i = 0; i < NUM_NBORS; i++)
    {
        if (nbors[i] >= 0 && nbors[i] < map_size) 
        {
            // no neighbours should be occupied		
            if(map.data[nbors[i]] > OCC_THRESHOLD)
                return false;

            // frontier point if one neighbour is unoccupied (and known)
            if (map.data[nbors[i]] == UNOCCUPIED)
                return true;
        }
    }

    return false;
}

int Wavefront_Detector::nearest_frontier_idx(const std::vector<geometry_msgs::Point>& frontier_pos)
{
    tf::TransformListener map_listener;
    tf::StampedTransform map_transform;
    map_listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
    map_listener.lookupTransform("/map", "/base_link", ros::Time(0), map_transform);
    float cur_pos_x = map_transform.getOrigin().x();
    float cur_pos_y = map_transform.getOrigin().y();
    
    int idx = 0;
    float closest_frontier_dist = std::numeric_limits<float>::infinity();
    
    for (int i = 0; i < frontier_pos.size(); i++) 
    {
        float dist = points_dist(frontier_pos[i].x, cur_pos_x, 
            frontier_pos[i].y, cur_pos_y);

        if (dist > 0.7 && dist <= closest_frontier_dist) 
        {
            closest_frontier_dist = dist;
            idx = i;
        }
    }
    
    return idx;
}

float Wavefront_Detector::points_dist(float x_1, float x_2, float y_1, float y_2)
{
    return sqrt(pow((x_1 - x_2), 2.0) + pow((y_1 - y_2), 2.0));
}

int Wavefront_Detector::frontier_median(std::vector<int> frontier)
{
    std::sort(frontier.begin(), frontier.end());
    return frontier[frontier.size()/2];
}