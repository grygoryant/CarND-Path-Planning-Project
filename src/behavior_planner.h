//
// Created by grigorii on 9/8/19.
//
#ifndef PATH_PLANNING_BEHAVIOR_PLANNER_H
#define PATH_PLANNING_BEHAVIOR_PLANNER_H

#include <string>
#include <vector>
#include <map>

enum action
{
    KL = 0,
    LCL,
    LCR,
    PLCL,
    PLCR
};

using sensor_fusion_data_t = std::vector<std::vector<double>>;
using vehicle_state_t = std::vector<double>;
using trajectory_t = std::tuple<std::vector<double>, std::vector<double>>;

const std::map<action, std::vector<action>> action_adjacency =
        {
                {action::KL,   {action::KL, action::PLCL, action::PLCR}},
                {action::LCL,  {action::KL, action::LCL}},
                {action::LCR,  {action::KL, action::LCR}},
                {action::PLCL, {action::KL, action::LCL,  action::PLCL}},
                {action::PLCR, {action::KL, action::LCR,  action::PLCR}}
        };

class behavior_planner
{
    action m_curr_action {action::KL};
    std::vector<double> m_map_waypoints_x;
    std::vector<double> m_map_waypoints_y;
    std::vector<double> m_map_waypoints_s;
    std::vector<double> m_map_waypoints_dx;
    std::vector<double> m_map_waypoints_dy;
    int m_lane;
    double m_ref_vel;

    trajectory_t generate_trajectory(action dst_action,
            const sensor_fusion_data_t &sf_data,
            const trajectory_t &prev_path,
            const vehicle_state_t &curr_state,
            const std::pair<double, double> &prev_path_end_frenet);
public:
    behavior_planner() = default;
    behavior_planner(const std::vector<double>& map_wp_x,
                     const std::vector<double>& map_wp_y,
                     const std::vector<double>& map_wp_s,
                     int start_lane = 1,
                     double ref_vel = 0);

    trajectory_t get_trajectory(const sensor_fusion_data_t &sf_data,
                                const vehicle_state_t &curr_state,
                                const trajectory_t &prev_path,
                                const std::pair<double, double> &prev_path_end_frenet);
};


#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
