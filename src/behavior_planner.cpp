//
// Created by grigorii on 9/8/19.
//

#include "behavior_planner.h"

#include <algorithm>
#include <iostream>

#include "helpers.h"
#include "spline.h"

std::string to_string(action a)
{
    switch(a)
    {
        case(action::KL): return "action::KL";
        case(action::LCL): return "action::LCL";
        case(action::LCR): return "action::LCR";
        case(action::PLCL): return "action::PLCL";
        case(action::PLCR): return "action::PLCR";
        default: return "unknown action";
    }
}

trajectory_t behavior_planner::get_trajectory(const sensor_fusion_data_t &sf_data,
                                              const vehicle &curr_state,
                                              const trajectory_t &prev_path,
                                              const std::pair<double, double> &prev_path_end_frenet)
{
    const auto& next_states = action_adjacency.find(m_curr_action);
    if(next_states != std::end(action_adjacency))
    {
        std::vector<double> costs;
        std::vector<trajectory_t> final_trajectories;
        std::vector<action> final_states;
        for(const auto& state : next_states->second)
        {
            auto [dst_lane, trajectory] = generate_trajectory(state, sf_data,
                    prev_path, curr_state, prev_path_end_frenet);
            if (!std::get<0>(trajectory).empty())
            {
                auto cost = calculate_cost(sf_data,
                                           curr_state,
                                           trajectory, dst_lane);
                costs.push_back(cost);
                final_trajectories.emplace_back(trajectory);
                final_states.emplace_back(state);
            }
        }

        auto best_cost = std::min_element(begin(costs), end(costs));
        int best_idx = std::distance(begin(costs), best_cost);
        auto best_action = final_states[best_idx];
        if(best_action != m_curr_action)
        {
            std::cout << "State transition: " << to_string(m_curr_action) <<
                " -> " << to_string(best_action) << std::endl;
            m_curr_action = best_action;
        }

        return final_trajectories[best_idx];
    }
    return {{}, {}};
}

std::tuple<int, trajectory_t> behavior_planner::generate_trajectory(action dst_action,
                                                                    const sensor_fusion_data_t &sf_data,
                                                                    const trajectory_t &prev_path,
                                                                    const vehicle &curr_state,
                                                                    const std::pair<double, double> &prev_path_end_frenet)
{
    switch(dst_action)
    {
        case action::KL:
            return kl_trajectory(sf_data, prev_path, curr_state, prev_path_end_frenet);
        case action::LCL:
        case action::LCR:
            return lc_trajectory(dst_action, sf_data, prev_path, curr_state, prev_path_end_frenet);
        case action::PLCL:
        case action::PLCR:
            return plc_trajectory(dst_action, sf_data, prev_path, curr_state, prev_path_end_frenet);
        default: break;
    }

    return {};
}

behavior_planner::behavior_planner(const std::vector<double> &map_wp_x,
        const std::vector<double> &map_wp_y,
        const std::vector<double> &map_wp_s) :
            m_map_waypoints_x(map_wp_x),
            m_map_waypoints_y(map_wp_y),
            m_map_waypoints_s(map_wp_s)
{

}

std::tuple<int, trajectory_t> behavior_planner::kl_trajectory(const sensor_fusion_data_t &sf_data,
                                                              const trajectory_t &prev_path,
                                                              const vehicle &curr_state,
                                                              const std::pair<double, double> &prev_path_end_frenet)
{
    const auto& [previous_path_x, previous_path_y] = prev_path;
    double car_x = curr_state.x();
    double car_y = curr_state.y();
    double car_s = curr_state.s();
    double car_d = curr_state.d();
    double car_yaw = curr_state.yaw();
    double car_speed = curr_state.speed();
    auto end_path_s = prev_path_end_frenet.first;

    auto prev_size = previous_path_x.size();

    if(prev_size > 0)
    {
        car_s = end_path_s;
    }

    bool too_close = false;
    double target_vel = 0;
    auto curr_lane = curr_state.get_cur_lane();

    for(int i = 0; i < sf_data.size(); i++)
    {
        float d = sf_data[i][6];
        if(d < (2 + 4 * curr_lane + 2) && d > (2 + 4 * curr_lane - 2))
        {
            double vx = sf_data[i][3];
            double vy = sf_data[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sf_data[i][5];

            check_car_s += static_cast<double>(prev_size) * 0.02 * check_speed;

            if((check_car_s > car_s) && (check_car_s - car_s) < 25)
            {
                too_close = true;
                target_vel = check_speed;
                break;
            }
        }
    }

    if(too_close && m_ref_vel > target_vel)
    {
        m_ref_vel -= 0.224;
    }
    else if((too_close && m_ref_vel <= target_vel) ||
            m_ref_vel < 49.5)
    {
        m_ref_vel += 0.224;
    }

    return {curr_lane, get_trajectory_points(curr_lane, previous_path_x,
                                 previous_path_y, curr_state, m_ref_vel) };
}

std::tuple<int, trajectory_t>
behavior_planner::plc_trajectory(action dst_action,
        const sensor_fusion_data_t &sf_data,
        const trajectory_t &prev_path,
        const vehicle &curr_state,
        const std::pair<double, double> &prev_path_end_frenet)
{
    return {};
}

std::tuple<int, trajectory_t>
behavior_planner::lc_trajectory(action dst_action,
        const sensor_fusion_data_t &sf_data,
        const trajectory_t &prev_path,
        const vehicle &curr_state,
        const std::pair<double, double> &prev_path_end_frenet)
{
    const auto& dir = lane_direction.find(dst_action);
    if(dir != std::end(lane_direction))
    {
        int new_lane = curr_state.get_cur_lane() + dir->second;
        const auto& [previous_path_x, previous_path_y] = prev_path;

        return { new_lane, get_trajectory_points(new_lane, previous_path_x,
                                     previous_path_y, curr_state, m_ref_vel) };
    }
    return {};
}

trajectory_t behavior_planner::get_trajectory_points(int target_lane,
        const std::vector<double> &prev_path_x,
        const std::vector<double> &prev_path_y,
        const vehicle &curr_state,
        double ref_vel)
{
    auto ref_x = curr_state.x();
    auto ref_y = curr_state.y();
    auto ref_yaw = deg2rad(curr_state.yaw());
    std::vector<double> pts_x;
    std::vector<double> pts_y;

    if(prev_path_x.size() < 2)
    {
        auto prev_car_x = curr_state.x() - cos(curr_state.yaw());
        auto prev_car_y = curr_state.y() - sin(curr_state.yaw());

        pts_x.emplace_back(prev_car_x);
        pts_x.emplace_back(curr_state.x());

        pts_y.emplace_back(prev_car_y);
        pts_y.emplace_back(curr_state.y());
    }
    else
    {
        ref_x = prev_path_x[prev_path_x.size() - 1];
        ref_y = prev_path_y[prev_path_x.size() - 1];

        double ref_x_prev = prev_path_x[prev_path_x.size() - 2];
        double ref_y_prev = prev_path_y[prev_path_x.size() - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        pts_x.emplace_back(ref_x_prev);
        pts_x.emplace_back(ref_x);

        pts_y.emplace_back(ref_y_prev);
        pts_y.emplace_back(ref_y);
    }

    auto next_wp0 = getXY(curr_state.s() + 50, (2 + 4 * target_lane), m_map_waypoints_s,
                          m_map_waypoints_x, m_map_waypoints_y);
    auto next_wp1 = getXY(curr_state.s() + 60, (2 + 4 * target_lane), m_map_waypoints_s,
                          m_map_waypoints_x, m_map_waypoints_y);
    auto next_wp2 = getXY(curr_state.s() + 90, (2 + 4 * target_lane), m_map_waypoints_s,
                          m_map_waypoints_x, m_map_waypoints_y);

    pts_x.emplace_back(next_wp0[0]);
    pts_x.emplace_back(next_wp1[0]);
    pts_x.emplace_back(next_wp2[0]);

    pts_y.emplace_back(next_wp0[1]);
    pts_y.emplace_back(next_wp1[1]);
    pts_y.emplace_back(next_wp2[1]);

    for(int i = 0; i < pts_x.size(); i++)
    {
        auto shift_x = pts_x[i] - ref_x;
        auto shift_y = pts_y[i] - ref_y;

        pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    tk::spline spline;
    spline.set_points(pts_x, pts_y);

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    for(int i = 0; i < prev_path_x.size(); i++)
    {
        next_x_vals.emplace_back(prev_path_x[i]);
        next_y_vals.emplace_back(prev_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = spline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double x_add_on = 0;

    for(int i = 1; i <= 50 - prev_path_x.size(); i++)
    {
        double N = (target_dist / (0.02 * ref_vel / 2.24));
        double x_point = x_add_on + target_x / N;
        double y_point = spline(x_point);

        x_add_on = x_point;

        auto x_ref = x_point;
        auto y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.emplace_back(x_point);
        next_y_vals.emplace_back(y_point);
    }

    return {next_x_vals, next_y_vals};
}

double lane_speed(const sensor_fusion_data_t &sf_data,
        const vehicle& curr_state, int lane)
{
    for(const auto& v : sf_data)
    {
        double check_car_s = v[5];
        if((check_car_s > curr_state.s()) && (check_car_s - curr_state.s()) < 30)
        {
            double vx = v[3];
            double vy = v[4];

            double speed = sqrt(vx * vx + vy * vy);
            vehicle veh(v[1], v[2], v[5], v[6], 0, speed);
            if (veh.get_cur_lane() == lane)
            {
                return veh.speed();
            }
        }
    }
    // Found no vehicle in the lane
    return -1.0;
}

double inefficiency_cost(double target_speed,
                        const sensor_fusion_data_t &sf_data,
                        const vehicle &curr_state,
                        int curr_lane, int dst_lane) {
    double proposed_speed_intended = lane_speed(sf_data, curr_state, curr_lane);
    if (proposed_speed_intended < 0) {
        proposed_speed_intended = target_speed;
    }

    double proposed_speed_final = lane_speed(sf_data, curr_state, dst_lane);
    if (proposed_speed_final < 0) {
        proposed_speed_final = target_speed;
    }

    auto cost = (2.0*target_speed - proposed_speed_intended
                  - proposed_speed_final)/target_speed;

    return cost;
}

double lane_cost(int dst_lane)
{
    if(dst_lane < 0 || dst_lane > 2)
        return 1;
    return 0;
}

double collision_cost(const sensor_fusion_data_t &sf_data,
        const vehicle &curr_state,
        int curr_lane, int dst_lane)
{
    for(const auto& v : sf_data)
    {
        double check_car_s = v[5];
        if(check_car_s - curr_state.s() < 30)
        {
            double vx = v[3];
            double vy = v[4];

            double speed = sqrt(vx * vx + vy * vy);
            vehicle veh(v[1], v[2], v[5], v[6], 0, speed);
            if(veh.get_cur_lane() == dst_lane)
            {
                return 20;
            }
        }
    }
    return 0;
}

double lane_emptyness_cost(const sensor_fusion_data_t &sf_data,
                           const vehicle &curr_state,
                           int dst_lane)
{
    for(const auto& v : sf_data)
    {
        double check_car_s = v[5];
        if((check_car_s > curr_state.s()) && (check_car_s - curr_state.s()) < 30)
        {
            vehicle veh(v[1], v[2], v[5], v[6], 0, 0);
            if (veh.get_cur_lane() == dst_lane)
            {
                return 1;
            }
        }
    }
    return 0;
}

double behavior_planner::calculate_cost(const sensor_fusion_data_t &sf_data,
        const vehicle &curr_state,
        const trajectory_t &trajectory,
        int dst_lane)
{
    if(std::get<0>(trajectory).empty())
    {
        return std::numeric_limits<double>::max();
    }

    auto cost = inefficiency_cost(49.5, sf_data, curr_state,
            curr_state.get_cur_lane(), dst_lane) + 99999 * lane_cost(dst_lane) +
            99 * collision_cost(sf_data, curr_state, curr_state.get_cur_lane(), dst_lane) +
            5 * lane_emptyness_cost(sf_data, curr_state, dst_lane);

    return cost;
}
