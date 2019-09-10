//
// Created by grigorii on 9/8/19.
//

#include "behavior_planner.h"

#include <algorithm>

#include "helpers.h"
#include "spline.h"

trajectory_t behavior_planner::get_trajectory(const sensor_fusion_data_t &sf_data,
                                              const vehicle_state_t &curr_state,
                                              const trajectory_t &prev_path,
                                              const std::pair<double, double> &prev_path_end_frenet)
{
//    const auto& next_states = action_adjacency.find(m_curr_action);
//    if(next_states != std::end(action_adjacency))
//    {
//        std::vector<double> costs;
//        std::vector<trajectory_t> final_trajectories;
//        for(const auto& state : next_states->second)
//        {
//            auto trajectory = generate_trajectory(state, sf_data,
//                    prev_path, curr_state, prev_path_end_frenet);
//            if (!std::get<0>(trajectory).empty())
//            {
//                auto cost = 0;//calculate_cost(*this, predictions, trajectory);
//                costs.push_back(cost);
//                final_trajectories.push_back(trajectory);
//            }
//        }
//
//        auto best_cost = std::min_element(begin(costs), end(costs));
//        int best_idx = std::distance(begin(costs), best_cost);
//
//        return final_trajectories[best_idx];
//    }
//    return {{}, {}};
    return generate_trajectory(action::KL,
            sf_data,
            prev_path,
            curr_state,
            prev_path_end_frenet);
}

trajectory_t behavior_planner::generate_trajectory(action dst_action,
        const sensor_fusion_data_t &sf_data,
        const trajectory_t &prev_path,
        const vehicle_state_t &curr_state,
        const std::pair<double, double> &prev_path_end_frenet)
{
    const auto& [previous_path_x, previous_path_y] = prev_path;
    double car_x = curr_state[0];
    double car_y = curr_state[1];
    double car_s = curr_state[2];
    double car_d = curr_state[3];
    double car_yaw = curr_state[4];
    double car_speed = curr_state[5];
    auto end_path_s = prev_path_end_frenet.first;

    std::vector<double> pts_x;
    std::vector<double> pts_y;

    auto ref_x = car_x;
    auto ref_y = car_y;
    auto ref_yaw = deg2rad(car_yaw);

    auto prev_size = previous_path_x.size();

    if(prev_size > 0)
    {
        car_s = end_path_s;
    }

    bool too_close = false;

    for(int i = 0; i < sf_data.size(); i++)
    {
        float d = sf_data[i][6];
        if(d < (2 + 4 * m_lane + 2) && d > (2 + 4 * m_lane - 2))
        {
            double vx = sf_data[i][3];
            double vy = sf_data[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sf_data[i][5];

            check_car_s += static_cast<double>(prev_size) * 0.02 * check_speed;

            if((check_car_s > car_s) && (check_car_s - car_s) < 30)
            {
                //ref_vel = 29.5;
                too_close = true;
                if(m_lane > 0)
                {
                    m_lane = 0;
                }
            }
        }
    }

    if(too_close)
    {
        m_ref_vel -= 0.224;
    }
    else if(m_ref_vel < 49.5)
    {
        m_ref_vel += 0.224;
    }

    if(prev_size < 2)
    {
        auto prev_car_x = car_x - cos(car_yaw);
        auto prev_car_y = car_y - sin(car_yaw);

        pts_x.emplace_back(prev_car_x);
        pts_x.emplace_back(car_x);

        pts_y.emplace_back(prev_car_y);
        pts_y.emplace_back(car_y);
    }
    else
    {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        pts_x.emplace_back(ref_x_prev);
        pts_x.emplace_back(ref_x);

        pts_y.emplace_back(ref_y_prev);
        pts_y.emplace_back(ref_y);
    }

    auto next_wp0 = getXY(car_s + 30, (2 + 4 * m_lane), m_map_waypoints_s,
                          m_map_waypoints_x, m_map_waypoints_y);
    auto next_wp1 = getXY(car_s + 60, (2 + 4 * m_lane), m_map_waypoints_s,
                          m_map_waypoints_x, m_map_waypoints_y);
    auto next_wp2 = getXY(car_s + 90, (2 + 4 * m_lane), m_map_waypoints_s,
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

    for(int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.emplace_back(previous_path_x[i]);
        next_y_vals.emplace_back(previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = spline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double x_add_on = 0;

    for(int i = 1; i <= 50 - previous_path_x.size(); i++)
    {
        double N = (target_dist / (0.02 * m_ref_vel / 2.24));
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

behavior_planner::behavior_planner(const std::vector<double> &map_wp_x,
        const std::vector<double> &map_wp_y,
        const std::vector<double> &map_wp_s,
        int start_lane,
        double ref_vel) :
            m_map_waypoints_x(map_wp_x),
            m_map_waypoints_y(map_wp_y),
            m_map_waypoints_s(map_wp_s),
            m_lane(start_lane),
            m_ref_vel(ref_vel)
{

}
