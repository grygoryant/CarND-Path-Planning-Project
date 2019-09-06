#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlj = nlohmann::json;

int main()
{
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;

    // Waypoint map to read from
    std::string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    std::string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    int lane = 1;
    double ref_vel = 0;

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                        &map_waypoints_dx, &map_waypoints_dy, &lane, &ref_vel]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode)
                {
                    // "42" at the start of the message means there's a websocket message event.
                    // The 4 signifies a websocket message
                    // The 2 signifies a websocket event
                    if (length && length > 2 && data[0] == '4' && data[1] == '2')
                    {

                        auto s = hasData(data);

                        if (s != "")
                        {
                            auto j = nlj::parse(s);

                            std::string event = j[0].get<string>();

                            if (event == "telemetry")
                            {
                                // j[1] is the data JSON object

                                // Main car's localization Data
                                double car_x = j[1]["x"];
                                double car_y = j[1]["y"];
                                double car_s = j[1]["s"];
                                double car_d = j[1]["d"];
                                double car_yaw = j[1]["yaw"];
                                double car_speed = j[1]["speed"];

                                // Previous path data given to the Planner
                                auto previous_path_x = j[1]["previous_path_x"];
                                auto previous_path_y = j[1]["previous_path_y"];
                                // Previous path's end s and d values
                                double end_path_s = j[1]["end_path_s"];
                                double end_path_d = j[1]["end_path_d"];

                                // Sensor Fusion Data, a list of all other cars on the same side
                                //   of the road.
                                auto sensor_fusion = j[1]["sensor_fusion"];

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

                                for(int i = 0; i < sensor_fusion.size(); i++)
                                {
                                    float d = sensor_fusion[i][6];
                                    if(d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
                                    {
                                        double vx = sensor_fusion[i][3];
                                        double vy = sensor_fusion[i][4];
                                        double check_speed = sqrt(vx * vx + vy * vy);
                                        double check_car_s = sensor_fusion[i][5];

                                        check_car_s += static_cast<double>(prev_size) * 0.02 * check_speed;

                                        if((check_car_s > car_s) && (check_car_s - car_s) < 30)
                                        {
                                            //ref_vel = 29.5;
                                            too_close = true;
                                            if(lane > 0)
                                            {
                                                lane = 0;
                                            }
                                        }
                                    }
                                }

                                if(too_close)
                                {
                                    ref_vel -= 0.224;
                                }
                                else if(ref_vel < 49.5)
                                {
                                    ref_vel += 0.224;
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

                                auto next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s,
                                        map_waypoints_x, map_waypoints_y);
                                auto next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s,
                                        map_waypoints_x, map_waypoints_y);
                                auto next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s,
                                        map_waypoints_x, map_waypoints_y);

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

                                nlj msgJson;
                                msgJson["next_x"] = next_x_vals;
                                msgJson["next_y"] = next_y_vals;

                                auto msg = "42[\"control\"," + msgJson.dump() + "]";

                                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                            }  // end "telemetry" if
                        } else
                        {
                            // Manual driving
                            std::string msg = "42[\"manual\",{}]";
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }
                    }  // end websocket if
                }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                   {
                       std::cout << "Connected!!!" << std::endl;
                   });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length)
                      {
                          ws.close();
                          std::cout << "Disconnected" << std::endl;
                      });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    } else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}