//
// Created by grigorii on 9/15/19.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H


class vehicle
{
    int m_curr_lane {1};
    double m_x {0};
    double m_y {0};
    double m_s {0};
    double m_d {0};
    double m_yaw {0};
    double m_speed {0};
public:
    vehicle() = default;
    vehicle(double x, double y,
            double s, double d,
            double yaw, double speed) :
            m_x(x), m_y(y), m_s(s), m_d(d),
            m_yaw(yaw), m_speed(speed)
    {}

    double x() const { return m_x; }
    void set_x(double x) { m_x = x; }
    double y() const { return m_y; }
    void set_y(double y) { m_y = y; }
    double s() const { return m_s; }
    void set_s(double s) { m_s = s; }
    double d() const { return m_d; }
    void set_d(double d) { m_d = d; }
    double yaw() const { return m_yaw; }
    void set_yaw(double yaw) { m_yaw = yaw; }
    double speed() const { return m_speed; }
    void set_speed(double speed) { m_speed = speed; }
    int get_cur_lane() const
    {
        if(m_d >= 0 && m_d < 4)
            return 0;
        else if(m_d >= 4 && m_d < 8)
            return 1;
        else if(m_d >= 8 && m_d <= 12)
            return 2;
        return -1;
    }
};


#endif //PATH_PLANNING_VEHICLE_H
