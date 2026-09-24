#ifndef _ROBOT_HPP_
#define _ROBOT_HPP_

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <vector>

#include "tools/math_tools/math_tools.hpp"

namespace motion_generator
{

struct Armor
{
    int id_;
    Eigen::Vector4d observation;

    explicit Armor(const int id);
};

class Robot
{
public:
    Robot() = default;
    explicit Robot(const YAML::Node& config);
    ~Robot() = default;

    // 禁止拷贝
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    const Eigen::VectorXd& get_states() const { return states_; }

    const Eigen::Vector4d& get_observation() const;
    Eigen::Vector4d& get_observation();

    int get_locked_id() const { return locked_id_; }
    int get_switch_times() const { return switch_times_; }

    void update_state(const Eigen::VectorXd& raw_state);

private:
    std::vector<Armor> armors_;
    
    Eigen::VectorXd states_;
    int locked_id_{0};
    int switch_times_{0};

    double detect_min_threshold_{-60.0};
    double detect_max_threshold_{60.0};

    Eigen::MatrixXd raw2states_mat_;

    int armor_nums_{4};

private:
    // raw_states: x vx ax y vy ay z vz az angle w aw r dl height 
    void raw2states(const Eigen::VectorXd& raw_states);

    void raw2observation(const Eigen::VectorXd& raw_states);

    void update_locked_id();

};

} // namespace motion_generator

#endif // _ROBOT_HPP_
