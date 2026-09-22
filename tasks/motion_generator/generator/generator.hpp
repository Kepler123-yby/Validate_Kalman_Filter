#ifndef _GENERATOR_HPP_
#define _GENERATOR_HPP_

#include <yaml-cpp/yaml.h>
#include <memory>

#include "spin/spin.hpp"
#include "translate/translate.hpp"

#include "tools/logger/logger.hpp"

namespace motion_generator
{


class Generator
{
public:
    explicit Generator(const YAML::Node& config);
    ~Generator() = default;

private:
    YAML::Node config_;

private:
    bool init_motion_generator() const; // 初始化运动生成器

};

} // namespace motion_generator

#endif // _GENERATOR_HPP_