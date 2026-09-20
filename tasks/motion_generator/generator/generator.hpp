#ifndef _GENERATOR_HPP_
#define _GENERATOR_HPP_

#include <yaml-cpp/yaml.h>
#include <memory>

#include "spin/spin_base/spin_base.hpp"
#include "spin/sine/sine.hpp"
#include "spin/spin_uniform/spin_uniform.hpp"
#include "spin/random/random.hpp"

#include "translate/translate_base/translate_base.hpp"
#include "translate/translate_uniform/translate_uniform.hpp"
#include "translate/variation/variation.hpp"

#include "tools/logger/logger.hpp"

namespace motion_generator
{

enum class MotionType
{
    // spin
    SineSpin,
    UniformSpin,
    RandomSpin,
    
    // translate
    UniformTranslate,
    VariationTranslate,

    // mixed
    SineUniform,
    SineVariation,
    UniformUniform,
    UniformVariation,
    RandomUniform,
    RandomVariation
};

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