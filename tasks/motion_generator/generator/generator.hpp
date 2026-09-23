#ifndef _GENERATOR_HPP_
#define _GENERATOR_HPP_

#include <yaml-cpp/yaml.h>
#include <memory>
#include <mutex>
#include <thread>

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
    std::unique_ptr<TranslationGenerator> translation_;
    std::unique_ptr<SpinGenerator> spin_;
    std::mutex state_;
    std::thread motion_thread_;

private:
    
};

} // namespace motion_generator

#endif // _GENERATOR_HPP_