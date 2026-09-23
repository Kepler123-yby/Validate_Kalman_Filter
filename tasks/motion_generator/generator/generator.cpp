#include "generator.hpp"

namespace motion_generator
{

Generator::Generator(const YAML::Node& config)
{
    auto spinConfig = config["spin"];
    auto translationConfig = config["translation"];

    this->translation_ = std::make_unique<TranslationGenerator>(translationConfig);
    this->spin_ = std::make_unique<SpinGenerator>(spinConfig);
}

} // namespace motion_generator