#include "generator.hpp"

namespace motion_generator
{

Generator::Generator(const YAML::Node& config) : config_(config)
{
    int motion_id = config["motion_type"] ? config["motion_type"].as<int>() : 1;
    MotionType motion_type = static_cast<MotionType>(motion_id);

    if(init_motion_generator())
    {
        tools::logger()->info("Motion generator initialized successfully.");
    } else {
        tools::logger()->error("Failed to initialize motion generator.");
    }
}

bool Generator::init_motion_generator()
{
    if (motion_types_ == MotionType::SineSpin)
    {
        spin_ = std::make_shared<Sine>(config);
        return true;
    }
    else if (motion_types_ == MotionType::UniformSpin)
    {
        spin_ = std::make_shared<SpinUniform>(config);
        return true;
    }
    else if (motion_types_ == MotionType::RandomSpin)
    {
        spin_ = std::make_shared<Random>(config);
        return true;
    }
    else if (motion_types_ == MotionType::UniformTranslate)
    {
        translate_ = std::make_shared<TranslateUniform>(config);
        return true;
    }
    else if (motion_types_ == MotionType::VariationTranslate)
    {
        translate_ = std::make_shared<Variation>(config);
        return true;
    }
    else if (motion_types_ == MotionType::SineUniform)
    {
        spin_ = std::make_shared<Sine>(config);
        translate_ = std::make_shared<TranslateUniform>(config);
        return true;
    }
    else if (motion_types_ == MotionType::SineVariation)
    {
        spin_ = std::make_shared<Sine>(config);
        translate_ = std::make_shared<Variation>(config);
        return true;
    }
    else if (motion_types_ == MotionType::UniformUniform)
    {
        spin_ = std::make_shared<SpinUniform>(config);
        translate_ = std::make_shared<TranslateUniform>(config);
        return true;
    }
    else if (motion_types_ == MotionType::UniformVariation)
    {
        spin_ = std::make_shared<SpinUniform>(config);
        translate_ = std::make_shared<Variation>(config);
        return true;
    }
    else if (motion_types_ == MotionType::RandomUniform)
    {
        spin_ = std::make_shared<Random>(config);
        translate_ = std::make_shared<TranslateUniform>(config);
        return true;
    }
    else if (motion_types_ == MotionType::RandomVariation)
    {
        spin_ = std::make_shared<Random>(config);
        translate_ = std::make_shared<Variation>(config);
        return true;
    }
    else
    {
        tools::logger()->error("Unsupported motion type.");
        return false;
    }
}

} // namespace motion_generator