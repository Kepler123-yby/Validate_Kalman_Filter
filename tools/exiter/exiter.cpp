#include "exiter.hpp"

namespace tools
{

Exiter::Exiter()
{
    if (exiter_inited_) throw std::runtime_error("Multiple Exiter instances!");
    std::signal(SIGINT, [](int) { exit_ = true; });
    exiter_inited_ = true;
}

bool Exiter::exit() const
{
    return exit_;
}

} // namespace