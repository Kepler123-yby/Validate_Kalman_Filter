/**
 * @file exiter.cpp
 * @brief @ref tools::Exiter 的实现。
 */

#include "exiter.hpp"

namespace tools
{

volatile std::sig_atomic_t Exiter::exit_flag_ = 0;
bool Exiter::exiter_inited_ = false;

Exiter::Exiter()
{
    if (exiter_inited_)
    {
        throw std::runtime_error("Multiple Exiter instances!");
    }

    // 信号处理函数中只能执行异步信号安全的操作，这里仅写入原子标志。
    std::signal(SIGINT, [](int) { exit_flag_ = 1; });
    exiter_inited_ = true;
}

bool Exiter::exit() const
{
    return exit_flag_ != 0;
}

} // namespace tools
