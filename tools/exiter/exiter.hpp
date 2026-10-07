/**
 * @file exiter.hpp
 * @brief 基于 Ctrl-C (SIGINT) 的优雅退出标志。
 *
 * 仿真/测试程序通常运行在无限循环中，需要一种简单方式响应中断请求。
 * @ref tools::Exiter 在构造时注册 SIGINT 信号处理函数，把退出意图记录到
 * 全局原子标志中，业务循环通过 @ref tools::Exiter::exit 轮询即可。
 */

#ifndef VALIDATE_KALMAN_FILTER_TOOLS_EXITER_HPP_
#define VALIDATE_KALMAN_FILTER_TOOLS_EXITER_HPP_

#include <csignal>
#include <stdexcept>

namespace tools
{

/**
 * @brief 全局唯一的 Ctrl-C 退出标志管理器。
 *
 * @note 由于信号处理函数是进程级的，该类被设计为单例语义：重复构造同一
 *       进程内的第二个实例会抛出 @c std::runtime_error。
 */
class Exiter
{
public:
    /**
     * @brief 注册 SIGINT 处理函数。
     *
     * @throws std::runtime_error 当同一进程中已存在另一个 @ref Exiter 实例时。
     */
    Exiter();

    /**
     * @brief 查询是否收到过 Ctrl-C。
     *
     * @return 收到过 SIGINT 返回 @c true，否则返回 @c false。
     */
    bool exit() const;

private:
    static volatile std::sig_atomic_t exit_flag_; ///< 信号安全的退出标志。
    static bool exiter_inited_;                   ///< 是否已有实例注册过信号。
};

} // namespace tools

#endif // VALIDATE_KALMAN_FILTER_TOOLS_EXITER_HPP_
