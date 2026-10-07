/**
 * @file plotjuggler.hpp
 * @brief 通过 UDP 将实时数据发送给 PlotJuggler 的轻量客户端。
 *
 * PlotJuggler 监听指定 UDP 端口后，可实时解析 JSON 数据流并绘制曲线。
 * 本类封装了 socket 的创建、发送与线程安全的互斥访问，便于在仿真循环中
 * 直接调用 @ref tools::PlotJuggler::plot。
 */

#ifndef VALIDATE_KALMAN_FILTER_TOOLS_PLOTJUGGLER_HPP_
#define VALIDATE_KALMAN_FILTER_TOOLS_PLOTJUGGLER_HPP_

#include <netinet/in.h>

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools
{

/**
 * @brief 向 PlotJuggler 发送 JSON 数据帧的 UDP 客户端。
 *
 * @note 该类持有系统 socket，禁止拷贝；请在进程生命周期内复用同一实例。
 */
class PlotJuggler
{
public:
    /**
     * @brief 创建 UDP 客户端并解析目标地址。
     *
     * @param host 目标主机地址，默认本机回环地址。
     * @param port 目标 UDP 端口，默认 9870（PlotJuggler 常用端口）。
     */
    explicit PlotJuggler(
        const std::string& host = "127.0.0.1",
        uint16_t port = 9870);

    /// @brief 关闭底层 socket。
    ~PlotJuggler();

    PlotJuggler(const PlotJuggler&) = delete;
    PlotJuggler& operator=(const PlotJuggler&) = delete;

    /**
     * @brief 将一帧 JSON 数据发送给 PlotJuggler。
     *
     * 每个 JSON 字段会被 PlotJuggler 当作一条曲线上的一个采样点。该方法内部
     * 加锁，可从多个线程并发调用。
     *
     * @param json 待发送的数据帧，字段名即曲线名。
     */
    void plot(const nlohmann::json& json);

private:
    int socket_{-1};             ///< UDP socket 文件描述符。
    sockaddr_in destination_{};  ///< 目标地址（已填充 family/port/addr）。
    std::mutex mutex_;           ///< 保护 @ref socket_ 的并发发送。
};

} // namespace tools

#endif // VALIDATE_KALMAN_FILTER_TOOLS_PLOTJUGGLER_HPP_
