/**
 * @file plotjuggler.cpp
 * @brief @ref tools::PlotJuggler 的实现。
 */

#include "plotjuggler.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdexcept>

namespace tools
{

PlotJuggler::PlotJuggler(const std::string& host, const uint16_t port)
{
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0)
    {
        throw std::runtime_error("PlotJuggler: failed to create UDP socket");
    }

    destination_.sin_family = AF_INET;
    destination_.sin_port = ::htons(port);
    destination_.sin_addr.s_addr = ::inet_addr(host.c_str());
}

PlotJuggler::~PlotJuggler()
{
    if (socket_ >= 0)
    {
        ::close(socket_);
    }
}

void PlotJuggler::plot(const nlohmann::json& json)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto data = json.dump();
    ::sendto(
        socket_, data.c_str(), data.length(), 0,
        reinterpret_cast<sockaddr*>(&destination_), sizeof(destination_));
}

} // namespace tools
