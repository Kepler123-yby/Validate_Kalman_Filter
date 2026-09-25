#include "plotjuggler.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace tools
{
PlotJuggler::PlotJuggler(const std::string& host, const uint16_t port)
{
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

    destination_.sin_family = AF_INET;
    destination_.sin_port = ::htons(port);
    destination_.sin_addr.s_addr = ::inet_addr(host.c_str());
}

PlotJuggler::~PlotJuggler() { ::close(socket_); }

void PlotJuggler::plot(const nlohmann::json & json)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto data = json.dump();
  ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));
}

} // namespace tools