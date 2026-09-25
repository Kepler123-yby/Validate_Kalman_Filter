#ifndef _TOOLS_PLOTJUGGLER_HPP_
#define _TOOLS_PLOTJUGGLER_HPP_

#include <netinet/in.h>
#include <mutex>
#include <nlohmann/json.hpp>

namespace tools
{

class PlotJuggler
{
public:
    PlotJuggler(const std::string& host = "127.0.0.1", const uint16_t port = 9870);

    ~PlotJuggler();

    void plot(const nlohmann::json & json);

private:
    int socket_;
    sockaddr_in destination_;
    std::mutex mutex_;
};

} // namespace tools

#endif // _TOOLS_PLOTJUGGLER_HPP_