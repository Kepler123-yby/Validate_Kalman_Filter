#ifndef _TOOLS_LOGGER_HPP_
#define _TOOLS_LOGGER_HPP_

#include <spdlog/spdlog.h>

namespace tools
{
    
std::shared_ptr<spdlog::logger> logger();

}  // namespace tools

#endif // _TOOLS_LOGGER_HPP_