#ifndef _EXITER_HPP_
#define _EXITER_HPP_

#include <csignal>
#include <stdexcept>

namespace tools
{

class Exiter
{
public:
    Exiter();

    bool exit() const;

private:
    inline static bool exit_ = false;
    inline static bool exiter_inited_ = false;

};


} // namespace tools

#endif // _EXITER_HPP_