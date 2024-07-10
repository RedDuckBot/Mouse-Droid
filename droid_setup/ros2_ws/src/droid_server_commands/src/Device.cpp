#include "libgpio/Device.hpp"

#include <stdexcept>
#include <fmt/core.h>

//File taken from repo: https://github.com/buildrobotsbetter/rpi4b_gpio-example

namespace libgpio
{

// Initialize the static variable
uint64_t Device::m_deviceCount = 0;

Device::Device()
{
    if (m_deviceCount == 0)
    {
        #ifndef USE_PIGPIO_SIGNALHANDLER
        // Disable built-in pigpio signal handling
        // Must be called before gpioInitialise()
        int cfg = gpioCfgGetInternals();
        cfg |= PI_CFG_NOSIGHANDLER;
        gpioCfgSetInternals(cfg);
        #endif

        // Initialize pigpio library
        int result = gpioInitialise();
        if (result == PI_INIT_FAILED)
        {
            throw std::runtime_error(fmt::format("pigpio initialization Failed"));
        }
    }
    m_deviceCount++;
}

Device::~Device()
{
    m_deviceCount--;
    if (m_deviceCount == 0)
    {
        gpioTerminate();
    }
}

} // namespace libgpio
