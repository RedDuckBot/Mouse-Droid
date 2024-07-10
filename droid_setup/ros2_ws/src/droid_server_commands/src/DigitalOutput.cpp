#include "libgpio/DigitalOutput.hpp"

#include <stdexcept>
#include <fmt/core.h>

//File taken from repo: https://github.com/buildrobotsbetter/rpi4b_gpio-example
namespace libgpio
{

DigitalOutput::DigitalOutput(uint32_t gpioPin):
    m_gpioPin(gpioPin)
{
    auto result = gpioSetMode(m_gpioPin, PI_OUTPUT);
    if (result == 0)
    {
        return;
    }

    switch (result)
    {       
        case PI_BAD_GPIO:
        {
            throw std::runtime_error(fmt::format("{} is a bad gpio pin", m_gpioPin));
        }

        case PI_BAD_MODE:
        {
            throw std::runtime_error(fmt::format("Bad mode specified for gpio pin {}", m_gpioPin));
        }

        default:
        {
            throw std::runtime_error(fmt::format("Unexpected error encountered when setting mode specified for gpio pin {} (Error = {})", m_gpioPin, result));
        }
    }
}

DigitalOutput::~DigitalOutput()
{
    // Automatically turn off on destruction
    setOutput(false);
}

bool DigitalOutput::getState()
{
    return pin_state;
}

void DigitalOutput::setOutput(bool value)
{
    auto result = gpioWrite(m_gpioPin, static_cast<unsigned int>(value));
    if (result == 0)
    {
        pin_state = value;
        return;
    }

    switch (result)
    {       
        case PI_BAD_GPIO:
        {
            throw std::runtime_error(fmt::format("{} is a bad gpio pin", m_gpioPin));
        }

        case PI_BAD_LEVEL:
        {
            throw std::runtime_error(fmt::format("Bad level specified for gpio pin {}", m_gpioPin));
        }

        default:
        {
            throw std::runtime_error(fmt::format("Unexpected error encountered when setting mode specified for gpio pin {} (Error = {})", m_gpioPin, result));
        }
    }
}

} // namespace libgpio
