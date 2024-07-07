#include "libgpio/MotorDriver.hpp"

#include <stdexcept>
#include <fmt/core.h>
#include <iostream>

namespace libgpio
{

MotorDriver::MotorDriver(uint32_t input1Pin, uint32_t input2Pin, uint32_t enablePin):
    m_input1Pin(input1Pin),
    m_input2Pin(input2Pin),
    m_enablePin(enablePin),
    m_effort_percent(0.0)
{
    setDirection(MotorDirection::FORWARD);
}

MotorDriver::~MotorDriver()
{
    // Automatically stop the motor on destruction
    setEffort_percent(0.0);
}

const MotorDirection& MotorDriver::getDirection() const
{
    return m_direction;
}

void MotorDriver::setDirection(const MotorDirection& value)
{
    m_direction = value;
    switch (m_direction)
    {
        case MotorDirection::FORWARD:
        {        
            m_input1Pin.setOutput(true);
            m_input2Pin.setOutput(false);
            break;
        }

        case MotorDirection::BACKWARD:
        {        
            m_input1Pin.setOutput(false);
            m_input2Pin.setOutput(true);
            break;
        }
    }
}

double MotorDriver::getEffort_percent() const
{
    return m_effort_percent;
}

void MotorDriver::setEffort_percent(double value)
{
    if ((value < 0.0) || (value > 100.0))
    {
        throw std::runtime_error(fmt::format("Effort value {} is invalid. Valid range [0.0, 100.0]", value)); 
    }

    m_effort_percent = value;

    int duty_cycle = (int) (255 * (m_effort_percent / 100.0));
    auto result = gpioPWM(m_enablePin, duty_cycle);
    if (result == 0)
    {
        return;
    }

    switch (result)
    {       
        case PI_BAD_USER_GPIO:
        {
            throw std::runtime_error(fmt::format("{} is a bad gpio pin", m_enablePin));
        }

        case PI_BAD_DUTYCYCLE:
        {
            throw std::runtime_error(fmt::format("Bad duty cycle specified for gpio pin {}", m_enablePin));
        }

        default:
        {
            throw std::runtime_error(fmt::format("Unexpected error encountered when setting mode specified for gpio pin {} (Error = {})", m_enablePin, result));
        }
    }
}

} // namespace libgpio
