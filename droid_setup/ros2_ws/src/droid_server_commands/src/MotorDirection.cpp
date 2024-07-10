#include "libgpio/MotorDirection.hpp"

#include <stdexcept>

//File taken from repo: https://github.com/buildrobotsbetter/rpi4b_gpio-example
namespace libgpio
{

MotorDirection::operator Value() const
{
    return m_value;
}

std::string MotorDirection::toString() const
{
    switch (m_value)
    {
        case FORWARD:
            return "FORWARD";

        case BACKWARD:
            return "BACKWARD";
    }

    throw std::runtime_error("Could convert MotorDirection to string");
}

} // namespace libgpio
