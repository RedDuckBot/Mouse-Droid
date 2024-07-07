#include "libgpio/PullDirection.hpp"

#include <stdexcept>

namespace libgpio
{

PullDirection::operator Value() const
{
    return value;
}

std::string PullDirection::toString() const
{
    switch (value)
    {
        case UP:
            return "UP";

        case DOWN:
            return "DOWN";

        case OFF:
            return "OFF";
    }

    throw std::runtime_error("Could convert PullDirection to string");
}

} // namespace libgpio
