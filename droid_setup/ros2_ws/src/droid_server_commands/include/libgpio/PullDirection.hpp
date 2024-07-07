#ifndef LIBGPIO_PULLDIRECTION
#define LIBGPIO_PULLDIRECTION

#include <pigpio.h>
#include <string>

namespace libgpio
{
    
class PullDirection
{
public:
    // Enum values match the pigpio PI_PUD_xxx define values
    // to allow straight pass through to pigpio functions
    enum Value {
        OFF = PI_PUD_OFF,
        DOWN = PI_PUD_DOWN,
        UP = PI_PUD_UP
    };

    constexpr PullDirection(uint8_t value) :
        PullDirection((PullDirection::Value)value) {}
    constexpr PullDirection(Value value) :
        value(value) {};
    ~PullDirection() = default;

    operator Value() const;            // Allow switch and comparisons.
    explicit operator bool() = delete; // Prevent usage: if (status)

    std::string toString() const;

private:
    Value value;
};

} // namespace libgpio

#endif // LIBGPIO_PULLDIRECTION
