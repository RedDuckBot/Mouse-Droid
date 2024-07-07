#ifndef LIBGPIO_PULLDIRECTION
#define LIBGPIO_PULLDIRECTION

#include <pigpio.h>
#include <string>

namespace libgpio
{
    
class MotorDirection
{
public:
    // Enum values match the pigpio PI_PUD_xxx define values
    // to allow straight pass through to pigpio functions
    enum Value {
        FORWARD,
        BACKWARD
    };

    constexpr MotorDirection() :
        MotorDirection(MotorDirection::FORWARD) {}
    constexpr MotorDirection(uint8_t value) :
        MotorDirection((MotorDirection::Value)value) {}
    constexpr MotorDirection(Value value) :
        m_value(value) {};
    ~MotorDirection() = default;

    operator Value() const;            // Allow switch and comparisons.
    explicit operator bool() = delete; // Prevent usage: if (status)

    std::string toString() const;

private:
    Value m_value;
};

} // namespace libgpio

#endif // LIBGPIO_PULLDIRECTION
