#ifndef LIBGPIO_LED
#define LIBGPIO_LED

#include "Device.hpp"
#include "PullDirection.hpp"

//File taken from repo: https://github.com/buildrobotsbetter/rpi4b_gpio-example
namespace libgpio
{
    
class Switch : public Device
{
public:
    Switch(uint32_t gpioPin, PullDirection pullDirection);
    ~Switch() = default;

    /// @brief Get the input generated by the switch
    /// @param value true = HIGH, false = LOW
    bool getInput() const;

private:
    void configurePull();
    void configureMode();

    uint32_t m_gpioPin;
    PullDirection m_pullDirection;
};

} // namespace libgpio

#endif // LIBGPIO_LED
