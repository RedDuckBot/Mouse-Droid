#ifndef LIBGPIO_DIGITALOUTPUT
#define LIBGPIO_DIGITALOUTPUT

#include "Device.hpp"

//File taken from repo: https://github.com/buildrobotsbetter/rpi4b_gpio-example
namespace libgpio
{

class DigitalOutput : public Device
{
public:
    DigitalOutput(uint32_t gpioPin);
    ~DigitalOutput() override;

    /// @brief Set the output of the LED
    /// @param value true = On, false = Off
    void setOutput(bool value);

    bool getState();

private:
    unsigned int m_gpioPin;
    bool pin_state;
};

} // namespace libgpio

#endif // LIBGPIO_DIGITALOUTPUT
