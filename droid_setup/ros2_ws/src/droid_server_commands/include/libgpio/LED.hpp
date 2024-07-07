#ifndef LIBGPIO_LED
#define LIBGPIO_LED

#include "DigitalOutput.hpp"

namespace libgpio
{
    
/// @brief An LED is a specialized type of DigitalOutput device.
/// This class works identically to the generic DigitalOutput class
/// and is provided as a convenience as a mechanism to improve code
/// readability. 
class LED : public DigitalOutput
{
public:
    LED(uint32_t gpioPin) : DigitalOutput(gpioPin) {};
    ~LED() override = default;
};

} // namespace libgpio

#endif // LIBGPIO_LED
