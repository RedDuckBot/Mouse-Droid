#ifndef LIBGPIO_DEVICE_H
#define LIBGPIO_DEVICE_H

//File taken from repo: https://github.com/buildrobotsbetter/rpi4b_gpio-example
#include <pigpio.h>

namespace libgpio
{

class Device
{
public:
    Device();
    virtual ~Device();

private:
    static uint64_t m_deviceCount;
};

} // namespace libgpio

#endif // LIBGPIO_DEVICE_H
