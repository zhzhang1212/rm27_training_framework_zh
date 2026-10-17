#include "LED.hpp"

#include "main.h"

namespace
{
struct LedPin
{
    GPIO_TypeDef *port;
    uint16_t pin;
};


constexpr GPIO_PinState LED_ON_STATE = GPIO_PIN_SET;
constexpr GPIO_PinState LED_OFF_STATE = GPIO_PIN_RESET;

LedPin GetLedPin(LED::Channel channel)
{
    switch (channel)
    {
    case LED::Channel::Red:
        return {LED_R_GPIO_Port, LED_R_Pin}; // 返回红色LED的GPIO端口和引脚
    case LED::Channel::Green:
        return {LED_G_GPIO_Port, LED_G_Pin};
    case LED::Channel::Blue:
        return {LED_B_GPIO_Port, LED_B_Pin};
    }

    return {nullptr, 0U};
}
}

namespace LED
{
void Init()
{
    AllOff();
}

void Set(Channel channel, bool on)
{
    const LedPin led = GetLedPin(channel);
    if (led.port == nullptr)
    {
        return;
    }

    HAL_GPIO_WritePin(led.port, led.pin, on ? LED_ON_STATE : LED_OFF_STATE);
}

void Toggle(Channel channel)
{
    const LedPin led = GetLedPin(channel);
    if (led.port == nullptr)
    {
        return;
    }

    HAL_GPIO_TogglePin(led.port, led.pin);
}

void SetRGB(bool red, bool green, bool blue)
{
    Set(Channel::Red, red);
    Set(Channel::Green, green);
    Set(Channel::Blue, blue);
}

void AllOff()
{
    SetRGB(false, false, false);
}
}
