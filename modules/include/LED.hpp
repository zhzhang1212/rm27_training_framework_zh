#ifndef LED_HPP
#define LED_HPP

namespace LED
{
enum class Channel
{
    Red,
    Green,
    Blue,
};

void Init();
void Set(Channel channel, bool on);
void Toggle(Channel channel);
void SetRGB(bool red, bool green, bool blue);
void AllOff();
}

#endif // LED_HPP
