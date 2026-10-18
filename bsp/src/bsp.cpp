#include "bsp.hpp"

#include "bsp_can.hpp"
#include "bsp_usart.hpp"

bool bsp_Init()
{
    const bool usart_ready = USART_Init();
    const bool can_ready = CAN_Init();
    return usart_ready && can_ready;
}
