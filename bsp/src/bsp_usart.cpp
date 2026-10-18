#include "bsp_usart.hpp"

namespace
{
constexpr uint32_t USART_BLOCK_TIMEOUT_MS = 100U;

bool IsConfiguredUart(const UART_HandleTypeDef *huart)
{
    return huart != nullptr &&
           (huart->Instance == USART1 || huart->Instance == USART6) &&
           huart->gState != HAL_UART_STATE_RESET &&
           huart->RxState != HAL_UART_STATE_RESET;
}

bool IsDmaReady(const UART_HandleTypeDef *huart)
{
    return IsConfiguredUart(huart) &&
           huart->hdmarx != nullptr &&
           huart->hdmatx != nullptr;
}
}

bool USART_Init(void)
{
    return USART1_Init() && USART6_Init();
}

bool USART1_Init(void)
{
    return IsDmaReady(&huart1);
}

bool USART6_Init(void)
{
    return IsDmaReady(&huart6);
}

HAL_StatusTypeDef USART_Transmit(UART_HandleTypeDef *huart,
                                 const uint8_t *pData,
                                 uint16_t Size,
                                 USART_Mode mode)
{
    if (!IsConfiguredUart(huart) || pData == nullptr || Size == 0U)
    {
        return HAL_ERROR;
    }

    switch (mode)
    {
    case USART_MODE_BLOCK:
        return HAL_UART_Transmit(huart, pData, Size, USART_BLOCK_TIMEOUT_MS);

    case USART_MODE_DMA:
        if (huart->hdmatx == nullptr)
        {
            return HAL_ERROR;
        }
        return HAL_UART_Transmit_DMA(huart, pData, Size);

    case USART_MODE_IT:
        return HAL_UART_Transmit_IT(huart, pData, Size);

    default:
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef USART_Receive(UART_HandleTypeDef *huart,
                                uint8_t *pData,
                                uint16_t Size,
                                USART_Mode mode)
{
    if (!IsConfiguredUart(huart) || pData == nullptr || Size == 0U)
    {
        return HAL_ERROR;
    }

    switch (mode)
    {
    case USART_MODE_BLOCK:
        return HAL_UART_Receive(huart, pData, Size, USART_BLOCK_TIMEOUT_MS);

    case USART_MODE_DMA:
        if (huart->hdmarx == nullptr)
        {
            return HAL_ERROR;
        }
        return HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);

    case USART_MODE_IT:
        return HAL_UARTEx_ReceiveToIdle_IT(huart, pData, Size);

    default:
        return HAL_ERROR;
    }
}
