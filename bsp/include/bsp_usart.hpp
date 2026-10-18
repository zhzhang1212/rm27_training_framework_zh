#ifndef BSP_USART_HPP
#define BSP_USART_HPP

#include "usart.h"

enum USART_Mode
{
    USART_MODE_BLOCK = 0,
    USART_MODE_DMA = 1,
    USART_MODE_IT = 2
};

/**
 * @brief 检查 USART1 和 USART6 及其 DMA 是否已由 CubeMX 初始化。
 * @return 两个串口均就绪时返回 true
 */
bool USART_Init(void);

/**
 * @brief 检查 USART1 及其 DMA 是否就绪。
 * @return 句柄和 DMA 句柄有效时返回 true
 */
bool USART1_Init(void);

/**
 * @brief 检查 USART6 及其 DMA 是否就绪。
 * @return 句柄和 DMA 句柄有效时返回 true
 */
bool USART6_Init(void);

/**
 * @brief 串口发送数据。
 * @param huart 指向串口句柄的指针
 * @param pData 发送的数据
 * @param Size 数据长度
 * @param mode 发送模式
 * @return HAL 操作状态；参数或模式无效时返回 HAL_ERROR
 */
HAL_StatusTypeDef USART_Transmit(UART_HandleTypeDef *huart,
                                 const uint8_t *pData,
                                 uint16_t Size,
                                 USART_Mode mode);

/**
 * @brief 串口接收数据。
 * @param huart 指向串口句柄的指针
 * @param pData 接收数据的缓冲区
 * @param Size 数据长度
 * @param mode 接收模式；DMA 和中断模式使用空闲线检测
 * @return HAL 操作状态；参数或模式无效时返回 HAL_ERROR
 * @note 非阻塞模式下，缓冲区必须保持有效，直到接收停止。
 */
HAL_StatusTypeDef USART_Receive(UART_HandleTypeDef *huart,
                                uint8_t *pData,
                                uint16_t Size,
                                USART_Mode mode = USART_MODE_DMA);

#endif // BSP_USART_HPP
