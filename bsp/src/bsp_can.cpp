#include "bsp_can.hpp"
#include "DJIMotorHandler.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

namespace
{
constexpr uint32_t CAN2_START_FILTER_BANK = 14U;

bool ConfigureAndStartCan(CAN_HandleTypeDef *hcan, uint32_t filter_bank)
{
    if (hcan == nullptr || (hcan->Instance != CAN1 && hcan->Instance != CAN2))
    {
        return false;
    }

    const HAL_CAN_StateTypeDef state = HAL_CAN_GetState(hcan);
    if (state == HAL_CAN_STATE_LISTENING)
    {
        return true;
    }
    if (state != HAL_CAN_STATE_READY)
    {
        return false;
    }

    CAN_FilterTypeDef filter{};
    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = filter_bank;
    filter.SlaveStartFilterBank = CAN2_START_FILTER_BANK;

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK ||
        HAL_CAN_Start(hcan) != HAL_OK)
    {
        return false;
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        (void)HAL_CAN_Stop(hcan);
        return false;
    }

    return true;
}
}

/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
bool CAN_Init(void)
{
    if (!ConfigureAndStartCan(&hcan1, 0U))
    {
        return false;
    }

    if (!ConfigureAndStartCan(&hcan2, CAN2_START_FILTER_BANK))
    {
        (void)HAL_CAN_Stop(&hcan1);
        return false;
    }

    return true;
}

HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan,
                               uint32_t Id,
                               const uint8_t *msg,
                               uint16_t len)
{
    if (hcan == nullptr ||
        (hcan != &hcan1 && hcan != &hcan2) ||
        Id > 0x7FFU || len > 8U || (msg == nullptr && len > 0U))
    {
        return HAL_ERROR;
    }

    const HAL_CAN_StateTypeDef state = HAL_CAN_GetState(hcan);
    if (state != HAL_CAN_STATE_READY && state != HAL_CAN_STATE_LISTENING)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return HAL_BUSY;
    }

    CAN_TxHeaderTypeDef tx_header{};
    tx_header.StdId = Id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    // HAL发送函数固定读取8字节，先补零可避免短帧缓冲区越界。
    uint8_t frame[8]{};
    for (uint16_t index = 0U; index < len; ++index)
    {
        frame[index] = msg[index];
    }

    uint32_t mailbox = 0U;
    return HAL_CAN_AddTxMessage(hcan, &tx_header, frame, &mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) //当FIFO0接收到消息时，HAL库会调用此回调函数
{
    if (hcan != &hcan1 && hcan != &hcan2)
    {
        return;
    }

    CAN_RxHeaderTypeDef rx_header{};
    uint8_t rx_data[8]{};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) // 从FIFO0获取接收到的消息
    {
        return;
    }

    if (rx_header.IDE != CAN_ID_STD ||
        rx_header.RTR != CAN_RTR_DATA ||
        rx_header.DLC != 8U ||
        rx_header.StdId < 0x201U ||
        rx_header.StdId > 0x208U)
    {
        return;
    }

    const int index = static_cast<int>(rx_header.StdId - 0x201U);
    DJIMotorHandler::Instance()->updateFeedback(hcan, rx_data, index); //转交给电机管理器
}
