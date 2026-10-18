#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"

void DJIMotorHandler::registerMotor(DJIMotor *motor, CAN_HandleTypeDef *hcan, uint16_t canId)
{
    if (motor == nullptr || canId < 0x201U || canId > 0x208U)
    {
        return;
    }

    int bus = -1;
    if (hcan == &hcan1)
    {
        bus = 0;
    }
    else if (hcan == &hcan2)
    {
        bus = 1;
    }
    else
    {
        return;
    }

    const int index = static_cast<int>(canId - 0x201U); //将canID转化成电机下标
    if (DJIMotorList[bus][index] != nullptr && DJIMotorList[bus][index] != motor)
    {
        return;
    }

    DJIMotorList[bus][index] = motor;
    motor->hcan = hcan;
    motor->canId = canId; //建立双向关系

    const bool low_group = canId <= 0x204U;
    if (bus == 0)
    {
        if (low_group) CAN1_0x200_Exist = true;
        else CAN1_0x1FF_Exist = true;
    }
    else
    {
        if (low_group) CAN2_0x200_Exist = true;
        else CAN2_0x1FF_Exist = true;
    }
}

HAL_StatusTypeDef DJIMotorHandler::sendControlData()
{
    for (int i = 0; i < 8; ++i)
    {
        can1_send_data_0[i] = 0;
        can1_send_data_1[i] = 0;
        can2_send_data_0[i] = 0;
        can2_send_data_1[i] = 0; //清除缓冲区
    }

    for (int bus = 0; bus < 2; ++bus)
    {
        for (int index = 0; index < 8; ++index)
        {
            DJIMotor *motor = DJIMotorList[bus][index];
            if (motor == nullptr)
            {
                continue;
            }

            uint8_t *frame = nullptr;
            if (bus == 0)
            {
                frame = index < 4 ? can1_send_data_0 : can1_send_data_1;
            }
            else
            {
                frame = index < 4 ? can2_send_data_0 : can2_send_data_1;
            }

            const int frame_index = index % 4;
            const uint16_t current = static_cast<uint16_t>(motor->currentSet);
            frame[frame_index * 2] = static_cast<uint8_t>(current >> 8);
            frame[frame_index * 2 + 1] = static_cast<uint8_t>(current);
        }
    }

    HAL_StatusTypeDef status = HAL_OK;

    if (CAN1_0x200_Exist)
    {
        status = CAN_Transmit(&hcan1, 0x200U, can1_send_data_0, 8U);
    }
    if (CAN1_0x1FF_Exist)
    {
        const HAL_StatusTypeDef current_status = CAN_Transmit(&hcan1, 0x1FFU, can1_send_data_1, 8U);
        if (status == HAL_OK)
        {
            status = current_status;
        }
    }
    if (CAN2_0x200_Exist)
    {
        const HAL_StatusTypeDef current_status = CAN_Transmit(&hcan2, 0x200U, can2_send_data_0, 8U);
        if (status == HAL_OK)
        {
            status = current_status;
        }
    }
    if (CAN2_0x1FF_Exist)
    {
        const HAL_StatusTypeDef current_status = CAN_Transmit(&hcan2, 0x1FFU, can2_send_data_1, 8U);
        if (status == HAL_OK)
        {
            status = current_status;
        }
    }

    return status;
}

void DJIMotorHandler::updateFeedback(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    if (rx_data == nullptr || index < 0 || index >= 8)
    {
        return;
    }

    int bus = -1;
    if (hcan == &hcan1)
    {
        bus = 0;
    }
    else if (hcan == &hcan2)
    {
        bus = 1;
    }
    else
    {
        return;
    }

    DJIMotor *motor = DJIMotorList[bus][index];
    if (motor != nullptr)
    {
        UpdateSensorData(motor, rx_data);
    }
}

void DJIMotorHandler::UpdateSensorData(DJIMotor *motor, uint8_t *can_data)
{
    if (motor == nullptr || can_data == nullptr)
    {
        return;
    }

    constexpr float two_pi = 6.28318530718f;
    constexpr float encoder_resolution = 8192.0f;

    float reduction_ratio = 1.0f;
    if (motor->gearBox == GearBox_M2006)
    {
        reduction_ratio = 36.0f;
    }
    else if (motor->gearBox == GearBox_M3508)
    {
        reduction_ratio = 3591.0f / 187.0f;
    }

    const uint16_t ecd = static_cast<uint16_t>((can_data[0] << 8) | can_data[1]);
    const int16_t speed_rpm = static_cast<int16_t>((can_data[2] << 8) | can_data[3]);
    const int16_t current = static_cast<int16_t>((can_data[4] << 8) | can_data[5]);

    motor->motorFeedback.lastSpeedFdb = motor->motorFeedback.speedFdb;
    motor->motorFeedback.speed_rpm = speed_rpm;
    motor->motorFeedback.speedFdb =
        static_cast<float>(speed_rpm) * two_pi / 60.0f / reduction_ratio; //转子速度转化为角速度
    motor->motorFeedback.currentFdb = static_cast<float>(current);
    motor->motorFeedback.temperatureFdb = static_cast<float>(can_data[6]);

    if (motor->AliveFlag == 0U)
    {
        motor->motorFeedback.ecd = ecd;
        motor->motorFeedback.last_ecd = static_cast<int16_t>(ecd);
        motor->motorFeedback.lastPositionFdb = motor->motorFeedback.positionFdb;
    }
    else
    {
        int32_t delta = static_cast<int32_t>(ecd) - motor->motorFeedback.last_ecd;
        if (delta > 4096)
        {
            delta -= 8192;
        }
        else if (delta < -4096)
        {
            delta += 8192;
        }

        motor->motorFeedback.lastPositionFdb = motor->motorFeedback.positionFdb;
        motor->motorFeedback.positionFdb +=
            static_cast<float>(delta) * two_pi / encoder_resolution / reduction_ratio; //编码器增量转化为角度增量
        motor->motorFeedback.ecd = ecd;
        motor->motorFeedback.last_ecd = static_cast<int16_t>(ecd);
    }

    ++motor->AliveFlag;
}

void DJIMotorHandler::AllMotorAliveCheck()
{
    for (int bus = 0; bus < 2; ++bus)
    {
        for (int index = 0; index < 8; ++index)
        {
            if (DJIMotorList[bus][index] != nullptr)
            {
                DJIMotorList[bus][index]->AliveCheck();
            }
        }
    }
}
