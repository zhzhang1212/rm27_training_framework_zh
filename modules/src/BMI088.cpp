#include "BMI088.hpp"
#include "tx_api.h"
#include "bsp_pwm.hpp"
#include "spi.h"
#include <cmath>

namespace
{
constexpr uint32_t BMI088_SPI_TIMEOUT_MS = 10U;

struct ChipSelect
{
    GPIO_TypeDef *port;
    uint16_t pin;
};

bool GetChipSelect(BMI088::BMI088_SENSOR sensor, ChipSelect *chip_select)
{
    if (chip_select == nullptr)
    {
        return false;
    }

    switch (sensor)
    {
    case BMI088::BMI088_CS_ACC:
        *chip_select = {ACC_CS_GPIO_Port, ACC_CS_Pin};
        return true;
    case BMI088::BMI088_CS_GYRO:
        *chip_select = {GYRO_CS_GPIO_Port, GYRO_CS_Pin};
        return true;
    }

    return false;
}

int16_t DecodeInt16(const uint8_t *data)
{
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                                (static_cast<uint16_t>(data[1]) << 8U));
}
}

namespace BMI088
{
    /**
     * @brief BMI088 acc gyro 标定
     * @note 标定后的陀螺仪零偏存储在 Gyro_offset 中。
     * @attention 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定(RobotInit关闭了全局中断)
     * @attention 标定精度和等待时间有关。
     * @todo 将标定次数(等待时间)变为参数供设定
     * @section 整体流程为1.累加加速度数据计算gNrom()
     *                   2.累加陀螺仪数据计算零飘
     *                   3. 如果标定过程运动幅度过大,重新标定
     *                   4.保存标定参数
     */
    void cBMI088::Calibrate()
    {
        const int calib_samples = 4000; // 采样次数
        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        gyro_data_t temp_gyro;

        // 1. 清除旧的 Offset，防止叠加
        Gyro_offset[0] = 0.0f;
        Gyro_offset[1] = 0.0f;
        Gyro_offset[2] = 0.0f;

        // 2. 循环采样
        for (int i = 0; i < calib_samples; i++)
        {
            ReadGyroData(&temp_gyro); // 这里读取的是原始值（因为Offset已清零）
            gyro_sum[0] += temp_gyro.x;
            gyro_sum[1] += temp_gyro.y;
            gyro_sum[2] += temp_gyro.z;
            
            tx_thread_sleep(1); // 等待一个 ThreadX tick；实际耗时由 tick 频率决定。
        }

        // 3. 计算平均值作为零偏
        Gyro_offset[0] = gyro_sum[0] / calib_samples;
        Gyro_offset[1] = gyro_sum[1] / calib_samples;
        Gyro_offset[2] = gyro_sum[2] / calib_samples;
        
        // 如果零偏过大（例如超过 0.1 rad/s），可能是在运动中标定的，应报错或丢弃
        if (fabs(Gyro_offset[0]) > 0.1f || fabs(Gyro_offset[1]) > 0.1f || fabs(Gyro_offset[2]) > 0.1f)
        {
            self_test.CALIBRATE_ERR = true;
            // 恢复默认值或保留上次值
            Gyro_offset[0] = BMI088_GYRO_PRE_CALI_OFFSET_X; 
            Gyro_offset[1] = BMI088_GYRO_PRE_CALI_OFFSET_Y;
            Gyro_offset[2] = BMI088_GYRO_PRE_CALI_OFFSET_Z;
        }
        else
        {
            self_test.CALIBRATE_ERR = false;
        }
    }

    void cBMI088::TemperatureControl(float target_temp)
    {
        // TODO: 温度读取完成后，用 TempPid 计算加热占空比并限制到 [0, 1]。
        // 未完成前保持加热关闭。
        (void)target_temp;
        (void)PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, 0.0f, HEATING_RESISTANCE_CHANNEL);
    }

    void cBMI088::VerifyAccChipID()
    {
        uint8_t chip_id = 0U;

        // 加速度计上电后默认使用 I2C，首次 SPI 读取仅用于切换接口。
        (void)ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, &chip_id, 1U);
        const bool read_ok = ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, &chip_id, 1U);

        self_test.ACC_CHIP_ID_ERR = !read_ok || chip_id != ACC_CHIP_ID_VAL;
        self_test.INIT_ERR = self_test.ACC_CHIP_ID_ERR || self_test.GYRO_CHIP_ID_ERR;
    }

    void cBMI088::VerifyGyroChipID()
    {
        uint8_t chip_id = 0U;
        const bool read_ok = ReadReg(BMI088_CS_GYRO, GYRO_CHIP_ID_ADDR, &chip_id, 1U);

        self_test.GYRO_CHIP_ID_ERR = !read_ok || chip_id != GYRO_CHIP_ID_VAL;
        self_test.INIT_ERR = self_test.ACC_CHIP_ID_ERR || self_test.GYRO_CHIP_ID_ERR;
    }

    void cBMI088::VerifyAccData()
    {
        ReadAccData(&acc_data);
    }

    void cBMI088::VerifyGyroData()
    {
        ReadGyroData(&gyro_data);
    }

    bool cBMI088::WriteReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        if (data == nullptr || len == 0U)
        {
            return false;
        }

        ChipSelect chip_select{};
        if (!GetChipSelect(cs, &chip_select))
        {
            return false;
        }

        uint8_t command = addr & BMI088_SPI_WRITE_CODE;
        HAL_GPIO_WritePin(chip_select.port, chip_select.pin, GPIO_PIN_RESET); // 拉低片选，开始接受数据

        HAL_StatusTypeDef status = HAL_SPI_Transmit(
            &hspi1,
            &command,
            1U,
            BMI088_SPI_TIMEOUT_MS);
        if (status == HAL_OK)
        {
            status = HAL_SPI_Transmit(
                &hspi1,
                data,
                len,
                BMI088_SPI_TIMEOUT_MS);
        }

        HAL_GPIO_WritePin(chip_select.port, chip_select.pin, GPIO_PIN_SET); // 恢复片选，结束传输
        return status == HAL_OK;
    }

    bool cBMI088::ReadReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        if (data == nullptr || len == 0U)
        {
            return false;
        }

        for (uint8_t i = 0U; i < len; ++i)
        {
            data[i] = 0U;
        }

        ChipSelect chip_select{};
        if (!GetChipSelect(cs, &chip_select))
        {
            return false;
        }

        uint8_t command = addr | BMI088_SPI_READ_CODE;
        HAL_GPIO_WritePin(chip_select.port, chip_select.pin, GPIO_PIN_RESET);

        HAL_StatusTypeDef status = HAL_SPI_Transmit(
            &hspi1,
            &command,
            1U,
            BMI088_SPI_TIMEOUT_MS);

        if (status == HAL_OK && cs == BMI088_CS_ACC)
        {
            uint8_t dummy = 0U;
            status = HAL_SPI_Receive(
                &hspi1,
                &dummy,
                1U,
                BMI088_SPI_TIMEOUT_MS);
        }

        if (status == HAL_OK)
        {
            status = HAL_SPI_Receive(
                &hspi1,
                data,
                len,
                BMI088_SPI_TIMEOUT_MS);
        }

        HAL_GPIO_WritePin(chip_select.port, chip_select.pin, GPIO_PIN_SET);

        if (status != HAL_OK)
        {
            for (uint8_t i = 0U; i < len; ++i)
            {
                data[i] = 0U;
            }
            return false;
        }

        return true;
    }

    void cBMI088::Config()
    {
        tx_thread_sleep(10); //< 等待系统稳定

        /*-------------------------------------加速度计初始化-------------------------------------*/

        // 加速度计上电后默认处于 I2C 模式，先进行一次无效读取，通过片选上升沿切换到 SPI 模式。
        uint8_t dummy_data = 0U;
        (void)ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, &dummy_data, 1U);

        //< 先软重启，清空所有寄存器
        uint8_t pTxData;
        pTxData = ACC_SOFTRESET_VAL;
        WriteReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        // 软复位会使加速度计恢复默认接口，必须再次通过无效读取切换到 SPI 模式。
        dummy_data = 0U;
        (void)ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, &dummy_data, 1U);

        //< 打开加速度计电源
        pTxData = ACC_PWR_CTRL_ON;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(150);

        //< 加速度计变成正常模式
        pTxData = ACC_PWR_CONF_ACT;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(10); //

        //< 测量范围
        pTxData = ACC_RANGE_6G;
        WriteReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0xAB;
        WriteReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x08;
        WriteReg(BMI088_CS_ACC, INT1_IO_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x04;
        WriteReg(BMI088_CS_ACC, INT_MAP_DATA_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        /*-------------------------------------陀螺仪初始化-------------------------------------*/
        //< 先软重启，清空所有寄存器
        pTxData = GYRO_SOFTRESET_VAL;
        WriteReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        pTxData = GYRO_RANGE_2000_DEG_S;
        WriteReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_ODR_2000Hz_BANDWIDTH_230Hz | GYRO_LPM1_SUS;
        WriteReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_LPM1_NOR;
        WriteReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_DRDY_ON;
        WriteReg(BMI088_CS_GYRO, GYRO_INT_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x00;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x01;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_MAP_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms
    }


    void cBMI088::ReadAccData(acc_data_t *data)
    {
        if (data == nullptr)
        {
            self_test.ACC_DATA_ERR = true;
            return;
        }

        uint8_t raw_data[ACC_XYZ_LEN]{};
        if (!ReadReg(BMI088_CS_ACC, ACC_X_LSB_ADDR, raw_data, ACC_XYZ_LEN))
        {
            *data = {};
            self_test.ACC_DATA_ERR = true;
            return;
        }

        data->x = static_cast<float>(DecodeInt16(&raw_data[0])) * IMU_ACCEL_6G_SEN;
        data->y = static_cast<float>(DecodeInt16(&raw_data[2])) * IMU_ACCEL_6G_SEN;
        data->z = static_cast<float>(DecodeInt16(&raw_data[4])) * IMU_ACCEL_6G_SEN;
        self_test.ACC_DATA_ERR = false;
    }

    void cBMI088::ReadGyroData(gyro_data_t *data)
    {
        if (data == nullptr)
        {
            self_test.GYRO_DATA_ERR = true;
            return;
        }

        uint8_t raw_data[GYRO_XYZ_LEN]{};
        if (!ReadReg(BMI088_CS_GYRO, GYRO_RATE_X_LSB_ADDR, raw_data, GYRO_XYZ_LEN))
        {
            *data = {};
            self_test.GYRO_DATA_ERR = true;
            return;
        }

        data->x = static_cast<float>(DecodeInt16(&raw_data[0])) * IMU_GYRO_2000_SEN - Gyro_offset[0];
        data->y = static_cast<float>(DecodeInt16(&raw_data[2])) * IMU_GYRO_2000_SEN - Gyro_offset[1];
        data->z = static_cast<float>(DecodeInt16(&raw_data[4])) * IMU_GYRO_2000_SEN - Gyro_offset[2];
        self_test.GYRO_DATA_ERR = false;
    }

    void cBMI088::ReadAccTemperature(float *temp)
    {
        if (temp == nullptr)
        {
            self_test.ACC_DATA_ERR = true;
            return;
        }

        uint8_t raw_data[TEMP_LEN]{};
        if (!ReadReg(BMI088_CS_ACC, TEMP_MSB_ADDR, raw_data, TEMP_LEN) || raw_data[0] == 0x80U)
        {
            *temp = 0.0f;
            self_test.ACC_DATA_ERR = true;
            return;
        }

        int16_t raw_temperature = static_cast<int16_t>(
            (static_cast<uint16_t>(raw_data[0]) << 3U) |
            (static_cast<uint16_t>(raw_data[1]) >> 5U));
        if (raw_temperature > 1023)
        {
            raw_temperature -= 2048;
        }

        *temp = static_cast<float>(raw_temperature) * TEMP_UNIT + TEMP_BIAS;
    }

}

