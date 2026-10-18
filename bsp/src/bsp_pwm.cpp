//
// Created by cosmosmount on 2025/9/2.
//

#include "bsp_pwm.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace
{
bool IsConfiguredPwmTimer(const TIM_HandleTypeDef *htim)
{
    return htim != nullptr &&
           (htim->Instance == TIM1 || htim->Instance == TIM10);
}

bool IsConfiguredPwmChannel(const TIM_HandleTypeDef *htim, uint32_t channel)
{
    return IsConfiguredPwmTimer(htim) &&
           IS_TIM_CCX_INSTANCE(htim->Instance, channel);
}

uint32_t GetApb2TimerClock()
{
    RCC_ClkInitTypeDef clock_config{};
    uint32_t flash_latency = 0U;
    HAL_RCC_GetClockConfig(&clock_config, &flash_latency);

    uint32_t timer_clock = HAL_RCC_GetPCLK2Freq();
    if (clock_config.APB2CLKDivider != RCC_HCLK_DIV1)
    {
        timer_clock *= 2U;
    }

    return timer_clock;
}
}

bool PWM_Init(void)
{
    // CubeMX 已完成定时器和 GPIO 初始化；启动前先将输出置零，避免意外驱动舵机或加热电阻。
    if (!PWM_SetDutyRatio(&htim1, 0.0f, TIM_CHANNEL_1) ||
        !PWM_SetDutyRatio(&htim10, 0.0f, TIM_CHANNEL_1))
    {
        return false;
    }

    if (!PWM_Start(&htim1, TIM_CHANNEL_1))
    {
        return false;
    }

    if (!PWM_Start(&htim10, TIM_CHANNEL_1))
    {
        (void)PWM_Stop(&htim1, TIM_CHANNEL_1);
        return false;
    }

    return true;
}

bool PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (!IsConfiguredPwmChannel(htim, channel))
    {
        return false;
    }

    return HAL_TIM_PWM_Start(htim, channel) == HAL_OK;
}

bool PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (!IsConfiguredPwmChannel(htim, channel))
    {
        return false;
    }

    return HAL_TIM_PWM_Stop(htim, channel) == HAL_OK;
}

bool PWM_SetPeriod(TIM_HandleTypeDef *htim, float period_s)
{
    if (!IsConfiguredPwmTimer(htim) || !std::isfinite(period_s) || period_s <= 0.0f)
    {
        return false;
    }

    const uint32_t timer_clock = GetApb2TimerClock();
    const double counter_frequency =
        static_cast<double>(timer_clock) / (static_cast<double>(htim->Init.Prescaler) + 1.0);
    const double period_counts = counter_frequency * static_cast<double>(period_s);
    const uint64_t max_counts = IS_TIM_32B_COUNTER_INSTANCE(htim->Instance)
                                    ? static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) + 1ULL
                                    : static_cast<uint64_t>(std::numeric_limits<uint16_t>::max()) + 1ULL; // 计算最大计数值，考虑到计数器从 0 开始计数，因此最大计数值为最大可表示值 + 1。

    if (period_counts < 1.0 || period_counts > static_cast<double>(max_counts))
    {
        return false;
    }

    const uint32_t reload = static_cast<uint32_t>(period_counts + 0.5) - 1U;
    htim->Init.Period = reload;
    __HAL_TIM_SET_AUTORELOAD(htim, reload);
    __HAL_TIM_SET_COUNTER(htim, 0U);
    return HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE) == HAL_OK;
}

bool PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float duty_ratio, uint32_t channel)
{
    if (!IsConfiguredPwmChannel(htim, channel) ||
        !std::isfinite(duty_ratio) || duty_ratio < 0.0f || duty_ratio > 1.0f)
    {
        return false;
    }

    const uint64_t period_counts = static_cast<uint64_t>(__HAL_TIM_GET_AUTORELOAD(htim)) + 1ULL;
    uint64_t compare = static_cast<uint64_t>(
        static_cast<double>(period_counts) * static_cast<double>(duty_ratio) + 0.5);
    const uint64_t max_compare = IS_TIM_32B_COUNTER_INSTANCE(htim->Instance)
                                     ? std::numeric_limits<uint32_t>::max()
                                     : std::numeric_limits<uint16_t>::max();
    if (compare > max_compare)
    {
        compare = max_compare;
    }

    __HAL_TIM_SET_COMPARE(htim, channel, static_cast<uint32_t>(compare));
    return true;
}
