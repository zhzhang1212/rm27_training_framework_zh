//
// Created by cosmosmount on 2025/9/2.
//

#ifndef RM26_H7_BSP_PWM_HPP
#define RM26_H7_BSP_PWM_HPP

#include "tim.h"

/**
 * @brief 将已由 CubeMX 初始化的 PWM 通道置零并启动
 * @return 两个通道均成功启动时返回 true
 */
bool PWM_Init(void);

/**
 * @brief 启动指定 PWM 通道
 * @return 启动成功时返回 true
 */
bool PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 停止指定 PWM 通道
 * @return 停止成功时返回 true
 */
bool PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 设置 PWM 周期
 * @param htim 定时器句柄
 * @param period_s 周期，单位为秒
 * @return 参数有效且周期设置成功时返回 true
 */
bool PWM_SetPeriod(TIM_HandleTypeDef *htim, float period_s);

/**
 * @brief 设置 PWM 占空比
 * @param htim 定时器句柄
 * @param duty_ratio 占空比，范围为 [0, 1]
 * @param channel 定时器通道
 * @return 参数有效且比较值设置成功时返回 true
 */
bool PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float duty_ratio, uint32_t channel);

#endif //RM26_H7_BSP_PWM_HPP
