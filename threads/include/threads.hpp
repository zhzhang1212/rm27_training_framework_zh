#ifndef THREADS_HPP
#define THREADS_HPP

#include "tx_api.h"

#ifdef __cplusplus
#include "BMI088.hpp"

extern BMI088::cBMI088 g_bmi088;

extern "C" {
#endif

UINT AliveThread_Init(TX_BYTE_POOL *byte_pool);
UINT ImuThread_Init(TX_BYTE_POOL *byte_pool);
UINT ControlThread_Init(TX_BYTE_POOL *byte_pool);

#ifdef __cplusplus
}
#endif

#endif // THREADS_HPP
