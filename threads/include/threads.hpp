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

UINT AliveThread_ReportInitialization(void);
UINT AliveThread_ReportImu(void);
UINT AliveThread_ReportControl(void);

#ifdef __cplusplus
}
#endif

#endif // THREADS_HPP
