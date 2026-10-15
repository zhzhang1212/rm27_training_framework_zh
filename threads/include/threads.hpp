#ifndef THREADS_HPP
#define THREADS_HPP

#include "tx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

UINT ControlThread_Init(TX_BYTE_POOL *byte_pool);

#ifdef __cplusplus
}
#endif

#endif // THREADS_HPP
