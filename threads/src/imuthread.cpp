#include "threads.hpp"

namespace
{
constexpr ULONG IMU_THREAD_STACK_SIZE = 512U;
constexpr UINT IMU_THREAD_PRIORITY = 6U;
constexpr ULONG IMU_PERIOD_TICKS = TX_TIMER_TICKS_PER_SECOND / 100U;
constexpr ULONG IMU_HEARTBEAT_PERIOD_TICKS = TX_TIMER_TICKS_PER_SECOND / 10U;

static_assert(IMU_PERIOD_TICKS > 0U, "IMU线程周期必须至少为一个ThreadX tick");
static_assert(IMU_HEARTBEAT_PERIOD_TICKS > 0U, "IMU心跳周期必须至少为一个ThreadX tick");

TX_THREAD imu_thread;
CHAR imu_thread_name[] = "imu";

void ImuThreadEntry(ULONG)
{
    g_bmi088.Config();
    g_bmi088.VerifyAccChipID();
    g_bmi088.VerifyGyroChipID();

    if (g_bmi088.self_test.INIT_ERR)
    {
        return;
    }

    ULONG last_heartbeat_tick = tx_time_get();
    (void)AliveThread_ReportImu();

    while (true)
    {
        g_bmi088.ReadAccData(&g_bmi088.acc_data);
        g_bmi088.ReadGyroData(&g_bmi088.gyro_data);
        g_bmi088.ReadAccTemperature(&g_bmi088.acc_data.temperature);

        const ULONG now = tx_time_get();
        if ((now - last_heartbeat_tick) >= IMU_HEARTBEAT_PERIOD_TICKS)
        {
            (void)AliveThread_ReportImu();
            last_heartbeat_tick = now;
        }

        tx_thread_sleep(IMU_PERIOD_TICKS);
    }
}
}

// IMU线程唯一写入该对象，其他线程可读取采集结果。
BMI088::cBMI088 g_bmi088;

extern "C" UINT ImuThread_Init(TX_BYTE_POOL *byte_pool)
{
    if (byte_pool == nullptr)
    {
        return TX_PTR_ERROR;
    }

    VOID *stack = nullptr;
    UINT status = tx_byte_allocate(
        byte_pool,
        &stack,
        IMU_THREAD_STACK_SIZE,
        TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    status = tx_thread_create(
        &imu_thread,
        imu_thread_name,
        ImuThreadEntry,
        0U,
        stack,
        IMU_THREAD_STACK_SIZE,
        IMU_THREAD_PRIORITY,
        IMU_THREAD_PRIORITY,
        TX_NO_TIME_SLICE,
        TX_AUTO_START);

    if (status != TX_SUCCESS)
    {
        tx_byte_release(stack);
    }

    return status;
}
