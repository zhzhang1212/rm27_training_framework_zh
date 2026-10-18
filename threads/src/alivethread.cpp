#include "threads.hpp"

#include "LED.hpp"

namespace
{
constexpr ULONG ALIVE_THREAD_STACK_SIZE = 512U;
constexpr UINT ALIVE_THREAD_PRIORITY = 10U;
constexpr ULONG ALIVE_MONITOR_PERIOD_TICKS = TX_TIMER_TICKS_PER_SECOND / 2U;

static_assert(ALIVE_MONITOR_PERIOD_TICKS > 0U, "alive监控周期必须至少为一个ThreadX tick");

TX_THREAD alive_thread;
CHAR alive_thread_name[] = "alive";
TX_SEMAPHORE initialization_semaphore;
TX_SEMAPHORE imu_heartbeat_semaphore;
TX_SEMAPHORE control_heartbeat_semaphore;
CHAR initialization_semaphore_name[] = "initialization";
CHAR imu_heartbeat_semaphore_name[] = "imu heartbeat";
CHAR control_heartbeat_semaphore_name[] = "control heartbeat";

bool ConsumeHeartbeat(TX_SEMAPHORE *semaphore)
{
    bool received = false;
    while (tx_semaphore_get(semaphore, TX_NO_WAIT) == TX_SUCCESS)
    {
        received = true;
    }
    return received;
}

// 灯效：初始化或IMU异常为红，control异常为蓝，两者异常为红蓝，全正常为绿；每500ms亮灭交替。
void SetStatusLed(bool initialization_ok, bool imu_alive, bool control_alive, bool led_on)
{
    if (!led_on)
    {
        LED::AllOff();
        return;
    }

    if (!initialization_ok)
    {
        LED::SetRGB(true, false, false);
        return;
    }

    LED::SetRGB(!imu_alive,
                imu_alive && control_alive,
                !control_alive);
}

void AliveThreadEntry(ULONG)
{
    LED::Init();
    bool initialization_ok = false;
    bool led_on = false;

    while (true)
    {
        tx_thread_sleep(ALIVE_MONITOR_PERIOD_TICKS);

        if (!initialization_ok)
        {
            initialization_ok = ConsumeHeartbeat(&initialization_semaphore);
        }

        const bool imu_alive = ConsumeHeartbeat(&imu_heartbeat_semaphore);
        const bool control_alive = ConsumeHeartbeat(&control_heartbeat_semaphore);

        led_on = !led_on;
        SetStatusLed(initialization_ok, imu_alive, control_alive, led_on);
    }
}
}

extern "C" UINT AliveThread_Init(TX_BYTE_POOL *byte_pool)
{
    if (byte_pool == nullptr)
    {
        return TX_PTR_ERROR;
    }

    VOID *stack = nullptr;
    UINT status = tx_byte_allocate(
        byte_pool,
        &stack,
        ALIVE_THREAD_STACK_SIZE,
        TX_NO_WAIT); // 分配线程堆栈内存
    if (status != TX_SUCCESS)
    {
        return status;
    }

    status = tx_semaphore_create(
        &initialization_semaphore,
        initialization_semaphore_name,
        0U);
    if (status != TX_SUCCESS)
    {
        tx_byte_release(stack);
        return status;
    }

    status = tx_semaphore_create(
        &imu_heartbeat_semaphore,
        imu_heartbeat_semaphore_name,
        0U);
    if (status != TX_SUCCESS)
    {
        tx_semaphore_delete(&initialization_semaphore);
        tx_byte_release(stack);
        return status;
    }

    status = tx_semaphore_create(
        &control_heartbeat_semaphore,
        control_heartbeat_semaphore_name,
        0U);
    if (status != TX_SUCCESS)
    {
        tx_semaphore_delete(&imu_heartbeat_semaphore);
        tx_semaphore_delete(&initialization_semaphore);
        tx_byte_release(stack);
        return status;
    }

    status = tx_thread_create(
        &alive_thread,
        alive_thread_name,
        AliveThreadEntry,
        0U,
        stack,
        ALIVE_THREAD_STACK_SIZE,
        ALIVE_THREAD_PRIORITY,
        ALIVE_THREAD_PRIORITY,
        TX_NO_TIME_SLICE,
        TX_AUTO_START); // 创建线程并自动启动

    if (status != TX_SUCCESS)
    {
        tx_semaphore_delete(&control_heartbeat_semaphore);
        tx_semaphore_delete(&imu_heartbeat_semaphore);
        tx_semaphore_delete(&initialization_semaphore);
        tx_byte_release(stack);
    }

    return status;
}

extern "C" UINT AliveThread_ReportInitialization(void)
{
    return tx_semaphore_put(&initialization_semaphore);
}

extern "C" UINT AliveThread_ReportImu(void)
{
    return tx_semaphore_put(&imu_heartbeat_semaphore);
}

extern "C" UINT AliveThread_ReportControl(void)
{
    return tx_semaphore_put(&control_heartbeat_semaphore);
}
