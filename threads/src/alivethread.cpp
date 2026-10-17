#include "threads.hpp"

#include "LED.hpp"

namespace
{
constexpr ULONG ALIVE_THREAD_STACK_SIZE = 512U;
constexpr UINT ALIVE_THREAD_PRIORITY = 10U;
constexpr ULONG LED_STEP_TICKS = TX_TIMER_TICKS_PER_SECOND / 2U;

TX_THREAD alive_thread;
CHAR alive_thread_name[] = "alive";

void AliveThreadEntry(ULONG)
{
    LED::Init();

    // 最小存活指示；启用 IMU 和控制线程后再加入基于信号量的监控。
    while (true)
    {
        LED::Toggle(LED::Channel::Green); // 绿色 LED 指示线程存活
        tx_thread_sleep(LED_STEP_TICKS);
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
        tx_byte_release(stack);
    }

    return status;
}
