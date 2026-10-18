#include "threads.hpp"

#include "DJIMotorHandler.hpp"
#include "M2006.hpp"
#include "bsp_can.hpp"

namespace
{
constexpr ULONG CONTROL_THREAD_STACK_SIZE = 768U;
constexpr UINT CONTROL_THREAD_PRIORITY = 5U;
constexpr ULONG CONTROL_PERIOD_TICKS = TX_TIMER_TICKS_PER_SECOND / 1000U;
constexpr ULONG CONTROL_HEARTBEAT_PERIOD_TICKS = TX_TIMER_TICKS_PER_SECOND / 10U;
constexpr uint32_t ALIVE_CHECK_PERIODS = 10U;

static_assert(CONTROL_PERIOD_TICKS > 0U, "control线程周期必须至少为一个ThreadX tick");
static_assert(CONTROL_HEARTBEAT_PERIOD_TICKS > 0U, "control心跳周期必须至少为一个ThreadX tick");

TX_THREAD control_thread;
CHAR control_thread_name[] = "control";
M2006 motor;

void ControlThreadEntry(ULONG)
{
    ULONG last_heartbeat_tick = tx_time_get();
    uint32_t alive_counter = 0;
    (void)AliveThread_ReportControl();

    while (true)
    {
        auto *handler = DJIMotorHandler::Instance();
        if (++alive_counter >= ALIVE_CHECK_PERIODS)
        {
            handler->AllMotorAliveCheck();
            alive_counter = 0;
        }

        motor.setOutput();
        if (handler->sendControlData() != HAL_OK)
        {
            motor.controlMode = DJIMotor::RELAX_MODE;
            motor.currentSet = 0;
        }

        const ULONG now = tx_time_get();
        if ((now - last_heartbeat_tick) >= CONTROL_HEARTBEAT_PERIOD_TICKS)
        {
            (void)AliveThread_ReportControl();
            last_heartbeat_tick = now;
        }

        tx_thread_sleep(CONTROL_PERIOD_TICKS);
    }
}
}

extern "C" UINT ControlThread_Init(TX_BYTE_POOL *byte_pool)
{
    if (byte_pool == nullptr)
    {
        return TX_PTR_ERROR;
    }

    VOID *stack = nullptr;
    UINT status = tx_byte_allocate(
        byte_pool,
        &stack,
        CONTROL_THREAD_STACK_SIZE,
        TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    if (!CAN_Init())
    {
        (void)tx_byte_release(stack);
        return TX_NOT_AVAILABLE;
    }
    DJIMotorHandler::Instance()->registerMotor(&motor, &hcan1, 0x201U);

    status = tx_thread_create(
        &control_thread,
        control_thread_name,
        ControlThreadEntry,
        0U,
        stack,
        CONTROL_THREAD_STACK_SIZE,
        CONTROL_THREAD_PRIORITY,
        CONTROL_THREAD_PRIORITY,
        TX_NO_TIME_SLICE,
        TX_AUTO_START);

    if (status != TX_SUCCESS)
    {
        tx_byte_release(stack);
    }

    return status;
}
