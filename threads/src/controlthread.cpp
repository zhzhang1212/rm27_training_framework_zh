#include "threads.hpp"

#include "DJIMotorHandler.hpp"
#include "M2006.hpp"
#include "bsp_can.hpp"

namespace
{
constexpr ULONG CONTROL_THREAD_STACK_SIZE = 768U; //栈大小
constexpr UINT CONTROL_THREAD_PRIORITY = 5U; //优先级，数字越小优先级越高
constexpr ULONG CONTROL_PERIOD_TICKS = 1U;
constexpr uint32_t ALIVE_CHECK_PERIODS = 10U;

TX_THREAD control_thread;
CHAR control_thread_name[] = "control";
M2006 motor;

void ControlThreadEntry(ULONG)
{
    auto *handler = DJIMotorHandler::Instance();
    uint32_t alive_counter = 0;

    while (true)
    {
        if (++alive_counter >= ALIVE_CHECK_PERIODS)
        {
            handler->AllMotorAliveCheck();
            alive_counter = 0;
        }

        motor.setOutput(); 
        handler->sendControlData(); // 发送控制数据到电机
        tx_thread_sleep(CONTROL_PERIOD_TICKS); // 休眠一段时间，控制周期为1ms
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

    CAN_Init();
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
