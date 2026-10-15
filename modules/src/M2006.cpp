#include "M2006.hpp"

M2006::M2006()
{
    gearBox = GearBox_M2006;
    maxCurrent = 10000U;

    // 配置了速度环和位置环初始参数
    speedPid = PID(300.0f, 5.0f, 0.0f,
                   static_cast<float>(maxCurrent), 3000.0f, PID_POSITION);
    positionPid = PID(5.0f, 0.0f, 0.0f,
                      10.0f, 0.0f, PID_POSITION);
}

void M2006::setOutput()
{
    if (MotorState != MOTOR_ONLINE)
    {
        currentSet = 0;
        return;
    }

    switch (controlMode)
    {
    case RELAX_MODE:
        currentSet = 0;
        speedPid.Clear();
        positionPid.Clear();
        return;

    case SPD_MODE:
        speedPid.ref = speedSet;
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();
        break;

    case POS_MODE:
        positionPid.ref = positionSet;
        positionPid.fdb = motorFeedback.positionFdb;
        positionPid.UpdateResult();

        speedPid.ref = positionPid.result;
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();
        break;

    default:
        currentSet = 0;
        speedPid.Clear();
        positionPid.Clear();
        return;
    }

    const float limited_current =
        Numeric::LimitABS(speedPid.result, static_cast<float>(maxCurrent));
    currentSet = static_cast<int16_t>(limited_current);
}

M2006::MotorStateTypedef M2006::AliveCheck()
{
    if (AliveFlag == Pre_AliveFlag)
    {
        MotorState = MOTOR_OFFLINE;
        currentSet = 0;
        speedPid.Clear();
        positionPid.Clear();
    }
    else
    {
        MotorState = MOTOR_ONLINE;
    }

    Pre_AliveFlag = AliveFlag;
    return MotorState;
}
