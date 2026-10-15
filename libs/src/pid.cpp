#include "pid.hpp"

#include <cmath>

PID::PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode)
    : mode(mode), kp(kp), ki(ki), kd(kd), maxOut(maxOut), maxIOut(maxIOut)
{
    Clear();
}

void PID::Tuning(float tuning_kp, float tuning_ki, float tuning_kd)
{
    kp = tuning_kp;
    ki = tuning_ki;
    kd = tuning_kd;
}

void PID::UpdateResult()
{
    err[2] = err[1];
    err[1] = err[0];
    err[0] = ref - fdb;

    const float outputLimit = std::fabs(maxOut);
    const float integralLimit = std::fabs(maxIOut);

    switch (mode)
    {
    case PID_POSITION:
        pResult = kp * err[0];
        iResult += ki * err[0];
        iResult = Numeric::LimitABS(iResult, integralLimit);
        dResult = kd * (err[0] - err[1]);
        result = pResult + iResult + dResult;
        break;

    case PID_DELTA:
        pResult = kp * (err[0] - err[1]);
        iResult = Numeric::LimitABS(ki * err[0], integralLimit);
        dResult = kd * (err[0] - 2.0f * err[1] + err[2]);
        result += pResult + iResult + dResult;
        break;

    default:
        pResult = 0.0f;
        iResult = 0.0f;
        dResult = 0.0f;
        result = 0.0f;
        return;
    }

    result = Numeric::LimitABS(result, outputLimit);
}

void PID::Clear()
{
    ref = fdb = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
}
