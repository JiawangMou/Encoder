#include "nrf_protocol.h"

static ACTUATOR_STATUS actuator_control;
static MOTION_STATUS motion_control;

const ACTUATOR_STATUS* GetActuatorControl()
{
    return &actuator_control;
}

void SetActuatorControl(const ACTUATOR_STATUS* input)
{
    actuator_control = *input;
}

const MOTION_STATUS* GetMotionControl()
{
    return &motion_control;
}

void SetMotionControl(const MOTION_STATUS* input)
{
    motion_control = *input;
}
//end of file
