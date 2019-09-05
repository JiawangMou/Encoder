#ifndef NRF_PROTOCOL_VARIABLE_H
#define NRF_PROTOCOL_VARIABLE_H
#include "nrf_protocol.h"

void SetActuatorControl(const ACTUATOR_STATUS* input);
void SetMotionControl(const MOTION_STATUS* input);

#endif
