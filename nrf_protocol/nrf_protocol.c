#include "write_protocol_variable.h"
#include <stdint.h>
#include "nrf_protocol.h"


void Actuator_assignment(const u8 *rx_buff, ACTUATOR_STATUS *Actuator_status)
{
//    ACTUATOR_STATUS temp_basic_control;
//    temp_basic_control.lift  = *(uint16_t*)&data_address[0];
//    temp_basic_control.yaw   = *( int16_t*)&data_address[2];
//    temp_basic_control.pitch = *( int16_t*)&data_address[4];
//    temp_basic_control.roll  = *( int16_t*)&data_address[6];
//    SetActuatorControl(&temp_basic_control);
	
	Actuator_status->Fly_Pulse = *rx_buff;
	Actuator_status->Climb_Pulse = *(rx_buff+1);
	Actuator_status->Pitch_Pulse = *(rx_buff+2);
	Actuator_status->Roll_Pulse = *(rx_buff+3);
	Actuator_status->Yaw_Pulse = *(rx_buff+4);
}

void Motion_assignment(const u8 *rx_buff, MOTION_STATUS *Motion_status)
{
//    MOTION_STATUS temp_motion_control;
//    temp_motion_control.x   = *(int16_t*)&data_address[0];
//    temp_motion_control.y   = *(int16_t*)&data_address[2];
//    temp_motion_control.z   = *(int16_t*)&data_address[4];
//    temp_motion_control.orientation = *(int16_t*)&data_address[6];
//    SetMotionControl(&temp_motion_control);
	
	Motion_status->x = *rx_buff;
	Motion_status->y = *(rx_buff+1);
	Motion_status->z = *(rx_buff+2);
	Motion_status->orientation = *(rx_buff+3);
}

void PID_assignment(const u8 *rx_buff, PID_PARAS *PID_paras)
{
	PID_paras->PID_ID = *rx_buff;
	switch( PID_paras->PID_ID )
	{
		case 0x01:
		{
			PID_paras->PID_Yaw_para.Kp_Int = *(rx_buff+1);
			PID_paras->PID_Yaw_para.Ki_Int = *(rx_buff+2);
			PID_paras->PID_Yaw_para.Kd_Int = *(rx_buff+3);
			PID_paras->PID_Yaw_para.Kp_Ext = *(rx_buff+4);
			PID_paras->PID_Yaw_para.Ki_Ext = *(rx_buff+5);
			PID_paras->PID_Yaw_para.Kd_Ext = *(rx_buff+6);
			PID_paras->PID_Yaw_para.SetPoint_Ext= *(int16_t*)&rx_buff[7];
		};break;
		case 0x02:
		{
			PID_paras->PID_Pitch_para.Kp_Int = *(rx_buff+1);
			PID_paras->PID_Pitch_para.Ki_Int = *(rx_buff+2);
			PID_paras->PID_Pitch_para.Kd_Int = *(rx_buff+3);
			PID_paras->PID_Pitch_para.Kp_Ext = *(rx_buff+4);
			PID_paras->PID_Pitch_para.Ki_Ext = *(rx_buff+5);
			PID_paras->PID_Pitch_para.Kd_Ext = *(rx_buff+6);
			PID_paras->PID_Pitch_para.SetPoint_Ext= *(int16_t*)&rx_buff[7];
		};break;
		case 0x03:
		{
			PID_paras->PID_Roll_para.Kp_Int = *(rx_buff+1);
			PID_paras->PID_Roll_para.Ki_Int = *(rx_buff+2);
			PID_paras->PID_Roll_para.Kd_Int = *(rx_buff+3);
			PID_paras->PID_Roll_para.Kp_Ext = *(rx_buff+4);
			PID_paras->PID_Roll_para.Ki_Ext = *(rx_buff+5);
			PID_paras->PID_Roll_para.Kd_Ext = *(rx_buff+6);
			PID_paras->PID_Roll_para.SetPoint_Ext= *(int16_t*)&rx_buff[7];
		};break;
	}
}

u8 Command_dispatch(u8 *rx_buff, ACTUATOR_STATUS *Actuator_status, MOTION_STATUS *Motion_status, PID_PARAS *PID_paras)
{
	uint16_t Mode_ID = *rx_buff;
	switch(Mode_ID)													
	{
		case 0xa0:Actuator_assignment(rx_buff+1,Actuator_status);break;
		case 0xa1:Motion_assignment(rx_buff+1,Motion_status);break;
		case 0xa2:PID_assignment(rx_buff+1,PID_paras);break;
		default:return 0xaa;
	}
	
	return Mode_ID;

}

void Command_patch(u8* tx_buff, PID_PARAS *PID_paras, ACTUATOR_STATUS* Actuator_Status, IMUFusion* Attitude)
{
	*tx_buff     = Actuator_Status->Fly_Pulse;
	*(tx_buff+1) = Actuator_Status->Climb_Pulse;
	*(tx_buff+2) = Actuator_Status->Pitch_Pulse;
	*(tx_buff+3) = Actuator_Status->Roll_Pulse;
	*(tx_buff+4) = Actuator_Status->Yaw_Pulse;
	switch( PID_paras->PID_ID )
    {
		case 0x01:
		{
			*(tx_buff+5)  = PID_paras->PID_Yaw_para.Kp_Int;
			*(tx_buff+6)  = PID_paras->PID_Yaw_para.Ki_Int;
			*(tx_buff+7)  = PID_paras->PID_Yaw_para.Kd_Int;
			*(tx_buff+8)  = PID_paras->PID_Yaw_para.Kp_Ext;
			*(tx_buff+9)  = PID_paras->PID_Yaw_para.Ki_Ext;
			*(tx_buff+10) = PID_paras->PID_Yaw_para.Kd_Ext;
			*(tx_buff+11) = PID_paras->PID_Yaw_para.SetPoint_Ext;
		};break;
		case 0x02:
		{
			*(tx_buff+5)  = PID_paras->PID_Pitch_para.Kp_Int;
			*(tx_buff+6)  = PID_paras->PID_Pitch_para.Ki_Int;
			*(tx_buff+7)  = PID_paras->PID_Pitch_para.Kd_Int;
			*(tx_buff+8)  = PID_paras->PID_Pitch_para.Kp_Ext;
			*(tx_buff+9)  = PID_paras->PID_Pitch_para.Ki_Ext;
			*(tx_buff+10) = PID_paras->PID_Pitch_para.Kd_Ext;
			*(tx_buff+11) = PID_paras->PID_Pitch_para.SetPoint_Ext;
		};break;
		case 0x03:
		{
			*(tx_buff+5)  = PID_paras->PID_Roll_para.Kp_Int;
			*(tx_buff+6)  = PID_paras->PID_Roll_para.Ki_Int;
			*(tx_buff+7)  = PID_paras->PID_Roll_para.Kd_Int;
			*(tx_buff+8)  = PID_paras->PID_Roll_para.Kp_Ext;
			*(tx_buff+9)  = PID_paras->PID_Roll_para.Ki_Ext;
			*(tx_buff+10) = PID_paras->PID_Roll_para.Kd_Ext;
			*(tx_buff+11) = PID_paras->PID_Roll_para.SetPoint_Ext;
		};break;
    }
	*(float*)&tx_buff[12] = Attitude->ypr[0];
	*(float*)&tx_buff[15] = Attitude->ypr[1];
	*(float*)&tx_buff[18] = Attitude->ypr[2];
	

}
