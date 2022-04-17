#include "realMain.h"
#include "main.h"
#include "motorControl.h"                   //NOTDONE needed?
#include "MPU6050.h"
#include "SBUS.h"
#include "PID.h"

uint8_t task;

void loop()
{
  if (TIM4->CNT >= (fastPPM_Pulselength - fastPPM_calcutationTime))
  {
    HAL_GPIO_TogglePin(ONBOARD_WRITE_3_GPIO_Port, ONBOARD_WRITE_3_Pin);
    updateMainMotorSpeed();   //40 us
    HAL_GPIO_TogglePin(ONBOARD_WRITE_3_GPIO_Port, ONBOARD_WRITE_3_Pin);

    switch (task)                       //execute the selected task
    {
    case 1:
      MPU6050_WaitForQuaternionSet();   //250 us if given 5000 us after FIFO reset
      MPU6050_ConvertToQuaternions();   //450 us
      MPU6050_resetFIFO();    //180 us
      get_XW_diffAngles();    //200 us
      update_PID(); 	  //18 us
      updateTailMotorSpeed();   //90 us
      update_FrameOriginQuaternion();   //90 us
      if (SBUSNewPackage == true) SBUS_postProcessing();    //30 us
      switchTuningMode();   //2 us
      if (SBUS_Channels[4] == 1000)  //if the set-PID-switch on the remote is on, read the values
      {
        getPIDValues();   //5 us
        getAngleOffset();   //4 us
      }
      getMainMotorOffset();   //65 us
      MainMotorDLPF();    //4 us
      break;
    case 2:
      task = 0;                 //reset to task 1  (keep in mind task ++; below)

      if (smoothMainMotorSpeed <= motorDeadzone)    //750 us
      {
        for (size_t i = 0; i < 1; i++)
        {
          unsigned char msg[300];
	        sprintf((char*)msg,"%f %f %f %f %f %f %f %f %f %f %f %f \r\n"   \
          , Pitch_PID_k[0] * 10                                           \
          , Pitch_PID_k[1] * 10                                           \
          , Pitch_PID_k[2] * 10                                           \
          , Roll_PID_k[0] * 10                                            \
          , Roll_PID_k[1] * 10                                            \
          , Roll_PID_k[2] * 10                                            \
          , Yaw_PID_k[0] * 10                                             \
          , Yaw_PID_k[1] * 10                                             \
          , Yaw_PID_k[2] * 10                                             \
          , mainMotorStartOffset                                          \
          , mainMotorSkewOffset                                           \
          , mainMotorMaxOffset);                                          \
	        uint8_t x = 0;
	        while (msg[x] != NULL)
	        {
	        	x++;
	        }
	        unsigned char msgTransmit[x];
	        for (size_t i = 0; i < x; i++)
	        {
	        	msgTransmit[i] = msg[i];
	        }
	        CDC_Transmit_FS((unsigned char*)msgTransmit, sizeof(msgTransmit));
        }
      }
      break;
    default:
      break;
    }
    task ++;

    while (TIM4->CNT > (fastPPM_Pulselength - fastPPM_calcutationTime));     //wait until TIM4 resets to 0
    while (TIM4->CNT < (fastPPM_Pulselength - fastPPM_calcutationTime));     //wait until fastPPM_Pulselength - fastPPM_calcutationTime has passed
  }
}
