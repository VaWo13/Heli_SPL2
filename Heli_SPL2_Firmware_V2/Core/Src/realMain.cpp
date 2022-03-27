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
    switch (task)                       //disable the SBUS pin interrupt for the selected tasks
    {
    case 5:
      HAL_NVIC_DisableIRQ(EXTI0_IRQn);
      break;
    default:
      break;
    }
    updateMainMotorSpeed();
    //PPM only updates the next cycle
    switch (task)                       //execute the selected task
    {
    case 1:
      MPU6050_WaitForQuaternionSet();   //500 us if given 5000 us after FIFO reset
      MPU6050_ConvertToQuaternions();   //650 us
      HAL_NVIC_EnableIRQ(EXTI0_IRQn);
      get_XW_diffAngles();    //1100 us
      break;
    case 2:
      update_PID(); 	  //140 us
      updateTailMotorSpeed();   //8 us
      update_FrameOriginQuaternion();   //600 us
      if (SBUSNewPackage == true) SBUS_postProcessing();    //800 us
      if (SBUS_Channels[4] >= 990)  //if the set-PID-switch on the remote is on, read the values
      {
        getPIDValues();   //30 us
        getAngleOffset();   //450us
      }
      switchTuningMode();   //12 us
      break;
    case 9:
      MPU6050_resetFIFO();    //300 us
      break;
    case 10:
      task = 0;                 //reset to task 1  (keep in mind task ++; below)

      if (SBUS_Channels[2] <= motorDeadzone)
      {
        for (size_t i = 0; i < 1; i++)
        {
          unsigned char msg[300];
	        sprintf((char*)msg,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %hd \r\n"                                                                                                        \
          , LoopXWQuaternion[0]                                                                                                                                                                      \
          , LoopXWQuaternion[1]                                                                                                                                                                      \
          , LoopXWQuaternion[2]                                                                                                                                                                      \
          , LoopXWQuaternion[3]                                                                                                                                                                      \
          , PID_Pitch_xw_diff                                                                                                                                                                        \
          , PID_Roll_xw_diff                                                                                                                                                                         \
          , PID_Yaw_xw_diff                                                                                                                                                                          \
          , PID_Pitch_y                                                                                                                                                                              \
          , PID_Roll_y                                                                                                                                                                               \
          , PID_Yaw_y                                                                                                                                                                                \
          , Pitch_PID_k[0] * 100                                                                                                                                                                     \
          , Pitch_PID_k[1] * 100                                                                                                                                                                     \
          , Pitch_PID_k[2] * 100                                                                                                                                                                     \
          , Roll_PID_k[0] * 100                                                                                                                                                                      \
          , Roll_PID_k[1] * 100                                                                                                                                                                      \
          , Roll_PID_k[2] * 100                                                                                                                                                                      \
          , Yaw_PID_k[0] * 100                                                                                                                                                                       \
          , Yaw_PID_k[1] * 100                                                                                                                                                                       \
          , Yaw_PID_k[2] * 100                                                                                                                                                                       \
          , ((((((float)adcValueChannel12 - hall2_center) * hall2_scaler) * cos_OffsetAngle) - ((((float)adcValueChannel11 - hall1_center) * hall1_scaler) * sin_OffsetAngle)) * (PID_Pitch_y * 1))  \
          , ((((((float)adcValueChannel11 - hall1_center) * hall1_scaler) * cos_OffsetAngle) + ((((float)adcValueChannel12 - hall2_center) * hall2_scaler) * sin_OffsetAngle)) * (PID_Roll_y  * 1))  \
          , (float)SBUS_Channels[5]                                                                                                                                                                  \
          , mainMotorAngleOffset);                                                                                                                                                                   \
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