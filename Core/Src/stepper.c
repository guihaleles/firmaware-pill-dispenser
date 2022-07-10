#include "stepper.h"
#include "main.h"

void stepper_set_rpm (int rpm)  // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	HAL_Delay(60000*2/stepsperrev/rpm);
}

void stepper_wave_drive (int step)
{
	switch (step){

		case 0:
			  HAL_GPIO_WritePin(GPIOC, Motor1_Pin, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(GPIOC, Motor2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOC, Motor3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOC, Motor4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 1:
			  HAL_GPIO_WritePin(GPIOC, Motor1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOC, Motor2_Pin, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(GPIOC, Motor3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOC, Motor4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 2:
			  HAL_GPIO_WritePin(GPIOC, Motor1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOC, Motor2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOC, Motor3_Pin, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(GPIOC, Motor4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 3:
			  HAL_GPIO_WritePin(GPIOC, Motor1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOC, Motor2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOC, Motor3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOC, Motor4_Pin, GPIO_PIN_SET);   // IN4
			  break;

		}
}

int angles[8][2] = {{0,8},{10,32},{42,73},{82,115},{125,160},{180,224},{225,270},{270,315}};
// 4 ->120
// 3 ->78
//2-> 37

int getFinalAngleBySlot(int slot) 
{
  return angles[slot][1];
}

int getInitialAngleBySlot(int slot) 
{
  return angles[slot][0];
}


int getSlotByAngle(int angle)
{

  for(int i = 0; i < 8; i++)
  {
    if(angle>= getInitialAngleBySlot(i) && angle<getFinalAngleBySlot(i))
    {
      return i;
    }
  }

  return 0;

}







