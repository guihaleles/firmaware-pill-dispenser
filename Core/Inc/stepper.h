/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported constants --------------------------------------------------------*/
#define stepsperrev 4096
#define clockwise 0
#define anticlockwise 1
/* Exported functions ------------------------------------------------------- */
void stepper_set_rpm (int rpm);
void stepper_wave_drive (int step);
void stepper_step_angle (float angle, int direction, int rpm);
int getSlotByAngle(int angle);
int getFinalAngleBySlot(int slot) ;
int getInitialAngleBySlot(int slot) ;


#endif /* __EEPROM_H */
