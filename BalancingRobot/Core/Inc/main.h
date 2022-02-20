/************************************************************************
 * Author  :Ahmed Elhamy
 * Date    :18/2/2022
 * Version :v_01
 ***********************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include  "MPU6050.h"
#include  "PID.h"
#include   <stdio.h>
/* USER CODE END Includes */

/* USER CODE BEGIN EM */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_TIM3_Init(void);
void MX_USART2_UART_Init(void);
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Motor1EN_Pin GPIO_PIN_0
#define Motor1EN_GPIO_Port GPIOA
#define Motor2EN_Pin GPIO_PIN_1
#define Motor2EN_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Motor2PWM_Pin GPIO_PIN_7
#define Motor2PWM_GPIO_Port GPIOA
#define Motor2Dir2_Pin GPIO_PIN_7
#define Motor2Dir2_GPIO_Port GPIOC
#define Motor1Dir1_Pin GPIO_PIN_8
#define Motor1Dir1_GPIO_Port GPIOA
#define Motor1Dir2_Pin GPIO_PIN_9
#define Motor1Dir2_GPIO_Port GPIOA
#define Motor2Dir1_Pin GPIO_PIN_10
#define Motor2Dir1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Motor1PWM_Pin GPIO_PIN_4
#define Motor1PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
