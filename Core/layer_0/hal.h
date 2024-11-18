/***********************************************************************************************************************
 * stepper_motor_controller
 * hal.h
 *
 * wilson
 * 11/12/24
 * 10:47 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef STEPPER_MOTOR_CONTROLLER_HAL_H
#define STEPPER_MOTOR_CONTROLLER_HAL_H

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
/* third-party includes */

/* layer_0_hal includes */

/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define STEPPER_1_PUL_Pin GPIO_PIN_6
#define STEPPER_1_PUL_GPIO_Port GPIOA
#define STEPPER_1_DIR_Pin GPIO_PIN_7
#define STEPPER_1_DIR_GPIO_Port GPIOA
#define STEPPER_0_DIR_Pin GPIO_PIN_6
#define STEPPER_0_DIR_GPIO_Port GPIOC
#define STEPPER_1_ENA_Pin GPIO_PIN_7
#define STEPPER_1_ENA_GPIO_Port GPIOC
#define STEPPER_0_PUL_Pin GPIO_PIN_8
#define STEPPER_0_PUL_GPIO_Port GPIOC
#define STEPPER_0_ALM_Pin GPIO_PIN_9
#define STEPPER_0_ALM_GPIO_Port GPIOC
#define STEPPER_1_ALM_Pin GPIO_PIN_9
#define STEPPER_1_ALM_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define STEPPER_0_POWER_RELAY_Pin GPIO_PIN_10
#define STEPPER_0_POWER_RELAY_GPIO_Port GPIOC
#define STEPPER_1_POWER_RELAY_Pin GPIO_PIN_12
#define STEPPER_1_POWER_RELAY_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB


TIM_HandleTypeDef* get_timer_1_handle();
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void MX_TIM1_Init();
void MX_TIM2_Init();
void Error_Handler();
void SystemClock_Config();
void MX_GPIO_Init();
void MX_CAN1_Init();
void MX_I2C1_Init();
void MX_IWDG_Init();
void MX_SPI2_Init();
void MX_USART2_UART_Init();
void MX_WWDG_Init();
void MX_USART3_UART_Init();
void hal_tim_msp_post_init(TIM_HandleTypeDef* htim);


#endif //STEPPER_MOTOR_CONTROLLER_HAL_H
