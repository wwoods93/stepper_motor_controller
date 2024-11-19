/***********************************************************************************************************************
 * stepper_motor_controller
 * main.cpp
 *
 * wilson
 * 11/12/24
 * 10:37 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"
/* third-party includes */

/* layer_0_hal includes */
#include "../layer_0/hal.h"
/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

/* main header */
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void hal_callback_tim_1_pulse_complete(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Start_IT(get_timer_1_handle(), TIM_CHANNEL_3);
}

static uint32_t count = 0U;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main()
{

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_I2C1_Init();
    MX_IWDG_Init();
    MX_SPI2_Init();
    MX_USART2_UART_Init();
    MX_WWDG_Init();
    MX_USART3_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    HAL_TIM_Base_Start(get_timer_2_handle());
    /* USER CODE BEGIN 2 */
    HAL_TIM_RegisterCallback(get_timer_1_handle(), HAL_TIM_PWM_PULSE_FINISHED_CB_ID, hal_callback_tim_1_pulse_complete);
    HAL_TIM_PWM_Start_IT(get_timer_1_handle(), TIM_CHANNEL_3);

    while (1)
    {
        if (get_timer_2_handle()->Instance->CNT - count > 50000U)
        {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_GPIO_TogglePin(STEPPER_0_DIR_GPIO_Port, STEPPER_0_DIR_Pin);
            count = get_timer_2_handle()->Instance->CNT;
        }
    }
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    HAL_TIM_IRQHandler(get_timer_1_handle());
}

void TIM1_CC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(get_timer_1_handle());
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    HAL_TIM_IRQHandler(get_timer_1_handle());
}
