/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
#include "LiquidCrystal.h"
int notStarted=1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
int LEDBlink=100;
int LEDCnt=0;
int lap=0;
int chances=8;
int num_obs=2;



void beforeStart(){
	//Writing on LCD
	display();
	setCursor(5,0);
	print("DEATH RACE");
	setCursor(6,1);
	print("NASTARAN");
	setCursor(7,2);
	print("FATEME");		
	// End of writing on the LCD
	
	
	//Blinking LEDs
	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,LEDBlink);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,LEDBlink);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,LEDBlink);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,LEDBlink);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,LEDBlink);
	if(LEDBlink==100)
		LEDBlink=0;
	else LEDBlink = 100;
	
	//End Blinking LEDs
	
	HAL_Delay(500);
	noDisplay();
	HAL_Delay(500);
}

void ready2Start(){
	//fill the screen
	display();
	clear();
	for(int i=0; i<20; i++)
		for(int j=0; j<4; j++){
			setCursor(i,j);
			write((0));
		}
	
	//step 2
	HAL_Delay(500);
	setCursor(8,0);
	print("     ");
	setCursor(7,1);
	print("       ");
	setCursor(7,2);
	print("       ");
	setCursor(8,3);
	print("     ");
		
	//step 3
	HAL_Delay(500);
	setCursor(2,0);
	print("               ");
	setCursor(1,1);
	print("                 ");
	setCursor(1,2);
	print("                 ");
	setCursor(2,3);
	print("               ");
		
	//print 3	
	HAL_Delay(500);
	clear();
	setCursor(7,0);
	write((0));
	setCursor(8,0);
	write((0));
	setCursor(9,0);
	write((0));
	setCursor(10,0);
	write((0));
	
	setCursor(7,1);
	write((1));
	setCursor(8,1);
	write((1));
	setCursor(9,1);
	write((1));
	setCursor(10,1);
	write((1));
	setCursor(11,1);
	write((3));
	
	setCursor(7,2);
	write((2));
	setCursor(8,2);
	write((2));
	setCursor(9,2);
	write((2));
	setCursor(10,2);
	write((2));
	setCursor(11,2);
	write((3));
		
	setCursor(7,3);
	write((0));
	setCursor(8,3);
	write((0));
	setCursor(9,3);
	write((0));
	setCursor(10,3);
	write((0));
	
	
	//2
	HAL_Delay(500);
	clear();
	setCursor(7,0);
	write((0));
	setCursor(8,0);
	write((0));
	setCursor(9,0);
	write((0));
	setCursor(10,0);
	write((0));
	setCursor(11,0);
	write((1));
			
	setCursor(7,1);
	write((1));
	setCursor(8,1);
	write((1));
	setCursor(9,1);
	write((1));
	setCursor(10,1);
	write((1));
	setCursor(11,1);
	write((3));
	
	setCursor(7,2);
	write((4));
	setCursor(8,2);
	write((2));
	setCursor(9,2);
	write((2));
	setCursor(10,2);
	write((2));
	setCursor(11,2);
	write((2));
	
	setCursor(7,3);
	write((2));
	setCursor(8,3);
	write((0));
	setCursor(9,3);
	write((0));
	setCursor(10,3);
	write((0));
	setCursor(11,3);
	write((0));
	
	//1
	HAL_Delay(500);
	clear();
	setCursor(9,0);
	write((5));
	setCursor(10,0);
	write((0));
	setCursor(10,1);
	write((0));
	setCursor(10,2);
	write((0));
	setCursor(8,3);
	write((0));
	setCursor(9,3);
	write((0));
	setCursor(10,3);
	write((0));
	setCursor(11,3);
	write((0));
	setCursor(12,3);
	write((0));
	notStarted=0;
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

	ready2Start();


  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles ADC1 and ADC2 interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */


  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
//	int count = __HAL_TIM_GET_COUNTER(&htim3);
//	lap++;
	
	
  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	
	
	if(notStarted){
		beforeStart();
	}
	else{
		HAL_ADC_Start_IT(&hadc1);
	}
	
	
	

  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */



/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
