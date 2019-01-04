/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
//extern ADC_HandleTypeDef hadc2;
int LEDBlink=100;
int LEDCnt=-1;
int lap=1;
int hearts=8;
int num_obs=3;
int notEntered=1;
int currentTime = 0;
int lastCrash = -1;
int tim4Cnt = 0;
int PrevLeft=9;
int seconds=0,minutes=0;


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
	int delay=1000;
	display();
	clear();
	for(int i=0; i<20; i++)
		for(int j=0; j<4; j++){
			setCursor(i,j);
			write((0));
		}
	
	//step 2
	HAL_Delay(delay);
	setCursor(8,0);
	print("     ");
	setCursor(7,1);
	print("       ");
	setCursor(7,2);
	print("       ");
	setCursor(8,3);
	print("     ");
		
	//step 3
	HAL_Delay(delay);
	setCursor(2,0);
	print("               ");
	setCursor(1,1);
	print("                 ");
	setCursor(1,2);
	print("                 ");
	setCursor(2,3);
	print("               ");
		
	//print 3	
	HAL_Delay(delay);
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
	write((1));
	
	setCursor(7,2);
	write((2));
	setCursor(8,2);
	write((2));
	setCursor(9,2);
	write((2));
	setCursor(10,2);
	write((2));
	setCursor(11,2);
	write((1));
		
	setCursor(7,3);
	write((0));
	setCursor(8,3);
	write((0));
	setCursor(9,3);
	write((0));
	setCursor(10,3);
	write((0));
	
	
	//2
	HAL_Delay(delay);
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
	write((1));
	
	setCursor(7,2);
	write((2));
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
	HAL_Delay(delay);
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
	HAL_Delay(delay);
	notStarted=0;
}

void displayGameBackGround(){
	clear();
	int startRoad = 3;
	setCursor(startRoad,0);
	write((5));
	setCursor(startRoad-1,1);
	write((5));
	setCursor(startRoad-2,2);
	write((5));
	setCursor(startRoad-3,3);
	write((5));
	
	int endRoad = 16;
	setCursor(endRoad,0);
	write((6));
	setCursor(endRoad+1,1);
	write((6));
	setCursor(endRoad+2,2);
	write((6));
	setCursor(endRoad+3,3);
	write((6));
	
	
	setCursor(endRoad+1,0);
	write((7));
	setCursor(endRoad+2,0);
	print(":");
	
	notEntered=0;
}


int calculateScore(int lap,int num_obs, int currentTime, int lastCrash){
	return 100;
}

void printCar(){
	//Volume sensor
	int Vol = HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Start(&hadc3);
	//HAL_Delay(500);
	int Left=1;
	if(Vol/230>=1 && Vol/230<18){
		Left = Vol/230;
	}
	if(PrevLeft!=Left){
		setCursor(PrevLeft,3);
		print("  ");
	}
	PrevLeft = Left;
	if(!notEntered){
		setCursor(Left,3);
		write((4));
		setCursor(Left+1,3);
		write((3));
	}
//	char strVol[4];
//	setCursor(5,2);
//	sprintf(strVol,"%04d",Vol);
//	print(strVol);
}

void turnOnLEDs(){
	//LDR sensor

	int LEDLight = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Start(&hadc1);
	int ratio = 2;


	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,LEDLight/ratio);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,LEDLight/ratio);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,LEDLight/ratio);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,LEDLight/ratio);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,LEDLight/ratio);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,LEDLight/ratio);
	//HAL_Delay(500);
//	char str[4];
//	setCursor(11,2);
//	sprintf(str,"%04d",LEDLight);
//	print(str);
}
void displayGame(int lap,int score, int hearts){
	display();
	
	//print score
	char scoreStr[4];
	setCursor(0,0);
	sprintf(scoreStr,"%03d",score);	
	print(scoreStr);
	
	//print heart
	char heartsStr[1];
	setCursor(19,0);
	sprintf(heartsStr,"%d",hearts);	
	print(heartsStr);
	
	//hearts--;
	
	HAL_Delay(500);
	
}

//void generateObstacles(){
//	int rnd = HAL_ADC_GetValue(&hadc2);
//	HAL_ADC_Start(&hadc2);
//	rnd%=
////	char strVol[4];
////	setCursor(12,2);
////	sprintf(strVol,"%04d",rnd);
////	print(strVol);
//	
//}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
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
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
//	HAL_ADC_Start_IT(&hadc1);
//	HAL_ADC_Start_IT(&hadc3);
	
	
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
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
	
	turnOnLEDs();
	printCar();

	if(notStarted){
		beforeStart();
		//HAL_Delay(500);
	}
	
	else{   // The game started!
		//displayGame();
		if(notEntered){
			displayGameBackGround();
			
		}
		
		if(tim4Cnt%30==0){ //each 15 seconds
			int score = calculateScore(lap,num_obs,currentTime,lastCrash);
			displayGame(lap,score,hearts);
			//generateObstacles();
			
			
			lap++;
			num_obs++;
		}
		
		
		tim4Cnt++;
	}
	
	
	
	

  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */



/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
