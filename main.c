#include "main.h"


#define COLUMN(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Width))    //see font.h, for defining LINE(X)

//defining HAL
HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);


static void SystemClock_Config(void);
static void Error_Handler(void);

//configuring the external buttons
void ExtBtn1_Config(void);
void ExtBtn2_Config(void);
void ExtBtn3_Config(void);

//configuring the external interrupt lines
static void EXTILine0_Config(void);
static void EXTILine1_Config(void);
static void EXTILine2_Config(void);
static void EXTILine3_Config(void);

//configuring outputs from board
void In1_Config(void);
void In2_Config(void);
void In3_Config(void);
void In4_Config(void);

//TIM3 Configuration
TIM_HandleTypeDef    Tim3_Handle;
TIM_OC_InitTypeDef Tim3_OCInitStructure;
uint16_t Tim3_PrescalerValue;
__IO uint16_t Tim3_CCR; 
void TIM3_Config(void);
void TIM3_OC_Config(void);

//required variables
__IO uint16_t OC_Count = 0;
int speed = 4000;
int direction = 0;
int state = 0;
int check = 0;
int step = 0;


int main(void){
	
		/* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
		HAL_Init();
	
		HAL_InitTick(0x0000); // set systick's priority to the highest.
			
		//calling external button configurations
		ExtBtn1_Config();
		ExtBtn2_Config();
		ExtBtn3_Config();
	
		//calling the external interrupt line configurations
		EXTILine1_Config();
		EXTILine2_Config();
		EXTILine3_Config();
	
		//configure use button as exti mode. 
		BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);  
	
		//configuring board output confgurations
		In1_Config();
		In2_Config();
		In3_Config();
		In4_Config();
	
		//configuring Tim3
		TIM3_Config();
		Tim3_CCR = 500;
		//TIM3_OC_Config();
	
		 /* Configure the system clock to 72 MHz */
		SystemClock_Config();
		
		 //Configure LED3 and LED4 ======================================
		BSP_LED_Init(LED3);
		BSP_LED_Init(LED4);
	
		BSP_LCD_Init();
		//BSP_LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address);
		BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);   //LCD_FRAME_BUFFER, defined as 0xD0000000 in _discovery_lcd.h
															// the LayerIndex may be 0 and 1. if is 2, then the LCD is dark.
		//BSP_LCD_SelectLayer(uint32_t LayerIndex);
		BSP_LCD_SelectLayer(0);
		BSP_LCD_SetLayerVisible(0, ENABLE); 
		
		BSP_LCD_Clear(LCD_COLOR_WHITE);  //need this line, otherwise, the screen is dark	
		BSP_LCD_DisplayOn();
	 
		BSP_LCD_SetFont(&Font20);  //the default font,  LCD_DEFAULT_FONT, which is defined in _lcd.h, is Font24
	
		LCD_DisplayString(2, 3, (uint8_t *)"Lab");
	
		LCD_DisplayInt(2, 8, 5);
		
		while(1) {	

			
		} // end of while loop
	
}  //end of main


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 

* 					Oscillator											=HSE
	*    				HSE frequencey 										=8,000,000   (8MHz)
	*      ----However, if the project is created by uVision, the default HSE_VALUE is 25MHz. thereore, need to define HSE_VALUE
	*						PLL Source											=HSE
  *            PLL_M                          = 4
  *            PLL_N                          = 72
  *            PLL_P                          = 2
  *            PLL_Q                          = 3
  *        --->therefore, PLLCLK =8MHz X N/M/P=72MHz   
	*            System Clock source            = PLL (HSE)
  *        --> SYSCLK(Hz)                     = 72,000,000
  *            AHB Prescaler                  = 1
	*        --> HCLK(Hz)                       = 72 MHz
  *            APB1 Prescaler                 = 2
	*        --> PCLK1=36MHz,  -->since TIM2, TIM3, TIM4 TIM5...are on APB1, thiese TIMs CLK is 36X2=72MHz
							 	
  *            APB2 Prescaler                 = 1
	*        --> PCLK1=72MHz 

  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

// configure Timer3 base.
void  TIM3_Config(void)
{

		/* -----------------------------------------------------------------------
    In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM4CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2
      => TIM4CLK = HCLK = SystemCoreClock
    To get TIM4 counter clock at 500 KHz, the Prescaler is computed as following:
    Prescaler = (TIM4CLK / TIM4 counter clock) - 1
    Prescaler = (SystemCoreClock  /500 KHz) - 1
       
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
  
  /* -----------------------------------------------------------------------
    TIM4 Configuration: Output Compare Timing Mode:
                                               
    (if CCR1_Val=500, then every 1 ms second will have an interrupt. If count 500 times of interrupt, thta is  0.5 seconds.
		 ----------------------------------------------------------------------- */ 	

/* Compute the prescaler value to have TIM3 counter clock equal to 500 KHz */
  Tim3_PrescalerValue = (uint32_t) (SystemCoreClock  / 500000) - 1;
  
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
   
  /* Initialize TIM4 peripheral as follows:
       + Period = 65535
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
	
	Tim3_Handle.Init.Period = 65535;  //does no matter, so set it to max.
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  //if(HAL_TIM_Base_Init(&Tim4_Handle) != HAL_OK)
  //{
    /* Initialization Error */
  //  Error_Handler();
  //} 
}

//configure timer 3 output compare 
void  TIM3_OC_Config(void)
{
		Tim3_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim3_OCInitStructure.Pulse=500;    // 500, this means every 1/1000 second generate an inerrupt //500 
		Tim3_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim3_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_OC_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_1); //must add this line to make OC work.!!!
	
	   /* **********see the top part of the hal_tim.c**********
		++ HAL_TIM_OC_Init and HAL_TIM_OC_ConfigChannel: to use the Timer to generate an 
              Output Compare signal. 
			similar to PWD mode and Onepulse mode!!!
	
	*******************/
	
	 	HAL_TIM_OC_Start_IT(&Tim3_Handle, TIM_CHANNEL_1); //this function enable IT and enable the timer. so do not need
				//HAL_TIM_OC_Start() any more
				
		
}

void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		while (*ptr!=NULL)
    {
				BSP_LCD_DisplayChar(COLUMN(ColumnNumber),LINE(LineNumber), *ptr); //new version of this function need Xpos first. so COLUMN() is the first para.
				ColumnNumber++;
			 //to avoid wrapping on the same line and replacing chars 
				if ((ColumnNumber+1)*(((sFONT *)BSP_LCD_GetFont())->Width)>=BSP_LCD_GetXSize() ){
					ColumnNumber=0;
					LineNumber++;
				}
					
				ptr++;
		}
}

void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		sprintf(lcd_buffer,"%d",Number);
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		
		sprintf(lcd_buffer,"%.*f",DigitAfterDecimalPoint, Number);  //6 digits after decimal point, this is also the default setting for Keil uVision 4.74 environment.
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

/**
 * Use this function to configure the GPIO to handle input from
 * external pushbuttons and configure them so that you will handle
 * them through external interrupts.
 */
void ExtBtn1_Config(void)     // for GPIO C pin 1
// can only use PA0, PB0... to PA4, PB4 .... because only  only  EXTI0, ...EXTI4,on which the 
	//mentioned pins are mapped to, are connected INDIVIDUALLY to NVIC. the others are grouped! 
		//see stm32f4xx.h, there is EXTI0_IRQn...EXTI4_IRQn, EXTI15_10_IRQn defined
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING; //output_pp
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void ExtBtn2_Config(void){  //**********PD2.***********

	//
	GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
	
  // Enable and set EXTI Line0 Interrupt to the lowest priority 
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

void ExtBtn3_Config(void){  //**********PE3.***********

	//
	GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_3;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_3);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
	
  // Enable and set EXTI Line0 Interrupt to the lowest priority 
  HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

static void EXTILine1_Config(void)  //for STM32f429_DISCO board, can not use PA1, PB1 and PD1,---PC1 is OK!!!!
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

static void EXTILine2_Config(void)  //for STM32f429_DISCO board, can not use PA1, PB1 and PD1,---PC1 is OK!!!!
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

static void EXTILine3_Config(void)  //for STM32f429_DISCO board, can not use PA1, PB1 and PD1,---PC1 is OK!!!!
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_3;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void In1_Config(void){  //**********PA2.***********

	//
	GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP ;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
	
  // Enable and set EXTI Line0 Interrupt to the lowest priority 
//  HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void In2_Config(void){  //**********PG3.***********

	//
	GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP ;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_3;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_3);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
	
  // Enable and set EXTI Line0 Interrupt to the lowest priority 
//  HAL_NVIC_SetPriority(EXTI5_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(EXTI5_IRQn);
}

void In3_Config(void){  //**********PA4.***********

	//
	GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP ;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_4;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_4);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
	
  // Enable and set EXTI Line0 Interrupt to the lowest priority 
//  HAL_NVIC_SetPriority(EXTI5_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(EXTI5_IRQn);
}

void In4_Config(void){  //**********PB5.***********

	//
	GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP ;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_5;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_5);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
	
  // Enable and set EXTI Line0 Interrupt to the lowest priority 
//  HAL_NVIC_SetPriority(EXTI5_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(EXTI5_IRQn);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//controls half step or full step
		if(GPIO_Pin == KEY_BUTTON_PIN)  //GPIO_PIN_0
		{
			LCD_DisplayInt(8, 0, 0);
			
			if (step == 0) {
				step = 1;
			}
			else if (step == 1) {
				step = 0;
			}
		}
		
		//slowing down rotation speed
		if(GPIO_Pin == GPIO_PIN_1)			//GPIOC_PIN_1
		{
				
				LCD_DisplayInt(5, 0, 1);
			
			speed += 1000;
			
		}  //end of PIN_1
		
		//rotation direction
		if(GPIO_Pin == GPIO_PIN_2) {		//GPIOD_PIN_2
		
				//LCD_DisplayInt(6, 0, 2);
			
			if (direction == 0) {
				direction = 1;
				LCD_DisplayInt(6, 0, 1);
			}
			
			else if (direction == 1) {
				direction = 0;
				LCD_DisplayInt(6, 0, 0);
			}
			
		} //end of if PIN_2	
		
		//speeding up rotation speed
		if(GPIO_Pin == GPIO_PIN_3)			//GPIOC_PIN_3
		{
						
				LCD_DisplayInt(7, 0, 3);
				
			if (speed >= 1000) {
				speed -= 1000;
			}
			
		} //end of if PIN_3
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
	//implementing full step and clockwise
	if ((*htim).Instance == TIM3 && direction == 0 && step == 0) {
		//using output compare counter to control the speed
		if (OC_Count < speed) {
		
			//checking the state to turn the motor
			if (state == 0) {
//				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET); //A2 on
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); //red/A1 off
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //red/A1 off
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); //B1 on
//				OC_Count = OC_Count + 1;
					BSP_LED_On(LED3);

				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET); //A2 on
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); //red/A1 off
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); //B1 on
//			
			OC_Count = OC_Count + 1;
				
				
			}
			else if (state == 1) {
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); //red/A1 off
//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); //B1 off
//				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET); //A2 on
//				OC_Count = OC_Count + 1;
					BSP_LED_On(LED4);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); //red/A1 off
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //red/A1 off
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); //B1 off
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET); //A2 on
				
					OC_Count = OC_Count + 1;
			}
			
		}
		else {
			if (state == 0) {
				state = 1;
			}
			else if (state == 1) {
				state = 0;
			}
			
			OC_Count = 0;
			BSP_LED_Off(LED3);
			BSP_LED_Off(LED4);
		}
	}
	
	//implementing full step and counterclockwise
		if ((*htim).Instance == TIM3 && direction == 1 && step == 0) {
			if (OC_Count < speed) {
			
				if (state == 0) {
	//				OC_Count = OC_Count + 1;
						BSP_LED_On(LED3);
						HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET); //A2 on
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); //A2 on
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); //A2 on
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //B1 on
						OC_Count = OC_Count + 1;
				}
				else if (state == 1) {
	//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); //red/A1 off
	//				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); //B1 off
	//				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET); //A2 on
	//				OC_Count = OC_Count + 1;
						BSP_LED_On(LED4);
					
					HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET); //A2 on
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); //A2 on
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //B1 off
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); //A2 on
					
						OC_Count = OC_Count + 1;
				}
				
			}
			else {
				if (state == 0) {
					state = 1;
				}
				else if (state == 1) {
					state = 0;
				}
				
				OC_Count = 0;
				BSP_LED_Off(LED3);
				BSP_LED_Off(LED4);
			}
	}
		
	if ((*htim).Instance == TIM3 && direction == 0 && step == 1) {
			
	}		
	
				//clear the timer counter!  in stm32f4xx_hal_tim.c, the counter is not cleared after  OC interrupt
		__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h

				
	
}
 
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/**
  * @}
  */

/**
  * @}
  */



