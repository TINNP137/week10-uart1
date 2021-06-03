/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char TxDataBuffer[32] =
{ 0 };
char RxDataBuffer[32] =
{ 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UARTRecieveAndResponsePolling();
int16_t UARTRecieveIT();
enum _StateDisplay
{
  StateDisplay_Start = 0,
  StateDisplay_MenuRoot_Print =10,
  StateDisplay_MenuRoot_WaitInput,
  StateDisplay_Menu1_Print =20,
  StateDisplay_Menu1_WaitInput,
  StateDisplay_Menu2_Print =30,
  StateDisplay_Menu2_WaitInput

};
uint8_t STATE_Display = 0;
uint32_t timestamp = 0;
int count =1;
uint8_t Is_LED_on =1;
int LED_TOGGLE_DELAY =500;
int is_press =0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  {
  char temp[]="HELLO WORLD\r\n please type something to test UART\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)temp, strlen(temp),10);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/*Method 1 Polling Mode*/  // BAD Implement

//		UARTRecieveAndResponsePolling();

		/*Method 2 Interrupt Mode*/
		HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 32);

		/*Method 2 W/ 1 Char Received*/
		int16_t inputchar = UARTRecieveIT();
		if(inputchar!=-1)
		{

			sprintf(TxDataBuffer, "ReceivedChar:[%c]\r\n", inputchar);
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);


		}
		switch (STATE_Display)
		{
			case StateDisplay_Start:
				STATE_Display = StateDisplay_MenuRoot_Print;
				break;
			case StateDisplay_MenuRoot_Print: //display one time state
				yemp();
				STATE_Display = StateDisplay_MenuRoot_WaitInput;
				break;
			case StateDisplay_MenuRoot_WaitInput: //wait state for input
				switch (inputchar)
				{
			      case 0:
			        //no input ; just wait input
			        break;
			      case -1:
			        //no input ; just wait input
			        break;
			      case '0':
			        STATE_Display = StateDisplay_Menu1_Print;
			        break;
			      case '1':
			        STATE_Display = StateDisplay_Menu2_Print;
			        break;
			      default: // actually error , you can add error message
			        wng_press();
			        STATE_Display = StateDisplay_MenuRoot_Print;
			        break;
				}
				break;
			case StateDisplay_Menu1_Print: //display one time state //LED
				LED_msg();
				STATE_Display = StateDisplay_Menu1_WaitInput;
				break;
			case StateDisplay_Menu2_Print: //display state
				B1_msg();
			    STATE_Display = StateDisplay_Menu2_WaitInput;
			    break;

			case StateDisplay_Menu2_WaitInput: //make decision state
			    if (HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin) ==1 && is_press ==0)
			    {
			    	B1_notpress();
			    	is_press =1;
			    }
			    else if (HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin) ==0 && is_press ==1)
			    {
			    	B1_press();
			    	is_press =0;
			    }
			    switch (inputchar)
			    {
			    	case 0:
			    		//no input ; just wait input
			    		break;
			    	case -1:
			    		//no input ; just wait input
			    	    break;
			    	case 'x': //
			    		STATE_Display = StateDisplay_MenuRoot_Print;
			    		break;
			    	default: // actually error , you can add error message
			    	    wng_press();
			    	    STATE_Display = StateDisplay_Menu2_Print;
			    	    break;
			    }
			    break;
			case StateDisplay_Menu1_WaitInput: //LED
			      switch (inputchar)
			      {
			      case 0:
			        //no input ; just wait input
			        break;
			      case -1:
			    	  //no input ; just wait input
			    	  break;
			      case 'a': //+1Hz
			    	  Is_LED_on = 1;
			    	  count +=1;

			    	  break;
			      case 's': // -1Hz
			    	  Is_LED_on = 1;
			    	  count -=1;

			    	  break;
			      case 'd': // On/Off
			    	  if (Is_LED_on == 1)
			    	  {
			    		  Is_LED_on = 0;
			    	  }
			    	  else if (Is_LED_on == 0)
			    	  {
			    		  Is_LED_on = 1;
			    	  }
			    	  STATE_Display = StateDisplay_Menu1_Print;
			    	  break;
			      case 'x': //
			    	  STATE_Display = StateDisplay_MenuRoot_Print;
			    	  break;

			      default: // actully error , you can add error message
			    	  wng_press();
			    	  STATE_Display = StateDisplay_Menu1_Print;
			    	  break;
			      }
			      break;
		}


		/*This section just simulate Work Load*/
		if (count<0)
		{
			count=0;
		}
		if(Is_LED_on ==1)
		{
			HAL_Delay(500/count);
//		if (((HAL_GetTick() >=-timestamp) >= ((LED_TOGGLE_DELAY / count))) && (Is_LED_on == 1)) // maybe -(count*Hertz)
//		{
//			timestamp = HAL_GetTick();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
		else if(Is_LED_on ==0)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UARTRecieveAndResponsePolling()
{
	char Recieve[32]={0};
	//start Receive in Polling mode
	HAL_UART_Receive(&huart2, (uint8_t*)Recieve, 32, 1000);
	//Create feedback text
	sprintf(TxDataBuffer, "Received:[%s]\r\n", Recieve);
	//send text
	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);

}

void yemp()
{
	char yemp[]= "Menu\r\n press 0 for LED_status\r\n press 1 for Button_status\r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)yemp, strlen(yemp),30);
}
void wng_press()
{
	char emp[]= "error please press the correct key\r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)emp, strlen(emp),10);
}
void LED_msg()
{
	char lemp[]= "a: speed +1 Hz\r\n s: speed -1 Hz\r\n d: On/Off \r\n x: back \r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)lemp, strlen(lemp),30);
}
void B1_msg()
{
	char kemp[]= "B1: shpw status \r\n x: back \r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)kemp, strlen(kemp),20);
}
void B1_press()
{
	char bemp[]= "Button is pressed\r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)bemp, strlen(bemp),10);
}
void B1_notpress()
{
	char nbemp[]= "Button is not pressed \r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)nbemp, strlen(nbemp),10);
}

int16_t UARTRecieveIT()
{
	//stored data last position
	static uint32_t dataPos =0;
	//create dummy data
	int16_t data=-1;
	//check pos in data vs last position
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		//read data to buffer
		data=RxDataBuffer[dataPos];
		//move to next pos
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	sprintf(TxDataBuffer, "Received:[%s]\r\n", RxDataBuffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);  // or  HAL_UART_TransmitIT(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer));
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
