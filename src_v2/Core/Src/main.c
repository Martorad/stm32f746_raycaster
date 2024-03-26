/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma2d.h"
#include "ltdc.h"
#include "quadspi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>
#include <math.h>
#include "../../Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
  float x;
  float y;
} vector;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAP_SIZE_X    24
#define MAP_SIZE_Y    24

#define SCREEN_WIDTH  60
#define SCREEN_HEIGHT 272
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void cast(void);
uint32_t CLUT(uint8_t index, int8_t hitSide);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// SYSTEM
volatile uint64_t _microSeconds;
uint64_t _time, _oldTime;
const float _screenWidth = SCREEN_WIDTH, _screenHeight = SCREEN_HEIGHT;
volatile uint8_t _activeBuffer = 1;
volatile float _moveSpeed, _rotSpeed;

// MAP
uint8_t _map[MAP_SIZE_X][MAP_SIZE_Y] = {
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,2,2,2,2,2,0,0,0,0,3,0,3,0,3,0,0,0,1},
  {1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,3,0,0,0,3,0,0,0,1},
  {1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,2,2,0,2,2,0,0,0,0,3,0,3,0,3,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,0,0,0,5,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

// PLAYER
//vector _pPos = {22, 12}, _pDir = {-1, 0}, _pPln = {0, 0.66};
float _pPosX = 22, _pPosY = 12, _pDirX = -1, _pDirY = 0, _pPlnX = 0, _pPlnY = 0.66;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
//  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
//  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA2D_Init();
  MX_QUADSPI_Init();
  MX_CRC_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_USB_OTG_FS_HCD_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LTDC_FOREGROUND, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerDefaultInit(LTDC_BACKGROUND, LCD_BB_START_ADDRESS);
  BSP_LCD_SetLayerVisible(LTDC_BACKGROUND, DISABLE);
  BSP_LCD_DisplayOn();
  BSP_LCD_SelectLayer(LTDC_FOREGROUND);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  cast();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (!HAL_GPIO_ReadPin(LCD_VSYNC_GPIO_Port, LCD_VSYNC_Pin)) {
      _activeBuffer ^= 1;
      BSP_LCD_SWAP(_activeBuffer);
      cast();
    }

    if (HAL_GPIO_ReadPin(ARDUINO_D2_GPIO_Port, ARDUINO_D2_Pin)) { // FORWARD
      if (_map[(int)(_pPosX + _pDirX * _moveSpeed)][(int)_pPosY] == 0) { _pPosX += _pDirX * _moveSpeed; }
      if (_map[(int)_pPosX][(int)(_pPosY + _pDirY * _moveSpeed)] == 0) { _pPosY += _pDirY * _moveSpeed; }
    }
    if (HAL_GPIO_ReadPin(ARDUINO_D3_GPIO_Port, ARDUINO_D3_Pin)) { // BACKWARD
      if (_map[(int)(_pPosX - _pDirX * _moveSpeed)][(int)_pPosY] == 0) { _pPosX -= _pDirX * _moveSpeed; }
      if (_map[(int)_pPosX][(int)(_pPosY - _pDirY * _moveSpeed)] == 0) { _pPosY -= _pDirY * _moveSpeed; }
    }
    if (HAL_GPIO_ReadPin(ARDUINO_D4_GPIO_Port, ARDUINO_D4_Pin)) { // RIGHT
      float oldDirX = _pDirX;
      _pDirX = _pDirX * cos(-_rotSpeed) - _pDirY * sin(-_rotSpeed);
      _pDirY = oldDirX * sin(-_rotSpeed) + _pDirY * cos(-_rotSpeed);

      float oldPlnX = _pPlnX;
      _pPlnX = _pPlnX * cos(-_rotSpeed) - _pPlnY * sin(-_rotSpeed);
      _pPlnY = oldPlnX * sin(-_rotSpeed) + _pPlnY * cos(-_rotSpeed);
    }
    if (HAL_GPIO_ReadPin(ARDUINO_D5_GPIO_Port, ARDUINO_D5_Pin)) { // LEFT

    }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    _microSeconds++;
  }
}

void cast(void) {
  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  for (uint16_t rCount = 0; rCount < _screenWidth; rCount++) {
    float    planeOffset = 2 * rCount / _screenWidth - 1;                               // Calculate ray angle as a component of screen width, normalize from -1 to 1
//    vector   rDir = {_pDirX + _pPlnX * planeOffset, _pDirY + _pPlnY * planeOffset};   // Calculate ray direction using the camera plane offset
    float    rDirX = _pDirX + _pPlnX * planeOffset;
    float    rDirY = _pDirY + _pPlnY * planeOffset;
    uint16_t mapX = (uint16_t)_pPosX, mapY = (uint16_t)_pPosY;                          // Floor player position floats to integers
    float    rSideDistX, rSideDistY;                                                    // Length of the ray to the first X and Y intersects
    float    rDeltaDistX = abs(1 / rDirX), rDeltaDistY = abs(1 / rDirY);                // Delta distance from first intersect to next intersect
    float    rWallDist;                                                                 // Stores length of entire ray
    int8_t   rStepX, rStepY;                                                            // Stores ray step, always either -1 or 1
    int8_t   rHit = 0, rHitSide;                                                        // Flags, set when ray hits a wall and when it is determined whether an X (0) or Y (1) wall was hit

    if (rDirX < 0) {
      rStepX = -1;
      rSideDistX = (_pPosX - mapX) * rDeltaDistX;
    }
    else {
      rStepX = 1;
      rSideDistX = (mapX + 1.0 - _pPosX) * rDeltaDistX;
    }
    if (rDirY < 0) {
      rStepY = -1;
      rSideDistY = (_pPosY - mapY) * rDeltaDistY ;
    }
    else {
      rStepY = 1;
      rSideDistY = (mapY + 1.0 - _pPosY) * rDeltaDistY;
    }

    while (rHit == 0) {
      if (rSideDistX < rSideDistY) {
        rSideDistX += rDeltaDistX;
        mapX += rStepX;
        rHitSide = 0;
      }
      else {
        rSideDistY += rDeltaDistY;
        mapY += rStepY;
        rHitSide = 1;
      }

      if (_map[mapX][mapY]) { rHit = 1; }
    }

    if (rHitSide) { rWallDist = rSideDistY - rDeltaDistY; }
    else          { rWallDist = rSideDistX - rDeltaDistX; }

    int16_t lineHeight = (int16_t)(_screenHeight / rWallDist);
    int16_t lineStart  = -lineHeight / 2 + _screenHeight / 2;
    if (lineStart < 0) { lineStart = 0; }

    BSP_LCD_SetTextColor(CLUT(_map[mapX][mapY], rHitSide));
//    BSP_LCD_DrawVLine(rCount, lineStart, lineHeight);
    BSP_LCD_FillRect(rCount * 8, lineStart, 8, lineHeight);
  }

  _oldTime = _time;
  _time = _microSeconds;
  uint64_t frameTime = (_time - _oldTime) / 1000;

  uint8_t frameTimeS[32];
  itoa(frameTime, frameTimeS, 10);
  BSP_LCD_SetTextColor(0xFF000000);
  BSP_LCD_SetBackColor(0xFFFF2244);
  BSP_LCD_DisplayStringAt(0, 0, frameTimeS, LEFT_MODE);

  _moveSpeed = frameTime * 0.00005;
  _rotSpeed  = frameTime * 0.00003;
}

uint32_t CLUT(uint8_t index, int8_t hitSide) {
  if (hitSide) { // Y side wall
    switch (index) {
      case 1:  return 0xFFAA0000;
      case 2:  return 0xFF00AA00;
      case 3:  return 0xFF0000AA;
      case 4:  return 0xFFAAAAAA;
      default: return 0xFFAAAA00;
    }
  }
  else { // X side wall
    switch (index) {
      case 1:  return 0xFFFF0000;
      case 2:  return 0xFF00FF00;
      case 3:  return 0xFF0000FF;
      case 4:  return 0xFFFFFFFF;
      default: return 0xFFFFFF00;
    }
  }
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
  HAL_GPIO_WritePin(ARDUINO_D13_GPIO_Port, ARDUINO_D13_Pin, GPIO_PIN_SET);
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
