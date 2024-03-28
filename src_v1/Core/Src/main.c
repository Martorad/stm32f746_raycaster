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
#include "adc.h"
#include "crc.h"
#include "dcmi.h"
#include "dma2d.h"
#include "eth.h"
#include "ltdc.h"
#include "quadspi.h"
#include "rtc.h"
#include "sai.h"
#include "spdifrx.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>
#include "math.h"
#include "../../Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_lcd.h"
#include "rc_config.h"
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
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// MAP
uint8_t _mSizeX = 32, _mSizeY = 16;
uint8_t _map[] = {
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
  1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,0,0,0,0,1,
  1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,1,0,1,0,1,1,0,0,0,0,0,0,1,0,1,
  1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,3,1,0,0,0,0,0,0,0,2,0,1,
  1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,
  1,0,0,0,0,0,0,0,2,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,4,0,1,
  1,0,1,2,3,4,3,2,1,2,3,4,3,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,
  1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,2,0,0,0,3,0,1,
  1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,2,0,0,0,3,0,1,
  1,0,1,2,3,4,3,2,1,2,3,4,3,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,
  1,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,4,0,1,
  1,1,1,1,1,1,1,1,1,4,4,4,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,
  1,4,4,4,4,4,4,4,4,4,4,0,0,4,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,2,0,1,
  1,4,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,1,
  1,4,4,4,4,4,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
};

// PLAYER
float _pPosX = 1, _pPosY = 8, _pAngle = 0 * FOV_INCR, _pDeltaX, _pDeltaY, _pMovSpeed, _pRotSpeed;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  _pDeltaX =  cos(_pAngle) * 15 * INCR_TRANSLATION;
  _pDeltaY = -sin(_pAngle) * 15 * INCR_TRANSLATION;
  /* USER CODE END 1 */
/* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_ADC3_Init();
  MX_CRC_Init();
  MX_DCMI_Init();
  MX_DMA2D_Init();
  MX_ETH_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_QUADSPI_Init();
  MX_RTC_Init();
  MX_SAI2_Init();
  MX_SPDIFRX_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerRgb565Init(LTDC_FOREGROUND, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerRgb565Init(LTDC_BACKGROUND, LCD_BB_START_ADDRESS);
  BSP_LCD_SetLayerVisible(LTDC_BACKGROUND, DISABLE);
  BSP_LCD_DisplayOn();
  BSP_LCD_SelectLayer(LTDC_FOREGROUND);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    if (!HAL_GPIO_ReadPin(LCD_VSYNC_GPIO_Port, LCD_VSYNC_Pin)) {
      pageFlip();

      uint8_t frameTime[32];
      itoa(cast(), frameTime, 10);
      BSP_LCD_SetTextColor(0xFF000000);
      BSP_LCD_DisplayStringAt(0, 0, frameTime, LEFT_MODE);
    }

    if (HAL_GPIO_ReadPin(ARDUINO_D4_GPIO_Port, ARDUINO_D4_Pin)) { // RIGHT
      _pAngle -= _pRotSpeed;
      if (_pAngle < 0) { _pAngle = M_TWOPI; }
      _pDeltaX =  cos(_pAngle) * _pMovSpeed;
      _pDeltaY = -sin(_pAngle) * _pMovSpeed;
    }
    if (HAL_GPIO_ReadPin(ARDUINO_D5_GPIO_Port, ARDUINO_D5_Pin)) { // LEFT
      _pAngle += _pRotSpeed;
      if (_pAngle > M_TWOPI) { _pAngle = 0; }
      _pDeltaX =  cos(_pAngle) * _pMovSpeed;
      _pDeltaY = -sin(_pAngle) * _pMovSpeed;
    }
    if (HAL_GPIO_ReadPin(ARDUINO_D2_GPIO_Port, ARDUINO_D2_Pin)) { // FORWARD
      _pPosX += _pDeltaX;
      _pPosY += _pDeltaY;
    }
    if (HAL_GPIO_ReadPin(ARDUINO_D3_GPIO_Port, ARDUINO_D3_Pin)) { // BACKWARD
      _pPosX -= _pDeltaX;
      _pPosY -= _pDeltaY;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint32_t cast() {
  // Variable naming convention: r = ray, m = map, p = performance, c = calculation
  uint32_t pStartTime = HAL_GetTick();
  uint16_t rCount, rCastLimitV, rCastLimitH, mX, mY, mPosition = 0;
  uint8_t  mColorV, mColorH;
  float    rIntersectX, rIntersectY, rAngle, rOffsetX, rOffsetY, rShortest;

  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  rAngle = _pAngle + FOV_HALF * FOV_INCR;
  if (rAngle < 0)       { rAngle += M_TWOPI; }
  if (rAngle > M_TWOPI) { rAngle -= M_TWOPI; }

  for (rCount = 0; rCount < FOV; rCount++) {
    // VERTICAL LINE CHECK
    rCastLimitV = 0;
    float rVertical = FLT_MAX;
    float cTan = tan(rAngle);

    if (rAngle > M_PI_2 && rAngle < M_3PI_2) { // looking left
      rIntersectX = (uint16_t)_pPosX - 0.000001;
      rIntersectY = (_pPosX - rIntersectX) * cTan + _pPosY;
      rOffsetX = -1;
      rOffsetY = -rOffsetX * cTan;
    }
    else if (rAngle < M_PI_2 || rAngle > M_3PI_2) { // looking right
      rIntersectX = (uint16_t)_pPosX + 1;
      rIntersectY = (_pPosX - rIntersectX) * cTan + _pPosY;
      rOffsetX = 1;
      rOffsetY = -rOffsetX * cTan;
    }
    else { // looking perfectly vertical
      rIntersectX = _pPosX;
      rIntersectY = _pPosY;
      rCastLimitV = DOF;
    }

    while (rCastLimitV < DOF) {
      mX = (uint16_t)rIntersectX;
      mY = (uint16_t)rIntersectY;
      mPosition = mY * _mSizeX + mX;

      if (mPosition > 0 && mPosition < _mSizeX * _mSizeY && _map[mPosition] >= 1) {
        rVertical = rayLength(_pPosX, rIntersectX, _pPosY, rIntersectY);
        rCastLimitV = R_HIT;
      }
      else {
        rIntersectX += rOffsetX;
        rIntersectY += rOffsetY;
        rCastLimitV++;
      }
    }

    mColorV = _map[mPosition];

    // HORIZONTAL LINE CHECK
    rCastLimitH = 0;
    float rHorizontal = FLT_MAX;
    float cRTan = 1 / tan(rAngle);

    if (rAngle < M_PI) { // looking up
      rIntersectY = (uint16_t)_pPosY - 0.000001;
      rIntersectX = (_pPosY - rIntersectY) * cRTan + _pPosX;
      rOffsetY = -1;
      rOffsetX = -rOffsetY * cRTan;
    }
    else if (rAngle > M_PI) { // looking down
      rIntersectY = (uint16_t)_pPosY + 1;
      rIntersectX = (_pPosY - rIntersectY) * cRTan + _pPosX;
      rOffsetY = 1;
      rOffsetX = -rOffsetY * cRTan;
    }
    else { // looking perfectly horizontal
      rIntersectY = _pPosY;
      rIntersectX = _pPosX;
      rCastLimitH = DOF;
    }

    while (rCastLimitH < DOF) {
      mX = (uint16_t)rIntersectX;
      mY = (uint16_t)rIntersectY;
      mPosition = mY * _mSizeX + mX;

      if (mPosition > 0 && mPosition < _mSizeX * _mSizeY && _map[mPosition] >= 1) {
        rHorizontal = rayLength(_pPosX, rIntersectX, _pPosY, rIntersectY);
        rCastLimitH = R_HIT;
      }
      else {
        rIntersectX += rOffsetX;
        rIntersectY += rOffsetY;
        rCastLimitH++;
      }
    }

    mColorH = _map[mPosition];

    // RENDERING
    uint8_t  colorIndex, hitSide;
    uint16_t rCastTotal;
    if (rVertical < rHorizontal) {
      rShortest = rVertical;
      colorIndex = mColorV;
      hitSide = 1;
      rCastTotal = rCastLimitV;
    }
    else {
      rShortest = rHorizontal;
      colorIndex = mColorH;
      hitSide = 0;
      rCastTotal = rCastLimitH;
    }

#ifdef REMOVE_FISHEYE
    float rFisheyeFix = _pAngle - rAngle;
    if (rFisheyeFix < 0)       { rFisheyeFix += M_TWOPI; }
    if (rFisheyeFix > M_TWOPI) { rFisheyeFix -= M_TWOPI; }
    rShortest *= cos(rFisheyeFix);
#endif

    if (rCastTotal == R_HIT) {
      float    lineHeight = 272 / rShortest;
      uint16_t lineOffset = 0;

      lineHeight *= LINE_VERTICAL_SCALE;
      if (lineHeight > 272 / DOF) { // if line is smaller than the shortest possible line defined by DOF, don't bother drawing it
        if (lineHeight > 272) { lineHeight = 272; }
        else { lineOffset = (uint16_t)(272 - lineHeight) >> 1; }
        BSP_LCD_SetTextColor(dimColor(CLUT(colorIndex, hitSide), lineHeight / 68));
        BSP_LCD_FillRect((rCount * FOV_RECT), lineOffset, FOV_RECT, lineHeight);
      }
    }

    rAngle -= FOV_INCR;
    if (rAngle < 0)       { rAngle += M_TWOPI; }
    if (rAngle > M_TWOPI) { rAngle -= M_TWOPI; }
  }

  uint32_t frameTime = HAL_GetTick() - pStartTime;

  _pMovSpeed = frameTime * INCR_TRANSLATION;
  _pRotSpeed = frameTime * INCR_ROTATION;

  return frameTime;
}

float rayLength(float ax, float bx, float ay, float by) {
  return sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
}

void pageFlip() {
  static volatile uint8_t activeBuffer = 1;
  activeBuffer ^= 1;
  BSP_LCD_SWAP(activeBuffer);
}

uint32_t CLUT(uint8_t index, uint8_t hitSide) {
  if (hitSide) { // Y side wall
    switch (index) {
      case 1:  return 0xFFAA0000;
      case 2:  return 0xFF00AA00;
      case 3:  return 0xFF0000AA;
      case 4:  return 0xFFAAAA00;
      default: return 0xFFAAAAAA;
    }
  }
  else { // X side wall
    switch (index) {
      case 1:  return 0xFFFF0000;
      case 2:  return 0xFF00FF00;
      case 3:  return 0xFF0000FF;
      case 4:  return 0xFFFFFF00;
      default: return 0xFFFFFFFF;
    }
  }
}

uint32_t dimColor(uint32_t inputColor, float dimmingFactor) {
  if (dimmingFactor > 1) { return inputColor; }
  if (dimmingFactor < 0) { dimmingFactor = 0; }
  return 0xFF000000 |
  (0x00FF0000 & (uint32_t)((0x00FF0000 & inputColor) * dimmingFactor)) |
  (0x0000FF00 & (uint32_t)((0x0000FF00 & inputColor) * dimmingFactor)) |
  (0x000000FF & (uint32_t)((0x000000FF & inputColor) * dimmingFactor));
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  HAL_GPIO_WritePin(ARDUINO_SCK_D13_GPIO_Port, ARDUINO_SCK_D13_Pin, GPIO_PIN_SET);
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
