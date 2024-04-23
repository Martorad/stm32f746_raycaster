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
#include <math.h>
#include "../../Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_lcd.h"
#include "rc_config.h"
#include "textures.h"
//#include "trig_lut.h"
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
const uint8_t _mSizeX = 48, _mSizeY = 16;
const uint8_t _map[] = {
  5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,7,7,7,7,7,7,7,7,7,5,5,5,5,5,5,5,5,
  5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,5,5,5,0,0,0,0,0,0,0,7,0,0,0,0,0,7,0,0,0,0,0,0,0,0,0,5,
  5,0,0,0,0,0,0,0,0,0,0,0,0,5,5,5,0,5,0,5,0,5,5,0,0,0,0,0,0,5,0,7,0,0,0,0,0,7,0,0,0,0,0,0,0,0,0,5,
  5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,5,5,5,0,0,0,0,0,0,0,5,0,7,0,0,5,0,0,7,0,0,0,0,0,0,0,0,0,5,
  5,5,5,5,5,5,5,5,5,0,0,0,0,0,0,5,5,5,5,5,5,0,0,0,0,0,0,0,0,0,0,7,0,0,0,0,0,7,0,0,0,0,0,0,0,0,0,5,
  5,0,0,0,0,0,0,0,5,0,0,0,0,0,0,5,0,0,0,5,0,0,0,0,0,0,0,0,0,5,0,7,0,0,0,0,0,7,0,0,0,5,5,5,5,5,5,5,
  5,0,5,5,5,5,5,5,5,5,5,5,5,0,0,5,5,5,5,0,0,0,0,0,0,0,0,0,0,0,0,7,7,7,0,7,7,7,0,0,0,5,0,0,0,0,5,5,
  5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,7,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,5,5,
  5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,7,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,5,5,
  5,0,5,5,5,0,0,5,5,5,5,5,5,0,0,5,5,5,5,0,0,0,0,0,0,0,0,0,0,0,0,3,3,3,0,3,3,3,0,0,0,0,0,0,0,0,5,5,
  5,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,5,0,3,0,0,0,0,0,3,0,0,0,5,5,5,5,5,5,5,
  5,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,3,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,5,
  5,1,1,1,1,1,1,1,1,1,1,0,0,1,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,5,0,3,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,5,
  5,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,5,0,3,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,5,
  5,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,3,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,5,
  5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,3,5,7,7,5,3,3,5,5,5,5,5,5,5,5,5,5
};

// PLAYER
float _pPosX = 1.5, _pPosY = 8, _pDeltaX, _pDeltaY, _pMovSpeed, _pRotSpeed;
int16_t _pAngle = 0; // Angle in increments of FOV_INCR radians

float _sinLUT[FOV_RANGE], _cosLUT[FOV_RANGE];

// SYSTEM
volatile uint32_t _sysElapsedTicks = 0; // 10K frequency, 1 tick = 100us = 0.1ms
float _fisheyeCosLUT[FOV];

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
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerRgb565Init(LTDC_FOREGROUND, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerRgb565Init(LTDC_BACKGROUND, LCD_BB_START_ADDRESS);
  BSP_LCD_SetLayerVisible(LTDC_BACKGROUND, DISABLE);
  BSP_LCD_DisplayOn();
  BSP_LCD_SelectLayer(LTDC_BACKGROUND);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_SelectLayer(LTDC_FOREGROUND);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  for (uint16_t i = 0; i < FOV; i++) { _fisheyeCosLUT[i] = 1 / cos((FOV / 2 - i) * FOV_INCR); } // Pre-calculate all cosine values to correct fisheye effect later
  for (uint16_t i = 0; i < FOV_RANGE; i++) { _sinLUT[i]  = sin(i * FOV_INCR + 0.0001); }
  for (uint16_t i = 0; i < FOV_RANGE; i++) { _cosLUT[i]  = cos(i * FOV_INCR + 0.0001); }

  HAL_TIM_Base_Start_IT(&htim7);

  cast();
  _pDeltaX =  _cosLUT[_pAngle] * _pMovSpeed;
  _pDeltaY = -_sinLUT[_pAngle] * _pMovSpeed;
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
      showFPS(cast());

      if (HAL_GPIO_ReadPin(ARDUINO_D4_GPIO_Port, ARDUINO_D4_Pin)) { // RIGHT
        _pAngle -= 5;
        if (_pAngle < 0) { _pAngle = FOV_RANGE; }
        _pDeltaX =  _cosLUT[_pAngle] * _pMovSpeed;
        _pDeltaY = -_sinLUT[_pAngle] * _pMovSpeed;
      }
      if (HAL_GPIO_ReadPin(ARDUINO_D5_GPIO_Port, ARDUINO_D5_Pin)) { // LEFT
        _pAngle += 5;
        if (_pAngle > FOV_RANGE) { _pAngle = 0; }
        _pDeltaX =  _cosLUT[_pAngle] * _pMovSpeed;
        _pDeltaY = -_sinLUT[_pAngle] * _pMovSpeed;
      }
      if (HAL_GPIO_ReadPin(ARDUINO_D2_GPIO_Port, ARDUINO_D2_Pin)) { // FORWARD
        if (_map[(uint16_t)_pPosY * _mSizeX + (uint16_t)(_pPosX + ((_pDeltaX < 0) ? -P_HITBOX_SIZE : P_HITBOX_SIZE))] == 0) { _pPosX += _pDeltaX; }
        if (_map[(uint16_t)(_pPosY + ((_pDeltaY < 0) ? -P_HITBOX_SIZE : P_HITBOX_SIZE)) * _mSizeX + (uint16_t)_pPosX] == 0) { _pPosY += _pDeltaY; }
      }
      if (HAL_GPIO_ReadPin(ARDUINO_D3_GPIO_Port, ARDUINO_D3_Pin)) { // BACKWARD
        if (_map[(uint16_t)_pPosY * _mSizeX + (uint16_t)(_pPosX - ((_pDeltaX < 0) ? -P_HITBOX_SIZE : P_HITBOX_SIZE))] == 0) { _pPosX -= _pDeltaX; }
        if (_map[(uint16_t)(_pPosY - ((_pDeltaY < 0) ? -P_HITBOX_SIZE : P_HITBOX_SIZE)) * _mSizeX + (uint16_t)_pPosX] == 0) { _pPosY -= _pDeltaY; }
      }
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
  RCC_OscInitStruct.PLL.PLLN = 432;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 309;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint32_t cast() {
  // Variable prefix convention: r = ray, m = map, p = performance, c = calculation, t = texture, sb = skybox
  uint32_t pStartTime = _sysElapsedTicks;
  uint16_t rCount;
  int16_t  rAngle;

  BSP_LCD_SelectLayer(0);
  BSP_LCD_SetTextColor(COLOR_GROUND);
  BSP_LCD_FillRect(0, SCREEN_HEIGHT_HALF, SCREEN_WIDTH, SCREEN_HEIGHT_HALF);

  rAngle = _pAngle + FOV / 2;
  if (rAngle > FOV_RANGE) { rAngle -= FOV_RANGE; }

  for (rCount = 0; rCount < FOV; rCount++) {
    // This uses David Ziemkiewicz' method of velocities and times, as well as Lodev's DDA. Massive thanks to both of these legends.
    float   rVelocityX = _cosLUT[rAngle], rVelocityY = -_sinLUT[rAngle], rIntersectX = _pPosX, rIntersectY = _pPosY, rTimeX, rTimeY, rLength = 0;
    int16_t mX = (int16_t)rIntersectX, mY = (int16_t)rIntersectY;
    int8_t  rStepX = (rVelocityX > 0) ? 1 : -1, rStepY = (rVelocityY > 0) ? 1 : -1, rHitSide = 0;

    for (uint16_t i = 0; i < _mSizeX * _mSizeY; i++) {
      if (_map[mY * _mSizeX + mX] > 0) { break; }

      rTimeX = (mX - rIntersectX + (rVelocityX > 0)) / rVelocityX;
      rTimeY = (mY - rIntersectY + (rVelocityY > 0)) / rVelocityY;

      if (rTimeX < rTimeY) { // Vertical line
        rIntersectY += rVelocityY * rTimeX;
        mX += rStepX;
        rIntersectX = mX - (rVelocityX < 0) * rStepX;
        rLength += rTimeX;
        rHitSide = 1;
      }
      else { // Horizontal line
        rIntersectX += rVelocityX * rTimeY;
        mY += rStepY;
        rIntersectY = mY - (rVelocityY < 0) * rStepY;
        rLength += rTimeY;
        rHitSide = 0;
      }
    }

    float   tLineHeight = SCREEN_HEIGHT / rLength * _fisheyeCosLUT[rCount] * LINE_VERTICAL_SCALE;
    uint8_t tTextureIndex = _map[mY * _mSizeX + mX] - 1;

    // DRAW SKYBOX
    //TODO: Find a better way to do this
    if (rCount % (SKYBOX_TEXEL_X / FOV_RECT) == 0) {
      float    sbTexelColumn = SKYBOX_SIZE_X - rAngle * FOV_INCR * SKYBOX_SCALE_F;
      uint16_t sbY = 0, sbOffset = SKYBOX_TEXEL_X - (uint16_t)(((sbTexelColumn - (uint16_t)sbTexelColumn) * SKYBOX_TEXEL_X) + 0.1);

      for (uint16_t i = 0; i < SKYBOX_SIZE_Y; i++) { // TODO: I currently overdraw the shit out of the skybox. It *may* be possible to fix that
        BSP_LCD_SetTextColor(_skybox[i * SKYBOX_SIZE_X + (uint16_t)sbTexelColumn]);

        switch (rCount) { // A switch seems to be marginally faster than an if here, not exactly sure why
          case 0:  BSP_LCD_FillRect((rCount * FOV_RECT), sbY, sbOffset + SKYBOX_TEXEL_X, SKYBOX_TEXEL_Y); break;
          default: BSP_LCD_FillRect((rCount * FOV_RECT) + sbOffset, sbY, (rCount == FOV - (SKYBOX_TEXEL_X / FOV_RECT)) ? (SKYBOX_TEXEL_X - sbOffset) : SKYBOX_TEXEL_X, SKYBOX_TEXEL_Y); break;
        }

        sbY += SKYBOX_TEXEL_Y;
      }
    }

    // DRAW WALLS
    if (tLineHeight > SCREEN_HEIGHT / DOF) { // if line is smaller than the shortest possible line defined by DOF, don't bother drawing it
      float tX, tY = 0, tYStep = tLineHeight / TEXTURE_SIZE, tOffset = (tLineHeight - SCREEN_HEIGHT) * 0.5;

      if (rHitSide) { tX = (1 - (rIntersectY - (uint16_t)rIntersectY)) * TEXTURE_SIZE; if (rAngle < FOV_RANGE / 4 || rAngle > (FOV_RANGE / 4) * 3) { tX = TEXTURE_SIZE - tX; }}
      else          { tX = (1 - (rIntersectX - (uint16_t)rIntersectX)) * TEXTURE_SIZE; if (rAngle < FOV_RANGE / 2)                                 { tX = TEXTURE_SIZE - tX; }}

      if (tLineHeight > SCREEN_HEIGHT) {
        uint16_t tSkipLines = tOffset / tYStep, tFirstLine = tYStep - (tOffset - tSkipLines * tYStep);
        tY = tFirstLine;
        for (uint16_t i = tSkipLines; i < TEXTURE_SIZE - tSkipLines; i++) {
          BSP_LCD_SetTextColor(_textures[tTextureIndex + rHitSide][i * TEXTURE_SIZE + (uint16_t)(tX)]);
          if (i != tSkipLines && i != TEXTURE_SIZE - tSkipLines - 1) {
            BSP_LCD_FillRect((rCount * FOV_RECT), tY, FOV_RECT, tYStep + 1);
            tY += tYStep;
          }
          else {
            BSP_LCD_FillRect((rCount * FOV_RECT), (i == tSkipLines) ? 0 : tY, FOV_RECT, tFirstLine + 2);
          }
        }
      }
      else {
        tOffset *= -1; // invert value of texture offset to make it positive

        for (uint16_t i = 0; i < TEXTURE_SIZE; i++) {
          BSP_LCD_SetTextColor(_textures[tTextureIndex + rHitSide][i * TEXTURE_SIZE + (uint16_t)(tX)]);
          BSP_LCD_FillRect((rCount * FOV_RECT), tOffset + tY, FOV_RECT, tYStep + 1);
          tY += tYStep;
        }
      }
    }

    rAngle--;
    if (rAngle < 0) { rAngle += FOV_RANGE; }
  }

  uint32_t pFrameTime = _sysElapsedTicks - pStartTime;

  _pMovSpeed = pFrameTime * INCR_TRANSLATION;
  _pRotSpeed = pFrameTime * INCR_ROTATION;

  return pFrameTime;
}

float rayLength(float ax, float ay, float bx, float by) {
  return fsqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
}

float rayLengthFast(float ax, float ay, float bx, float by) {
  return (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
}

float fsqrt(float x) {
  float g = x / 2.0;

  g = 0.5 * (g + x / g);
  g = 0.5 * (g + x / g);
  g = 0.5 * (g + x / g);

  return g;
}

void pageFlip() {
  static volatile uint8_t activeBuffer = 1;
  activeBuffer ^= 1;
  BSP_LCD_SWAP(activeBuffer);
}

uint32_t CLUT(uint8_t index, uint8_t hitSide) { // Compared to an array of const uint32_t, this is faster, as tested on Full resolution.
  if (hitSide) { // Y side wall
    switch (index) {
      case 1:  return 0xFFAA0000; // red
      case 2:  return 0xFF00AA00; // green
      case 3:  return 0xFF0000AA; // blue
      case 4:  return 0xFF00AAAA; // cyan
      case 5:  return 0xFFAA00AA; // magenta
      case 6:  return 0xFFAAAA00; // yellow
      default: return 0xFFAAAAAA; // white
    }
  }
  else { // X side wall
    switch (index) {
      case 1:  return 0xFFFF0000;
      case 2:  return 0xFF00FF00;
      case 3:  return 0xFF0000FF;
      case 4:  return 0xFF00FFFF;
      case 5:  return 0xFFFF00FF;
      case 6:  return 0xFFFFFF00;
      default: return 0xFFFFFFFF;
    }
  }
}

uint32_t dimColor(uint32_t inputColor, float dimmingFactor) {
  if (dimmingFactor > 1) { return inputColor; }
  return 0xFF000000 |
  (0x00FF0000 & (uint32_t)((0x00FF0000 & inputColor) * dimmingFactor)) |
  (0x0000FF00 & (uint32_t)((0x0000FF00 & inputColor) * dimmingFactor)) |
  (0x000000FF & (uint32_t)((0x000000FF & inputColor) * dimmingFactor));
}

void showFPS(uint32_t frameTime) {
  uint8_t frameTimeS[8];
  sprintf((char*)frameTimeS, "%li", frameTime);
  BSP_LCD_SetTextColor(0xFF0000FF);
  BSP_LCD_DisplayStringAt(0, 0, frameTimeS, LEFT_MODE);
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
  if (htim->Instance == TIM7) {
    _sysElapsedTicks++;
  }
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
