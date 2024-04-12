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
#include "trig_lut.h"
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
  5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
  5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,5,5,5,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,
  5,0,0,0,0,0,0,0,0,0,0,0,0,5,5,5,0,5,0,5,0,5,5,0,0,0,0,0,0,5,0,5,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,5,
  5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,5,5,5,0,0,0,0,0,0,0,5,0,5,5,5,0,0,0,0,0,0,0,0,0,0,0,0,0,5,
  5,5,5,5,5,5,5,5,5,0,0,0,0,0,0,5,5,5,5,5,5,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,
  5,0,0,0,0,0,0,0,5,0,0,0,0,0,0,5,0,0,0,5,0,0,0,0,0,0,0,0,0,5,0,5,0,0,0,0,0,0,0,0,0,5,5,5,5,5,5,5,
  5,0,5,5,5,5,5,5,5,5,5,5,5,0,0,5,5,5,5,0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,5,0,0,0,0,5,5,
  5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,3,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,5,5,
  5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,3,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,5,5,
  5,0,5,5,5,5,5,5,5,5,5,5,5,0,0,5,5,5,5,0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,5,
  5,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,5,0,5,0,0,0,0,0,0,0,0,0,5,5,5,5,5,5,5,
  5,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,
  5,1,1,1,1,1,1,1,1,1,1,0,0,1,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,5,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,
  5,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,5,0,5,5,5,0,0,0,0,0,0,0,0,0,0,0,0,0,5,
  5,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,
  5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5
};

// PLAYER
float _pPosX = 1.5, _pPosY = 8, _pAngle = 0 * FOV_INCR, _pDeltaX, _pDeltaY, _pMovSpeed, _pRotSpeed;

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

  for (uint16_t i = 0; i < FOV; i++) { _fisheyeCosLUT[i] = cos((FOV_HALF - i) * FOV_INCR); } // Pre-calculate all cosine values to correct fisheye effect later

  HAL_TIM_Base_Start_IT(&htim7);

  cast();
  _pDeltaX =  cos(_pAngle) * _pMovSpeed;
  _pDeltaY = -sin(_pAngle) * _pMovSpeed;
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
  // Variable naming convention: r = ray, m = map, p = performance, c = calculation, t = texture
  uint32_t pStartTime = _sysElapsedTicks;
  uint16_t rCount, rCastLimitV, rCastLimitH, mPosition = 0;
  uint8_t  mTextureV, mTextureH;
  float    rIntersectXV, rIntersectYV, rIntersectXH, rIntersectYH, rAngle, rOffsetX, rOffsetY, rShortest, rLenV, rLenH, cTan, cRTan;
  union    { float f; uint32_t u; } sign;

  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  rAngle = _pAngle + FOV_HALF * FOV_INCR;
  if (rAngle < 0)       { rAngle += M_TWOPI; }
  if (rAngle > M_TWOPI) { rAngle -= M_TWOPI; }

  for (rCount = 0; rCount < FOV; rCount++) {
    rCastLimitV = 0; rCastLimitH = 0;
    rLenV = FLT_MAX; rLenH = FLT_MAX;
    cTan = _tanLUT[(uint16_t)(rAngle * M_LUT_P)]; cRTan = 1 / cTan;

    // VERTICAL LINE CHECK
    rOffsetX = (rAngle < M_PI_2 || rAngle > M_3PI_2) ? 1 : -1; // looking right / left
    sign.f = rOffsetX; sign.u = sign.u >> 31; // A bit of a hack - extract sign bit from the float and use it to correct the value of rIntersect and mPosition in the array. Done to avoid floating point error when map size exceeds a power of two and to optimize above if-statement.
    rIntersectXV = (uint16_t)_pPosX + !sign.u;
    rIntersectYV = (_pPosX - rIntersectXV) * cTan + _pPosY;
    rOffsetY = -rOffsetX * cTan;

    while (rCastLimitV < DOF) { // TODO: Combine both while loops, idk why I haven't thought of that yet. Might be a decent idea to do it with arrays of size 2.
      mPosition = (uint16_t)rIntersectYV * _mSizeX + (uint16_t)rIntersectXV - sign.u;

      if (_map[mPosition]) {
        rLenV = rayLength(_pPosX, rIntersectXV, _pPosY, rIntersectYV);
        rCastLimitV = R_HIT;
      }
      else {
        rIntersectXV += rOffsetX;
        rIntersectYV += rOffsetY;
        rCastLimitV++;
      }
    }

    mTextureV = _map[mPosition] - 1;

    // HORIZONTAL LINE CHECK
    rOffsetY = (rAngle < M_PI) ? -1 : 1; // looking up / down
    sign.f = rOffsetY; sign.u = sign.u >> 31;
    rIntersectYH = (uint16_t)_pPosY + !sign.u;
    rIntersectXH = (_pPosY - rIntersectYH) * cRTan + _pPosX;
    rOffsetX = -rOffsetY * cRTan;

    while (rCastLimitH < DOF) {
      mPosition = ((uint16_t)rIntersectYH - sign.u) * _mSizeX + (uint16_t)rIntersectXH;

      if (_map[mPosition]) {
        rLenH = rayLength(_pPosX, rIntersectXH, _pPosY, rIntersectYH);
        rCastLimitH = R_HIT;
      }
      else {
        rIntersectXH += rOffsetX;
        rIntersectYH += rOffsetY;
        rCastLimitH++;
      }
    }

    mTextureH = _map[mPosition] - 1;

    // RENDERING
    uint16_t rCastTotal;
    if (rLenV < rLenH) { rCastTotal = rCastLimitV; }
    else               { rCastTotal = rCastLimitH; }

    if (rCastTotal == R_HIT) {
      uint8_t tTextureIndex, hitSide;
      if (rLenV < rLenH) {
        rShortest = rLenV;
        tTextureIndex = mTextureV;
        hitSide = 1;
      }
      else {
        rShortest = rLenH;
        tTextureIndex = mTextureH;
        hitSide = 0;
      }

      rShortest *= _fisheyeCosLUT[rCount];
      float lineHeight = SCREEN_HEIGHT / rShortest * LINE_VERTICAL_SCALE;

      // DRAW WALLS
      if (lineHeight > SCREEN_HEIGHT / DOF) { // if line is smaller than the shortest possible line defined by DOF, don't bother drawing it
        uint16_t skipLines;
        float    tX, tY = 0, tYStep = lineHeight * TEXTURE_SIZE_RECIPROCAL, tOffset = (lineHeight - SCREEN_HEIGHT) * 0.5, firstLine;

        if (hitSide) { tX = (1 - (rIntersectYV - (uint32_t)rIntersectYV)) * TEXTURE_SIZE; if (rAngle < M_PI_2 || rAngle > M_3PI_2) { tX = TEXTURE_SIZE - tX; }}
        else         { tX = (1 - (rIntersectXH - (uint32_t)rIntersectXH)) * TEXTURE_SIZE; if (rAngle < M_PI)                       { tX = TEXTURE_SIZE - tX; }}

        if (lineHeight > SCREEN_HEIGHT) {
          skipLines = tOffset / tYStep;
          firstLine = tYStep - (tOffset - skipLines * tYStep);
          tY = firstLine;
          for (uint16_t i = skipLines; i < TEXTURE_SIZE - skipLines; i++) {
            BSP_LCD_SetTextColor(_textures[tTextureIndex + hitSide][i * TEXTURE_SIZE + (uint16_t)(tX)]);
            if (i != skipLines && i != TEXTURE_SIZE - skipLines - 1) {
              BSP_LCD_FillRect((rCount * FOV_RECT), tY, FOV_RECT, tYStep + 1);
              tY += tYStep;
            }
            else {
              BSP_LCD_FillRect((rCount * FOV_RECT), (i == skipLines) ? 0 : tY, FOV_RECT, firstLine);
            }
          }
        }
        else {
          tOffset *= -1; // invert value of texture offset to make it positive
          for (uint16_t i = 0; i < TEXTURE_SIZE; i++) {
            BSP_LCD_SetTextColor(_textures[tTextureIndex + hitSide][i * TEXTURE_SIZE + (uint16_t)(tX)]);
            BSP_LCD_FillRect((rCount * FOV_RECT), tOffset + tY, FOV_RECT, tYStep + 1);
            tY += tYStep;
          }

          // DRAW FLOOR
          for (uint16_t i = tOffset + lineHeight; i < SCREEN_HEIGHT; i++) {
            float dY = i - (SCREEN_HEIGHT * 0.5), magic = 186 * TEXTURE_SIZE / dY / _fisheyeCosLUT[rCount];

            tX = _pPosX * 0.5 + _cosLUT[(uint16_t)(rAngle * M_LUT_P)] * magic;
            tY = _pPosY * 0.5 - _sinLUT[(uint16_t)(rAngle * M_LUT_P)] * magic;
            tX = (int16_t)tX & (TEXTURE_SIZE - 1);
            tY = (int16_t)tY & (TEXTURE_SIZE - 1);
            BSP_LCD_SetTextColor(_textures[7][(int16_t)(tY * TEXTURE_SIZE) + (int16_t)tX]);
            BSP_LCD_FillRect((rCount * FOV_RECT), i, FOV_RECT, FOV_RECT);
            BSP_LCD_SetTextColor(_textures[6][(int16_t)(tY * TEXTURE_SIZE) + (int16_t)tX]);
            BSP_LCD_FillRect((rCount * FOV_RECT), SCREEN_HEIGHT - i, FOV_RECT, FOV_RECT);
          }
        }
      }
    }

    rAngle -= FOV_INCR;
    if (rAngle < 0)       { rAngle += M_TWOPI; }
    if (rAngle > M_TWOPI) { rAngle -= M_TWOPI; }
  }

  uint32_t frameTime = _sysElapsedTicks - pStartTime;

  _pMovSpeed = frameTime * INCR_TRANSLATION;
  _pRotSpeed = frameTime * INCR_ROTATION;

  return frameTime;
}
float rayLength(float ax, float bx, float ay, float by) {
  return fsqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
}

float fsqrt(float x){
  if (x == 0) return 0;
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
