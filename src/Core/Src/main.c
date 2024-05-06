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
#include "map.h"
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
// PLAYER
static float _pPosX = 1.5, _pPosY = 8.0, _pDeltaX, _pDeltaY, _pVelocityX = 0, _pVelocityY = 0;
static int   _pAngle = 0; // Angle in increments of FOV_INCR radians

// SPRITES
static sprite_typedef _sprites[1] = {
// X | Y | tID | en
  {10.5, 8.0, 0, 1}
};

// SYSTEM
static volatile unsigned int _sysElapsedTicks = 0; // 10K frequency, 1 tick = 100us = 0.1ms
static float _zBuffer[RAYS];

// LOOKUP TABLES
static float        _fisheyeCosLUT[RAYS], _sinLUT[ANG_RANGE], _cosLUT[ANG_RANGE], _fZLUT[SCREEN_HEIGHT / 2];
static unsigned int _sbLUT[ANG_RANGE];

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
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_SelectLayer(LTDC_FOREGROUND);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  for (int i = 0; i < RAYS; i++) { _fisheyeCosLUT[i] = 1 / cos((RAYS / 2 - i) * ANG_INCR); } // Pre-calculate all cosine values to correct fisheye effect later
  for (int i = 0; i < ANG_RANGE; i++) {
    _sinLUT[i] = -sin(i * ANG_INCR + 0.0001); // Sin is inverted because I use screenspace coordinates so Y is inverted
    _cosLUT[i] =  cos(i * ANG_INCR + 0.0001); // Also why I have a separate cos LUT. While it uses more memory, it is faster because it avoids an expensive modulus operation
    _sbLUT[i]  =  (int)(SKYBOX_SIZE_X - (i + 1) * ANG_INCR * SKYBOX_SCALE_F);
  }
  for (int i = SCREEN_HEIGHT / 2; i < SCREEN_HEIGHT; i++) { _fZLUT[i - SCREEN_HEIGHT / 2] = (SCREEN_HEIGHT / 2 / (float)(i - SCREEN_HEIGHT / 2 + 1)) * LINE_VERTICAL_SCALE; }

  HAL_TIM_Base_Start_IT(&htim7);
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

      _pDeltaX = _cosLUT[_pAngle] * P_ACCEL;
      _pDeltaY = _sinLUT[_pAngle] * P_ACCEL;

      if (HAL_GPIO_ReadPin(ARDUINO_D4_GPIO_Port, ARDUINO_D4_Pin)) { // RIGHT
        _pAngle -= P_LOOK_SPEED;
        if (_pAngle < 0) { _pAngle += ANG_RANGE; }
      }
      else if (HAL_GPIO_ReadPin(ARDUINO_D5_GPIO_Port, ARDUINO_D5_Pin)) { // LEFT
        _pAngle += P_LOOK_SPEED;
        if (_pAngle >= ANG_RANGE) { _pAngle -= ANG_RANGE; }
      }
      if (HAL_GPIO_ReadPin(ARDUINO_D2_GPIO_Port, ARDUINO_D2_Pin)) { // FORWARD
        _pVelocityX += _pDeltaX;
        _pVelocityY += _pDeltaY;
      }
      else if (HAL_GPIO_ReadPin(ARDUINO_D3_GPIO_Port, ARDUINO_D3_Pin)) { // BACKWARD
        _pVelocityX -= _pDeltaX;
        _pVelocityY -= _pDeltaY;
      }

      if (_map[MAP_WALLS][(int)_pPosY * MAP_SIZE_X + (int)(_pPosX + ((_pVelocityX < 0) ? -P_HITBOX_SIZE : P_HITBOX_SIZE))] == 0) { _pPosX += _pVelocityX; }
      if (_map[MAP_WALLS][(int)(_pPosY + ((_pVelocityY < 0) ? -P_HITBOX_SIZE : P_HITBOX_SIZE)) * MAP_SIZE_X + (int)_pPosX] == 0) { _pPosY += _pVelocityY; }

      _pVelocityX *= (1 - P_FRICTION);
      _pVelocityY *= (1 - P_FRICTION);
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
int cast(void) {
  // Variable prefix convention: r = ray, m = map, p = performance, c = calculation, t = texture, sb = skybox, f = floor
  unsigned int pStartTime = _sysElapsedTicks;
  int          rAngle;

  BSP_LCD_SelectLayer(LTDC_BACKGROUND);

  rAngle = _pAngle + RAYS / 2;
  if (rAngle >= ANG_RANGE) { rAngle -= ANG_RANGE; }

  for (int rCount = 0; rCount < RAYS; rCount++) {
    // This uses David Ziemkiewicz' method of velocities and times, as well as Lodev's DDA. Massive thanks to both of these legends.
    float       rVelocityX = _cosLUT[rAngle], rVelocityY = _sinLUT[rAngle], rIntersectX = _pPosX, rIntersectY = _pPosY, rTimeX, rTimeY, rLength = 0, tLineHeight;
    int         mX = (short)rIntersectX, mY = (short)rIntersectY; // Casting to short instead of int is faster for some unknown reason.
    signed char rVelSignX = (rVelocityX > 0), rVelSignY = (rVelocityY > 0), rStepX = rVelSignX ? 1 : -1, rStepY = rVelSignY ? 1 : -1, rHitSide = 0;

    for (int i = 0; i < MAP_SIZE_X * MAP_SIZE_Y; i++) {
      if (_map[MAP_WALLS][mY * MAP_SIZE_X + mX] > 0) { break; }

      rTimeX = (mY - rIntersectY + rVelSignY) / rVelocityY; // This should be treated as "time to X-side wall"
      rTimeY = (mX - rIntersectX + rVelSignX) / rVelocityX;

      if (rTimeY < rTimeX) { // Vertical line
        mX += rStepX;
        rIntersectX = mX - !rVelSignX * rStepX;
        rIntersectY += rVelocityY * rTimeY;
        rLength += rTimeY;
        rHitSide = 1;
      }
      else { // Horizontal line
        mY += rStepY;
        rIntersectY = mY - !rVelSignY * rStepY;
        rIntersectX += rVelocityX * rTimeX;
        rLength += rTimeX;
        rHitSide = 0;
      }
    }

    // RENDERING
    tLineHeight = SCREEN_HEIGHT / rLength * _fisheyeCosLUT[rCount] * LINE_VERTICAL_SCALE;

    if (tLineHeight > SHORTEST_LINE) {
      float tX, tY = 0, tYStep = tLineHeight / TEXTURE_SIZE, tOffset = (tLineHeight - SCREEN_HEIGHT) * 0.5;

      if (rHitSide) { tX = (1 - (rIntersectY - (int)rIntersectY)) * TEXTURE_SIZE; if (rAngle < ANG_RANGE / 4 || rAngle > (ANG_RANGE / 4) * 3) { tX = TEXTURE_SIZE - tX; }}
      else          { tX = (1 - (rIntersectX - (int)rIntersectX)) * TEXTURE_SIZE; if (rAngle < ANG_RANGE / 2)                                 { tX = TEXTURE_SIZE - tX; }}

      if (tLineHeight > SCREEN_HEIGHT) { // Check if line fills up the screen, if it does, drawing the sky and floor is not necessary, so we just draw the wall
        if (rCount % 2 == 0) {
          int tSkipLines = tOffset / tYStep, tFirstLine = tYStep - (tOffset - tSkipLines * tYStep);
          tY = tFirstLine;
          for (int i = tSkipLines; i < TEXTURE_SIZE - tSkipLines; i++) {
            BSP_LCD_SetTextColor(_baseTex[_map[MAP_WALLS][mY * MAP_SIZE_X + mX] - 1 + rHitSide][i * TEXTURE_SIZE + (int)(tX)]);
            if (i != tSkipLines && i != TEXTURE_SIZE - tSkipLines - 1) {
              BSP_LCD_FillRect((rCount * RECT_Y), tY, RECT_Y * 2, tYStep + 1);
              tY += tYStep;
            }
            else {
              BSP_LCD_FillRect((rCount * RECT_Y), (i == tSkipLines) ? 0 : tY, RECT_Y * 2, tFirstLine + 2);
            }
          }
        }
      }
      else {
        tOffset *= -1;

        if (rCount % 2 == 0) {
          // DRAW SKYBOX
          int sbY = 0, sbDrawLines = ((int)(tOffset / SKYBOX_TEXEL_Y) + 1) & (TEXTURE_SIZE - 1);
          for (int i = 0; i < sbDrawLines; i++) {
            BSP_LCD_SetTextColor(_skyboxTex[0][i * SKYBOX_SIZE_X + _sbLUT[rAngle]]);
            BSP_LCD_FillRect((rCount * RECT_Y), sbY, RECT_Y * 2, SKYBOX_TEXEL_Y);
            sbY += SKYBOX_TEXEL_Y;
          }

          // DRAW WALLS
          for (int i = 0; i < TEXTURE_SIZE; i++) {
            BSP_LCD_SetTextColor(_baseTex[_map[MAP_WALLS][mY * MAP_SIZE_X + mX] - 1 + rHitSide][i * TEXTURE_SIZE + (int)(tX)]);
            BSP_LCD_FillRect((rCount * RECT_Y), tOffset + tY, RECT_Y * 2, tYStep + 1);
            tY += tYStep;
          }
        }

        // DRAW FLOOR
        for (int i = tOffset + tLineHeight; i < SCREEN_HEIGHT; i += RECT_Y) {
          float fZ = _fZLUT[i - SCREEN_HEIGHT / 2] * _fisheyeCosLUT[rCount], fX = _pPosX + rVelocityX * fZ, fY = _pPosY + rVelocityY * fZ;
          int   fTextureX = (int)(TEXTURE_SIZE * fX) & (TEXTURE_SIZE - 1), fTextureY = (int)(TEXTURE_SIZE * fY) & (TEXTURE_SIZE - 1);
          BSP_LCD_SetTextColor(_baseTex[_map[MAP_FLOOR][(int)fY * MAP_SIZE_X + (int)fX] - 1][fTextureY * TEXTURE_SIZE + fTextureX]);
          BSP_LCD_FillRect((rCount * RECT_Y), i, RECT_Y, RECT_Y);
//          BSP_LCD_FillRect((rCount * FOV_RECT), SCREEN_HEIGHT - i - 1, FOV_RECT, FOV_RECT); // DRAW CEILING
        }
      }
    }

    _zBuffer[rCount] = rLength;
    rAngle--;
    if (rAngle < 0) { rAngle += ANG_RANGE; }
  }

  return _sysElapsedTicks - pStartTime;
}

int drawSprites(void) {
  float dX, dY, pDX, pDY, dot, det, dAng, dist, scale;
  int   column;

  for (int s = 0; s < 1; s++) {
    if (_sprites[s].en) {
      dX     = _pPosX - _sprites[s].x;
      dY     = _pPosY - _sprites[s].y;
      pDX    = _cosLUT[_pAng];
      pDY    = _sinLUT[_pAng];
      dot    = pDX * dX + pDY * dY;
      det    = pDX * dY - pDY * dX;
      dAng   = atan2(det, dot);
      dist   = fsqrt(dX * dX + dY * dY);
      scale  = SPRITE_SIZE / dist; if (scale > SPRITE_SIZE * 8) { continue; }
      column = (RAYS / 2) + (RAYS * dAng) / FOV; if (column < 0 || column > 240) { continue; }
    }
  }

  return 0;
}

float fsqrt(float x) {
  float g = x / 2.0;

  g = 0.5 * (g + x / g);
  g = 0.5 * (g + x / g);
  g = 0.5 * (g + x / g);

  return g;
}

void pageFlip(void) {
  static volatile uint8_t activeBuffer = 1;
  activeBuffer ^= 1;
  BSP_LCD_SWAP(activeBuffer);
}

uint32_t dimColor(uint32_t inputColor, float dimmingFactor) {
  if (dimmingFactor > 1) { return inputColor; }
  return 0xFF000000 |
  (0x00FF0000 & (uint32_t)((0x00FF0000 & inputColor) * dimmingFactor)) |
  (0x0000FF00 & (uint32_t)((0x0000FF00 & inputColor) * dimmingFactor)) |
  (0x000000FF & (uint32_t)((0x000000FF & inputColor) * dimmingFactor));
}

void showFPS(uint32_t frameTime) {
  uint8_t frameTimeS[16];
  uint8_t frameRateS[10];
  sprintf((char*)frameTimeS, "%02li.%lims", frameTime / 10, frameTime % 10);
  sprintf((char*)frameRateS, "%03lifps", 10000 / frameTime);
  BSP_LCD_SetTextColor(0x001F);
  BSP_LCD_DisplayStringAt(0,  0, frameTimeS, LEFT_MODE);
  BSP_LCD_DisplayStringAt(0, 12, frameRateS, LEFT_MODE);
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
