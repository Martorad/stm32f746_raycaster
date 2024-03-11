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
#include "i2c.h"
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
#include "math.h"
#include "../../Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_lcd.h"
#include "rc_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_3PI_2  4.71238898038468985769
#define FOV_INCR 0.0174533
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
uint32_t cast();
float raylength(float ax, float ay, float bx, float by);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// MAP
uint8_t mapX = 8, mapY = 8, mapS = 64;
uint8_t map[] = {
    1, 1, 1, 1, 1, 1, 1, 1,
    1, 0, 0, 1, 0, 0, 1, 1,
    1, 0, 0, 1, 0, 0, 0, 1,
    1, 0, 0, 0, 0, 0, 0, 1,
    1, 0, 0, 0, 1, 1, 0, 1,
    1, 0, 0, 0, 1, 1, 0, 1,
    1, 0, 0, 0, 0, 0, 0, 1,
    1, 1, 1, 1, 1, 1, 1, 1
};

// PLAYER
float px = 160, py = 352, pdx = 0, pdy = 0, pa = 45 * FOV_INCR; // player X and Y, player delta X and Y and player Angle

// RENDERING
volatile uint8_t displayFlag  = 0;
volatile uint8_t activeBuffer = 1;
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
  MX_I2C1_Init();
  MX_I2C3_Init();
//  MX_LTDC_Init();
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LTDC_FOREGROUND, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerDefaultInit(LTDC_BACKGROUND, LCD_BB_START_ADDRESS);
  BSP_LCD_DisplayOn();

  BSP_LCD_SelectLayer(LTDC_FOREGROUND);
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  for (uint8_t i = 0; i < 40; i++) {
    BSP_LCD_SetTextColor(0xFF000000 | (uint32_t)((0.025 * i) * 0xFF) << 16 | (uint32_t)((0.025 * i) * 0xFF) << 8 | (uint32_t)((0.025 * i) * 0xFF));
    BSP_LCD_FillRect(0 + i * 12, 0 + i * 2, 12, 272 - i * 4);
  }

  BSP_LCD_SelectLayer(LTDC_BACKGROUND);
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  for (uint8_t i = 0; i < 40; i++) {
    BSP_LCD_SetTextColor(0xFF000000 | (uint32_t)((0.025 * i) * 0xFF) << 16 | (uint32_t)((0.025 * i) * 0x00) << 8 | (uint32_t)((0.025 * i) * 0xFF));
    BSP_LCD_FillRect(0 + i * 12, 0 + i * 2, 12, 272 - i * 4);
  }

  HAL_TIM_Base_Start_IT(&htim4);
//  HAL_Delay(3000);

//  BSP_LCD_SelectLayer(LTDC_FOREGROUND);
//  BSP_LCD_Clear(LCD_COLOR_BLACK);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // map

  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    if (displayFlag) {
      uint8_t frameTime[32];
      itoa(cast(), frameTime, 10);
      BSP_LCD_DisplayStringAt(0, 0, frameTime, LEFT_MODE);

      displayFlag = 0;
      activeBuffer ^= 1;
      BSP_LCD_SWAP(activeBuffer);
    }

    if (HAL_GetTick() % 1000 < 500) { HAL_GPIO_WritePin(ARDUINO_SCK_D13_GPIO_Port, ARDUINO_SCK_D13_Pin, GPIO_PIN_RESET); }
    else { HAL_GPIO_WritePin(ARDUINO_SCK_D13_GPIO_Port, ARDUINO_SCK_D13_Pin, GPIO_PIN_SET); }

    if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)) {
      pa -= 0.0001;
      if (pa < 0) { pa = M_TWOPI; }
      pdx = cos(pa) * 5;
      pdy = sin(pa) * 5;
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
  uint16_t r, mx, my, mp, dof; // r is amount of rays, mx and my are map x and y positions, mp is map position in array, dof is how many steps to attempt to cast a ray, before giving up
  float    rx, ry, ra, xo, yo, shrt; // rx and ry are the first intersect points, ra is ray angle, xo and yo are x and y offset or step, shrt is the shortest ray length
  uint32_t startTime = HAL_GetTick();

  ra = pa - 30 * FOV_INCR;
  if (ra < 0)       { ra += M_TWOPI; }
  if (ra > M_TWOPI) { ra -= M_TWOPI; }

  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  for (r = 0; r < 60; r++) {
    // HORIZONTAL LINE CHECK
    dof = 0;
    float dH = 100000, hx = px, hy = py;
    float aTan = -1 / tan(ra);

    // looking up
    if (ra < M_PI) {
      ry = (((uint16_t)py >> 6) << 6) + 64; // essentially acts as floor(py) + 64 to find next horizontal intersection of y
      rx = (py - ry) * aTan + px;           // some weird signage stuff, but essentially find the y distance between ry and py, divide by the tangent to get xn and add px to get rx
      yo = 64;                              // yo is equal to cellsize
      xo = yo * aTan;                       // xo is equal to the ratio of the tangent
    }

    // looking down
    if (ra > M_PI) {
      ry = (((uint16_t)py >> 6) << 6);
      rx = (py - ry) * aTan + px;
      yo = -64;
      xo = yo * aTan;
    }

    // looking perfectly horizontal
    if (ra == 0 || ra == M_PI) {
      ry = py;
      rx = px;
      dof = DOF;
    }

    while (dof < DOF) {
      mx = (uint16_t)rx >> 6; // divide ray's position to figure out the map square to check
      my = (uint16_t)ry >> 6;
      mp = my * mapX + mx;    // find that map square's position in the array

      if (mp > 0 && mp < mapX * mapY && map[mp] == 1) {
        hx = rx;
        hy = ry;
        dH = raylength(px, py, hx, hy); // calculate delta between hit x,y and player x,y, use pythagoras to get hypotenuse length
        dof = DOF;
      }
      else {
        rx += xo;
        ry += yo;
        dof++;
      }
    }

    // VERTICAL LINE CHECK
    dof = 0;
    float dV = 100000, vx = px, vy = py;
    float nTan = -tan(ra);

    // looking left
    if (ra > M_PI_2 && ra < M_3PI_2) {
      rx = (((uint16_t)px >> 6) << 6);
      ry = (px - rx) * nTan + py;
      xo = -64;
      yo = xo * nTan;
    }

    // looking right
    if (ra < M_PI_2 || ra > M_3PI_2) {
      rx = (((uint16_t)px >> 6) << 6) + 64;
      ry = (px - rx) * nTan + py;
      xo = 64;
      yo = xo * nTan;
    }

    // looking perfectly horizontal
    if (ra == 0 || ra == M_PI) {
      ry = py;
      rx = px;
      dof = DOF;
    }

    while (dof < DOF) {
      mx = (uint16_t)rx >> 6;
      my = (uint16_t)ry >> 6;
      mp = my * mapX + mx;

      if (mp > 0 && mp < mapX * mapY && map[mp] == 1) {
        vx = rx;
        vy = ry;
        dV = raylength(px, py, vx, vy);
        dof = DOF;
      }
      else {
        rx += xo;
        ry += yo;
        dof++;
      }
    }

    if (dV < dH) {
      shrt = dV;
      rx = vx;
      ry = vy;
    }
    if (dH < dV) {
      shrt = dH;
      rx = hx;
      ry = hy;
    }

    // Rendering
    float lineH = (mapS * 272) / shrt;
    if (lineH > 272) { lineH = 272; }

    float lineO = (272 - lineH) / 2;

    BSP_LCD_SetTextColor(0xFF000000 | (uint32_t)((0.0036 * lineH) * 0xFF) << 16 | (uint32_t)((0.0036 * lineH) * 0xFF) << 8 | (uint32_t)((0.0036 * lineH) * 0xFF));
    BSP_LCD_FillRect((r * 8), lineO, 8, lineH);

    ra += FOV_INCR;
    if (ra < 0)       { ra += M_TWOPI; }
    if (ra > M_TWOPI) { ra -= M_TWOPI; }
  }

  return (HAL_GetTick() - startTime);
}

float raylength(float ax, float ay, float bx, float by) {
  return sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
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
  if (htim->Instance == TIM4) {
    displayFlag = 1;
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
