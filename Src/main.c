/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili_drv.h"
#include "gears_img.h"
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
/* USER CODE BEGIN PFP */

/* Wrapper functions required by ILI driver */
void gpio_cs_pin (ili_gpio_pin_value_t value);
void gpio_dc_pin (ili_gpio_pin_value_t value);
void gpio_rst_pin (ili_gpio_pin_value_t value);
bool spi_tx_dma_ready (void);
int spi_tx_dma_b (const uint8_t* data, uint32_t len);

/* Procedures for drawing test patterns */
void test_fill_screen(ili_desc_ptr_t display, uint16_t color);
void test_growing_box(ili_desc_ptr_t display);
void test_regions(ili_desc_ptr_t display);
void test_switch_orientation(ili_desc_ptr_t display, ili_orientation_t orientation);
void test_draw_image(ili_desc_ptr_t display, const unsigned char* image, uint32_t size);

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
  ili_desc_ptr_t display;
  const ili_cfg_t display_cfg = {
		  .cs_pin = gpio_cs_pin,
		  .dc_pin = gpio_dc_pin,
		  .rst_pin = gpio_rst_pin,
		  .spi_tx_dma = spi_tx_dma_b,
		  .spi_tx_ready = spi_tx_dma_ready,
		  .orientation = ILI_ORIENTATION_HORIZONTAL,
		  .width = 320,
		  .height = 240,
		  .timeout_ms = 10000,
		  .wup_delay_ms = 20,
		  .restart_delay_ms = 20
  };

  const ili_hw_cfg_t hw_cfg = ili_get_default_hw_cfg(ILI_TYPE_9341);
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
  MX_DMA_Init();
  MX_CRC_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  display = ili_init(&display_cfg, &hw_cfg);
  if (display == NULL) {
	Error_Handler();
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	ili_set_orientation(display, ILI_ORIENTATION_HORIZONTAL_UD);

	test_fill_screen(display, RED);
	HAL_Delay(1000);
	test_fill_screen(display, GREEN);
	HAL_Delay(1000);
	test_fill_screen(display, BLUE);
	HAL_Delay(1000);

	test_fill_screen(display, WHITE);
	test_growing_box(display);

	test_fill_screen(display, BLACK);
	test_regions(display);

	test_fill_screen(display, WHITE);
	test_switch_orientation(display, ILI_ORIENTATION_HORIZONTAL);

	test_fill_screen(display, WHITE);
	test_switch_orientation(display, ILI_ORIENTATION_VERTICAL_UD);

	test_fill_screen(display, WHITE);
	test_switch_orientation(display, ILI_ORIENTATION_HORIZONTAL_UD);

	test_fill_screen(display, WHITE);
	test_switch_orientation(display, ILI_ORIENTATION_VERTICAL);

	test_fill_screen(display, WHITE);
	test_draw_image(display, gears_img, GEARS_IMG_SIZE);
	HAL_Delay(2000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/* USER CODE BEGIN 4 */

void gpio_cs_pin (ili_gpio_pin_value_t value)
{
	HAL_GPIO_WritePin(CSX_GPIO_Port, CSX_Pin, value);
}

void gpio_dc_pin (ili_gpio_pin_value_t value)
{
	HAL_GPIO_WritePin(WRX_DCX_GPIO_Port, WRX_DCX_Pin, value);

}

void gpio_rst_pin (ili_gpio_pin_value_t value)
{
	HAL_GPIO_WritePin(RST_NC_GPIO_Port, RST_NC_Pin, value);
}

bool spi_tx_dma_ready (void)
{
	return HAL_SPI_GetState(&hspi5) == HAL_SPI_STATE_READY;
}

int spi_tx_dma_b (const uint8_t* data, uint32_t len)
{
	/* STM HAL lhas limited data length to 16b */
	int segments = len/65536;
	int rest = len%65536;
	int i;

	for (i = 0; i < segments; i++) {
		HAL_SPI_Transmit_DMA(&hspi5, (unsigned char*)(data+i*65535), 65535);
		while(HAL_SPI_GetState(&hspi5) == HAL_SPI_STATE_BUSY_TX) ;
	}

	HAL_SPI_Transmit_DMA(&hspi5, (unsigned char*)(data+i*65535), rest);

	return 0;
}

void test_fill_screen(ili_desc_ptr_t display, uint16_t color)
{
    coord_2d_t top_left, bottom_right;

	top_left.x = 0;
	top_left.y = 0;
	bottom_right.x = ili_get_screen_width(display);
	bottom_right.y = ili_get_screen_height(display);
	ili_set_region(display, top_left, bottom_right);
	ili_fill_region(display, color);
}

void test_growing_box(ili_desc_ptr_t display)
{
    coord_2d_t top_left, bottom_right;
	uint16_t bg_color = WHITE;
	uint16_t colors[3] = {RED, GREEN, BLUE};

	for (int clr = 0; clr < 3; clr++) {
		for (int size = 10; size < 200; size += 10) {
			top_left.x = 320/2-size/2;
			top_left.y = 240/2-size/2;
			bottom_right.x = 320/2+size/2;
			bottom_right.y = 240/2+size/2;
			ili_set_region(display, top_left, bottom_right);
			ili_fill_region(display, colors[clr]);
			HAL_Delay(100);
		}

		for (int size = 200; size > 0; size -= 10) {
			ili_set_region(display, top_left, bottom_right);
			ili_fill_region(display, bg_color);
			top_left.x = 320/2-size/2;
			top_left.y = 240/2-size/2;
			bottom_right.x = 320/2+size/2;
			bottom_right.y = 240/2+size/2;
			ili_set_region(display, top_left, bottom_right);
			ili_fill_region(display, colors[clr]);
			HAL_Delay(100);
		}
	}
}

void test_regions(ili_desc_ptr_t display)
{
    coord_2d_t top_left, bottom_right;
	uint16_t color;

	top_left.x = 0;
	top_left.y = 0;
	bottom_right.x = 160;
	bottom_right.y = 120;
	color = BLACK;
	ili_set_region(display, top_left, bottom_right);
	ili_fill_region(display, color);
	HAL_Delay(1000);

	top_left.x = 0;
	top_left.y = 121;
	bottom_right.x = 160;
	bottom_right.y = 240;
	color = GREEN;
	ili_set_region(display, top_left, bottom_right);
	ili_fill_region(display, color);
	HAL_Delay(1000);

	top_left.x = 161;
	top_left.y = 0;
	bottom_right.x = 320;
	bottom_right.y = 120;
	color = BLUE;
	ili_set_region(display, top_left, bottom_right);
	ili_fill_region(display, color);
	HAL_Delay(1000);

	top_left.x = 161;
	top_left.y = 121;
	bottom_right.x = 320;
	bottom_right.y = 240;
	color = RED;
	ili_set_region(display, top_left, bottom_right);
	ili_fill_region(display, color);
	HAL_Delay(1000);
}

void test_switch_orientation(ili_desc_ptr_t display, ili_orientation_t orientation)
{
    coord_2d_t top_left, bottom_right;
	uint16_t color;

	ili_set_orientation(display, orientation);
	top_left.x = 0;
	top_left.y = 0;
	bottom_right.x = ili_get_screen_width(display)-20;
	bottom_right.y = ili_get_screen_height(display)-20;;
	color = BLACK;
	ili_set_region(display, top_left, bottom_right);
	ili_fill_region(display, color);
	HAL_Delay(1000);

}

void test_draw_image(ili_desc_ptr_t display, const unsigned char* image, uint32_t size)
{
    coord_2d_t top_left, bottom_right;

	top_left.x = 0;
	top_left.y = 0;
	bottom_right.x = ili_get_screen_width(display);
	bottom_right.y = ili_get_screen_height(display);
	ili_set_region(display, top_left, bottom_right);
	ili_draw_RGB565_dma(display, image, size);
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

  /* Update Display driver timers. */
  ili_1ms_timer_cb();

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
  while (1) ;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
