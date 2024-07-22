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
#include "dma.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "flash.h"
#include "stm32f0xx_ll_adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  ADC_BAT,
  ADC_BAT_2,
	ADC_X1,
	ADC_Y1,
  ADC_Y2,
  ADC_X2,
  ADC_REF_INT,
  ADC_CHN_MAX,
} ADC1_CHN;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDRESS 0x59
#define FIRMWARE_VERSION 2
#define APPLICATION_ADDRESS     ((uint32_t)0x08001000)
#define BOOTLOADER_VER_ADDR ((uint32_t)0x08001000 - 4)
uint32_t bootloader_version;

#define ADC_CHANNEL_NUMS                ADC_CHN_MAX
#define ADC_SAMPLES_NUMS                ADC_CHANNEL_NUMS * 20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t i2c_address[1] = {0};
__IO float bat_voltage = 0.0f;
__IO float bat_2_voltage = 0.0f;
__IO int32_t bat_voltage_int = 0;
__IO int32_t bat_2_voltage_int = 0;
__IO uint32_t uiAdcValueBuf[ADC_SAMPLES_NUMS];
__IO uint16_t usAdcValue16[ADC_CHANNEL_NUMS];
__IO uint8_t usAdcValue8[ADC_CHANNEL_NUMS];
__IO uint8_t fm_version = FIRMWARE_VERSION;
volatile uint32_t i2c_stop_timeout_delay = 0;

volatile uint8_t flag_jump_bootloader = 0;
volatile uint32_t jump_bootloader_timeout = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);
	//????????SRAM???
	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  //??? SRAM ??? 0x00000000
  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

void i2c_port_set_to_input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = (GPIO_PIN_9 | GPIO_PIN_10);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void i2c_address_write_to_flash(void) 
{   
  writeMessageToFlash(i2c_address , 1);   
}

void i2c_address_read_from_flash(void) 
{   
  if (!(readPackedMessageFromFlash(i2c_address, 1))) {
    i2c_address[0] = I2C_ADDRESS;
    i2c_address_write_to_flash();
  }
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  long result;

  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (result < out_min)
    result = out_min;
  else if (result > out_max)
    result = out_max;
  
  return result;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  uint64_t adcTotal[ADC_CHANNEL_NUMS]={0};
  
	HAL_ADC_Stop_DMA(hadc);
	for (uint8_t i = 0; i < ADC_SAMPLES_NUMS; i++) {
    adcTotal[i%ADC_CHANNEL_NUMS] += uiAdcValueBuf[i];
  }
	for (uint8_t i = 0; i < ADC_CHANNEL_NUMS; i++) {
    usAdcValue16[i] = (adcTotal[i] / 20);
    usAdcValue8[i] = map(usAdcValue16[i],0,4095,0,255); 
  }
  uint32_t ref_int = __LL_ADC_CALC_VREFANALOG_VOLTAGE(usAdcValue16[ADC_REF_INT], LL_ADC_RESOLUTION_12B);
  bat_voltage_int = usAdcValue16[ADC_BAT] / 4095.0f * ref_int * 2;
  bat_2_voltage_int = usAdcValue16[ADC_BAT_2] / 4095.0f * ref_int * 2;

	HAL_ADC_Start_DMA(hadc, (uint32_t *)uiAdcValueBuf, ADC_SAMPLES_NUMS);
}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len) 
{
  uint8_t rx_buf[16];
  uint8_t tx_buf[32];
  uint8_t rx_mark[16] = {0}; 

  if (len > 1) {
    if (rx_data[0] == 0xFF)
    {
      if (len == 2) {
        if (rx_data[1] < 128) {
          i2c_address[0] = rx_data[1];
          i2c_address_write_to_flash();
          user_i2c_init();
        }
      }
    }
    else if (rx_data[0] == 0xFD)
    {
      if (rx_data[1] == 1) {
        flag_jump_bootloader = 1;
        if (flag_jump_bootloader) {
          LL_I2C_DeInit(I2C1);
          LL_I2C_DisableAutoEndMode(I2C1);
          LL_I2C_Disable(I2C1);
          LL_I2C_DisableIT_ADDR(I2C1);
          HAL_ADC_DeInit(&hadc);
          __HAL_RCC_DMA1_CLK_DISABLE();
          HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
          HAL_ADC_Stop_DMA(&hadc);
          i2c_port_set_to_input();
          while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10))
          {
            jump_bootloader_timeout++;
            if (jump_bootloader_timeout >= 60000) {
              flag_jump_bootloader = 0;
              break;
            }
          }
          if (jump_bootloader_timeout < 60000) {
            NVIC_SystemReset();
          } else {
            // NVIC_SystemReset();
            MX_DMA_Init();
            MX_ADC_Init();          
            user_i2c_init();
            HAL_ADCEx_Calibration_Start(&hadc);
            HAL_ADC_Start_DMA(&hadc, (uint32_t *)uiAdcValueBuf, ADC_SAMPLES_NUMS);
            i2c1_it_enable();        
            jump_bootloader_timeout = 0;
          }
        }        
      }
    }      
  }
  if (len == 1) {
    if (rx_data[0] == 0xFF)
    {
      i2c1_set_send_data(i2c_address, 1); 
    }  
    else if (rx_data[0] == 0xFE)
    {
      i2c1_set_send_data((uint8_t *)&fm_version, 1);
    } 
    else if (rx_data[0] == 0xFC)
    {
      bootloader_version = *(uint8_t*)BOOTLOADER_VER_ADDR;
      i2c1_set_send_data((uint8_t *)&bootloader_version, 1);
    }    
    else if (rx_data[0] <= 0x03)
    {
      memcpy(&tx_buf[0], (uint8_t *)&usAdcValue16[ADC_X1], 2);
      memcpy(&tx_buf[2], (uint8_t *)&usAdcValue16[ADC_Y1], 2);
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0], 0x03-rx_data[0]+1);       
    }      
    else if (rx_data[0] >= 0x10 && rx_data[0] <= 0x11)
    {
      tx_buf[0] = usAdcValue8[ADC_X1];
      tx_buf[1] = usAdcValue8[ADC_Y1];
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x10], 0x11-rx_data[0]+1);       
    }      
    else if (rx_data[0] >= 0x20 && rx_data[0] <= 0x23)
    {
      memcpy(&tx_buf[0], (uint8_t *)&usAdcValue16[ADC_X2], 2);
      memcpy(&tx_buf[2], (uint8_t *)&usAdcValue16[ADC_Y2], 2);
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x20], 0x23-rx_data[0]+1);       
    }      
    else if (rx_data[0] >= 0x30 && rx_data[0] <= 0x31)
    {
      tx_buf[0] = usAdcValue8[ADC_X2];
      tx_buf[1] = usAdcValue8[ADC_Y2];
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x30], 0x31-rx_data[0]+1);       
    }      
    else if (rx_data[0] >= 0x40 && rx_data[0] <= 0x43)
    {
      memcpy(&tx_buf[0], (uint8_t *)&usAdcValue16[ADC_BAT], 2);
      memcpy(&tx_buf[2], (uint8_t *)&usAdcValue16[ADC_BAT_2], 2);
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x40], 0x43-rx_data[0]+1);       
    }    
    else if (rx_data[0] >= 0x50 && rx_data[0] <= 0x51)
    {
      tx_buf[0] = usAdcValue8[ADC_BAT];
      tx_buf[1] = usAdcValue8[ADC_BAT_2];
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x50], 0x51-rx_data[0]+1);       
    }    
    else if (rx_data[0] >= 0x60 && rx_data[0] <= 0x63)
    {
      memcpy(&tx_buf[0], (uint8_t *)&bat_voltage_int, 2);
      memcpy(&tx_buf[2], (uint8_t *)&bat_2_voltage_int, 2);
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x60], 0x63-rx_data[0]+1);       
    }       
    else if (rx_data[0] >= 0x70 && rx_data[0] <= 0x73)
    {
      tx_buf[0] = HAL_GPIO_ReadPin(LEFT_BTN_GPIO_Port, LEFT_BTN_Pin);
      tx_buf[1] = HAL_GPIO_ReadPin(RIGHT_BTN_GPIO_Port, RIGHT_BTN_Pin);
      tx_buf[2] = HAL_GPIO_ReadPin(LEFT_SW_B_GPIO_Port, LEFT_SW_B_Pin);
      tx_buf[3] = HAL_GPIO_ReadPin(RIGHT_SW_B_GPIO_Port, RIGHT_SW_B_Pin);
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x70], 0x73-rx_data[0]+1);       
    }       
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  IAP_Set();
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
  MX_ADC_Init();
  // MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  i2c_address_read_from_flash();
  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)uiAdcValueBuf, ADC_SAMPLES_NUMS);
  user_i2c_init();
  i2c1_it_enable(); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    i2c_timeout_counter = 0;
    if (i2c_stop_timeout_flag) {
      if (i2c_stop_timeout_delay < HAL_GetTick()) {
        i2c_stop_timeout_counter++;
        i2c_stop_timeout_delay = HAL_GetTick() + 10;
      }
    }
    if (i2c_stop_timeout_counter > 50) {
      LL_I2C_DeInit(I2C1);
      LL_I2C_DisableAutoEndMode(I2C1);
      LL_I2C_Disable(I2C1);
      LL_I2C_DisableIT_ADDR(I2C1);     
      user_i2c_init();    
      i2c1_it_enable();
      HAL_Delay(500);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {

  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);
}

/* USER CODE BEGIN 4 */

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
