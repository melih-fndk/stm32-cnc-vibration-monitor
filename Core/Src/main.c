/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"

#include "arm_math.h"
#include "arm_math_types.h"
#include "dsp/transform_functions.h"
/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIS302DL_CS_PORT GPIOE
#define LIS302DL_CS_PIN  GPIO_PIN_3

#define LIS3DSH_WHO_AM_I_ADDR  0x0F
#define LIS3DSH_CTRL_REG4      0x20

#define LIS3DSH_OUT_X_L        0x28
#define LIS3DSH_OUT_X_H        0x29
#define LIS3DSH_OUT_Y_L        0x2A
#define LIS3DSH_OUT_Y_H        0x2B
#define LIS3DSH_OUT_Z_L        0x2C
#define LIS3DSH_OUT_Z_H        0x2D

#define LIS_READ               0x80
#define LIS_WRITE              0x00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define FFT_SIZE        512
#define SAMPLE_RATE_HZ  1600.0f
#define MAX_ANALYSIS_FREQ_HZ  (SAMPLE_RATE_HZ / 2.0f)
arm_rfft_fast_instance_f32 fftInstance;

float32_t fftInput[FFT_SIZE];
float32_t fftOutput[FFT_SIZE];
float32_t fftMagnitude[FFT_SIZE / 2];

uint16_t fftIndex = 0;

char uartBuffer[200];

uint8_t uartRxByte;
char uartRxBuffer[100];
uint8_t uartRxIndex = 0;

/*
 * Varsayılan rulman bilgileri.
 * Qt arayüzünden CFG komutu gelince bu değerler güncellenecek.
 */
float32_t rpmValue = 60.0f;
float32_t ballCount = 8.0f;
float32_t ballDiameter = 5.0f;
float32_t pitchDiameter = 25.0f;
float32_t contactAngleDeg = 0.0f;

float32_t bpfoFreq = 0.0f;
float32_t bpfiFreq = 0.0f;
float32_t bsfFreq  = 0.0f;
float32_t ftfFreq  = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/



/* USER CODE BEGIN 0 */
uint8_t LIS_ReadReg(uint8_t reg)
{
    uint8_t txData[2];
    uint8_t rxData[2];

    txData[0] = reg | LIS_READ;
    txData[1] = 0x00;

    HAL_GPIO_WritePin(LIS302DL_CS_PORT, LIS302DL_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 100);
    HAL_GPIO_WritePin(LIS302DL_CS_PORT, LIS302DL_CS_PIN, GPIO_PIN_SET);

    return rxData[1];
}

void LIS_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t txData[2];

    txData[0] = reg | LIS_WRITE;
    txData[1] = value;

    HAL_GPIO_WritePin(LIS302DL_CS_PORT, LIS302DL_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, txData, 2, 100);
    HAL_GPIO_WritePin(LIS302DL_CS_PORT, LIS302DL_CS_PIN, GPIO_PIN_SET);
}

void LIS_Init(void)
{
    /*
      0x97 = 1600 Hz ODR
             X, Y, Z eksenleri aktif
    */
    LIS_WriteReg(LIS3DSH_CTRL_REG4, 0x97);
    HAL_Delay(10);
}

int16_t LIS_ReadAxis(uint8_t lowReg, uint8_t highReg)
{
    uint8_t low;
    uint8_t high;
    int16_t value;

    low = LIS_ReadReg(lowReg);
    high = LIS_ReadReg(highReg);

    value = (int16_t)((high << 8) | low);

    return value;
}

void LIS_ReadXYZ(int16_t *x, int16_t *y, int16_t *z)
{
    *x = LIS_ReadAxis(LIS3DSH_OUT_X_L, LIS3DSH_OUT_X_H);
    *y = LIS_ReadAxis(LIS3DSH_OUT_Y_L, LIS3DSH_OUT_Y_H);
    *z = LIS_ReadAxis(LIS3DSH_OUT_Z_L, LIS3DSH_OUT_Z_H);
}




//FFT FONKSIyonlari

void CalculateBearingFaultFrequencies(void)
{
    float32_t fr = rpmValue / 60.0f;
    float32_t angleRad = contactAngleDeg * (3.14159265f / 180.0f);
    float32_t ratio = (ballDiameter / pitchDiameter) * cosf(angleRad);

    ftfFreq  = 0.5f * fr * (1.0f - ratio);
    bpfoFreq = (ballCount / 2.0f) * fr * (1.0f - ratio);
    bpfiFreq = (ballCount / 2.0f) * fr * (1.0f + ratio);
    bsfFreq  = (pitchDiameter / (2.0f * ballDiameter)) * fr * (1.0f - ratio * ratio);
}

float32_t GetMagnitudeNearFrequency(float32_t targetFreq, float32_t bandwidthHz)
{
    if (targetFreq <= 0.0f || targetFreq > MAX_ANALYSIS_FREQ_HZ)
    {
        return -1.0f;
    }

    float32_t maxAmp = 0.0f;

    for (uint16_t k = 1; k < FFT_SIZE / 2; k+=2)
    {
        float32_t freq = ((float32_t)k * SAMPLE_RATE_HZ) / (float32_t)FFT_SIZE;

        if (fabsf(freq - targetFreq) <= bandwidthHz)
        {
            if (fftMagnitude[k] > maxAmp)
            {
                maxAmp = fftMagnitude[k];
            }
        }
    }

    return maxAmp;
}

void ProcessFFTAndSendUART(void)
{
    float32_t mean = 0.0f;
    float32_t rms = 0.0f;

    for (uint16_t i = 0; i < FFT_SIZE; i++)
    {
        mean += fftInput[i];
    }

    mean /= (float32_t)FFT_SIZE;

    for (uint16_t i = 0; i < FFT_SIZE; i++)
    {
        fftInput[i] = fftInput[i] - mean;
        rms += fftInput[i] * fftInput[i];
    }

    rms = sqrtf(rms / (float32_t)FFT_SIZE);

    arm_rfft_fast_f32(&fftInstance, fftInput, fftOutput, 0);

    fftMagnitude[0] = 0.0f;

    for (uint16_t k = 1; k < FFT_SIZE / 2; k++)
    {
        float32_t real = fftOutput[2 * k];
        float32_t imag = fftOutput[2 * k + 1];

        fftMagnitude[k] = sqrtf(real * real + imag * imag) * 2.0f / (float32_t)FFT_SIZE;
    }

    float32_t peakAmp = 0.0f;
    uint16_t peakIndex = 1;

    for (uint16_t k = 1; k < FFT_SIZE / 2; k++)
    {
        if (fftMagnitude[k] > peakAmp)
        {
            peakAmp = fftMagnitude[k];
            peakIndex = k;
        }
    }

    float32_t peakFreq = ((float32_t)peakIndex * SAMPLE_RATE_HZ) / (float32_t)FFT_SIZE;

    float32_t bandwidthHz = (SAMPLE_RATE_HZ / (float32_t)FFT_SIZE) * 3.0f;

    float32_t bpfoAmp = GetMagnitudeNearFrequency(bpfoFreq, bandwidthHz);
    float32_t bpfiAmp = GetMagnitudeNearFrequency(bpfiFreq, bandwidthHz);
    float32_t bsfAmp  = GetMagnitudeNearFrequency(bsfFreq,  bandwidthHz);
    float32_t ftfAmp  = GetMagnitudeNearFrequency(ftfFreq,  bandwidthHz);

    snprintf(uartBuffer,
             sizeof(uartBuffer),
             "FFT,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
             peakFreq,
             peakAmp,
             rms,
             bpfoAmp,
             bpfiAmp,
             bsfAmp,
             ftfAmp);

    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), 100);

    for (uint16_t k = 1; k < FFT_SIZE / 2; k++)
    {
        float32_t freq = ((float32_t)k * SAMPLE_RATE_HZ) / (float32_t)FFT_SIZE;

        snprintf(uartBuffer,
                 sizeof(uartBuffer),
                 "SPEC,%.2f,%.2f\r\n",
                 freq,
                 fftMagnitude[k]);

        HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), 100);
    }
}

void ProcessUARTCommand(char *cmd)
{
    if (strncmp(cmd, "CFG,", 4) == 0)
    {
        char tempBuffer[100];
        char *token;

        float32_t values[5];
        uint8_t valueIndex = 0;

        strncpy(tempBuffer, cmd, sizeof(tempBuffer) - 1);
        tempBuffer[sizeof(tempBuffer) - 1] = '\0';

        token = strtok(tempBuffer, ",");

        while ((token = strtok(NULL, ",")) != NULL && valueIndex < 5)
        {
            values[valueIndex] = (float32_t)atof(token);
            valueIndex++;
        }

        if (valueIndex == 5)
        {
            rpmValue = values[0];
            ballCount = values[1];
            ballDiameter = values[2];
            pitchDiameter = values[3];
            contactAngleDeg = values[4];

            CalculateBearingFaultFrequencies();

            snprintf(uartBuffer,
                     sizeof(uartBuffer),
                     "CFG_OK,RPM=%.2f,N=%.2f,d=%.2f,D=%.2f,A=%.2f\r\n",
                     rpmValue,
                     ballCount,
                     ballDiameter,
                     pitchDiameter,
                     contactAngleDeg);

            HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), 100);

            snprintf(uartBuffer,
                     sizeof(uartBuffer),
                     "FAULT_FREQ,BPFO=%.2f,BPFI=%.2f,BSF=%.2f,FTF=%.2f\r\n",
                     bpfoFreq,
                     bpfiFreq,
                     bsfFreq,
                     ftfFreq);

            HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), 100);
        }
        else
        {
            snprintf(uartBuffer,
                     sizeof(uartBuffer),
                     "ERROR,CFG_PARSE\r\n");

            HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), 100);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (uartRxByte == '\n' || uartRxByte == '\r')
        {
            uartRxBuffer[uartRxIndex] = '\0';

            if (uartRxIndex > 0)
            {
                ProcessUARTCommand(uartRxBuffer);
            }

            uartRxIndex = 0;
        }
        else
        {
            if (uartRxIndex < sizeof(uartRxBuffer) - 1)
            {
                uartRxBuffer[uartRxIndex++] = uartRxByte;
            }
            else
            {
                uartRxIndex = 0;
            }
        }

        HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();



  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LIS302DL_CS_PORT, LIS302DL_CS_PIN, GPIO_PIN_SET);

  uint8_t who_am_i;
  int16_t x, y, z;

  who_am_i = LIS_ReadReg(LIS3DSH_WHO_AM_I_ADDR);

  snprintf(uartBuffer, sizeof(uartBuffer), "WHO_AM_I: 0x%02X\r\n", who_am_i);
  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), 100);

  LIS_Init();

  arm_status fftStatus = arm_rfft_fast_init_f32(&fftInstance, FFT_SIZE);

  if (fftStatus != ARM_MATH_SUCCESS)
  {
      Error_Handler();
  }

  CalculateBearingFaultFrequencies();

  snprintf(uartBuffer,
           sizeof(uartBuffer),
           "FAULT_FREQ,BPFO=%.2f,BPFI=%.2f,BSF=%.2f,FTF=%.2f\r\n",
           bpfoFreq,
           bpfiFreq,
           bsfFreq,
           ftfFreq);

  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), 100);

  HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  LIS_ReadXYZ(&x, &y, &z);

	    float32_t vibration = sqrtf((float32_t)x * x +
	                                (float32_t)y * y +
	                                (float32_t)z * z);

	    fftInput[fftIndex] = vibration;
	    fftIndex++;

	    snprintf(uartBuffer,
	             sizeof(uartBuffer),
	             "RAW,%d,%d,%d\r\n",
	             x,
	             y,
	             z);

	    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), 100);

	    if (fftIndex >= FFT_SIZE)
	    {
	        fftIndex = 0;
	        ProcessFFTAndSendUART();
	    }

	    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	    HAL_Delay(10);

	    /* USER CODE END WHILE */
	    MX_USB_HOST_Process();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 921600;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
