/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - VERSION SIN ERRORES DE COMPILACION
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
volatile uint8_t uart_ready = 1;
uint32_t last_slow_loop = 0;
uint32_t last_led_tick = 0;

uint16_t adc_buffer[2];
uint8_t gps_rx_buffer[256];

typedef struct __attribute__((packed)) {
    uint16_t header;     // 0xAAAA
    uint32_t imu_ts;
    uint32_t enc_ts;
    int16_t accel[3];
    int16_t gyro[3];
    int16_t magnet[3];
    int32_t enc_l;
    int32_t enc_r;
    int32_t lat;
    int32_t lon;
    uint8_t gps_fix;
    uint16_t batt_v;
    uint16_t batt_curr;
} Telemetry_t;

Telemetry_t tx_buffer;

volatile int16_t imu_accel_x, imu_accel_y, imu_accel_z;
volatile int16_t imu_gyro_x, imu_gyro_y, imu_gyro_z;
volatile int16_t mag_x, mag_y, mag_z;
volatile int32_t gps_lat = 0, gps_lon = 0;
volatile uint8_t gps_fix = 0;

volatile uint8_t imu_data_ready_flag = 0;
volatile uint32_t last_imu_ts = 0;
volatile int32_t snapshot_enc_l = 0;
volatile int32_t snapshot_enc_r = 0;
/* USER CODE END PV */

/* Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);

void build_telemetry_packet(void);
void parse_gps_simple(void);
void ICM20948_Init(void);
void IMU_Read_Fast(void);
void IMU_Read_Mag(void);

/* USER CODE BEGIN 0 */
#define ICM20948_ADDR (0x69 << 1)
#define MAG_ADDR      (0x0C << 1)

void ICM20948_Init() {
    uint8_t data;
    data = 0x00; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x7F, 1, &data, 1, 10);
    data = 0x80; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x06, 1, &data, 1, 10);
    HAL_Delay(50);
    data = 0x01; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x06, 1, &data, 1, 10);
    HAL_Delay(20);
    data = 0x00; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x03, 1, &data, 1, 10);
    data = 0x02; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x0F, 1, &data, 1, 10);
    data = 0x01; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x11, 1, &data, 1, 10);
    data = 0x20; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x7F, 1, &data, 1, 10);
    data = 0x01; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x14, 1, &data, 1, 10);
    data = 0x00; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x7F, 1, &data, 1, 10);
}

void IMU_Read_Fast() {
    uint8_t raw_data[12];
    if(HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDR, 0x2D, 1, raw_data, 12, 5) == HAL_OK) {
        imu_accel_x = (int16_t)(raw_data[0] << 8 | raw_data[1]);
        imu_accel_y = (int16_t)(raw_data[2] << 8 | raw_data[3]);
        imu_accel_z = (int16_t)(raw_data[4] << 8 | raw_data[5]);
        imu_gyro_x  = (int16_t)(raw_data[6] << 8 | raw_data[7]);
        imu_gyro_y  = (int16_t)(raw_data[8] << 8 | raw_data[9]);
        imu_gyro_z  = (int16_t)(raw_data[10] << 8 | raw_data[11]);
    }
}

void IMU_Read_Mag() {
    uint8_t mag_data[6], mag_status;
    if(HAL_I2C_Mem_Read(&hi2c1, MAG_ADDR, 0x10, 1, &mag_status, 1, 1) == HAL_OK) {
        if(mag_status & 0x01) {
            if(HAL_I2C_Mem_Read(&hi2c1, MAG_ADDR, 0x11, 1, mag_data, 6, 2) == HAL_OK) {
                mag_x = (int16_t)(mag_data[1] << 8 | mag_data[0]);
                mag_y = (int16_t)(mag_data[3] << 8 | mag_data[2]);
                mag_z = (int16_t)(mag_data[5] << 8 | mag_data[4]);
            }
            uint8_t dummy;
            HAL_I2C_Mem_Read(&hi2c1, MAG_ADDR, 0x18, 1, &dummy, 1, 1);
        }
    }
}
/* USER CODE END 0 */

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init(); // Ahora sí está definida abajo
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM5_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_ADC1_Init();
    MX_USART6_UART_Init();

    /* USER CODE BEGIN 2 */
    ICM20948_Init();

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);
    HAL_UART_Receive_DMA(&huart6, gps_rx_buffer, sizeof(gps_rx_buffer));
    HAL_TIM_Base_Start(&htim3);

    //HAL_NVIC_DisableIRQ(EXTI0_IRQn);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    /* USER CODE END 2 */

    while (1) {
        uint32_t current_time = HAL_GetTick();

        if (current_time - last_led_tick >= 500) {
            last_led_tick = current_time;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }

        if (imu_data_ready_flag == 1) {
            imu_data_ready_flag = 0;
            IMU_Read_Fast();
            if(uart_ready) {
                uart_ready = 0;
                build_telemetry_packet();
                HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&tx_buffer, sizeof(tx_buffer));
            }
        }

        if (current_time - last_slow_loop >= 100) {
            last_slow_loop = current_time;
            IMU_Read_Mag();
            parse_gps_simple();
        }
    }
}

/* --- CONFIGURACIÓN DE RELOJ --- */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* --- PERIFÉRICOS --- */

static void MX_TIM1_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim1);
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
}

static void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);

    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  HAL_I2C_Init(&hi2c1);
}

static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  HAL_UART_Init(&huart2);
}

static void MX_USART6_UART_Init(void) {
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 38400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  HAL_UART_Init(&huart6);
}

static void MX_TIM3_Init(void) {
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  HAL_TIM_Base_Init(&htim3);
}

static void MX_TIM2_Init(void) {
  TIM_Encoder_InitTypeDef sConfig = {0};
  htim2.Instance = TIM2;
  htim2.Init.Period = 0xFFFFFFFF;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  HAL_TIM_Encoder_Init(&htim2, &sConfig);
}

static void MX_TIM5_Init(void) {
  TIM_Encoder_InitTypeDef sConfig = {0};
  htim5.Instance = TIM5;
  htim5.Init.Period = 0xFFFFFFFF;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  HAL_TIM_Encoder_Init(&htim5, &sConfig);
}

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* --- HELPERS --- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == GPIO_PIN_0) {
        last_imu_ts = __HAL_TIM_GET_COUNTER(&htim3);
        snapshot_enc_l = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        snapshot_enc_r = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        imu_data_ready_flag = 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART2) uart_ready = 1;
}

void build_telemetry_packet(void) {
    tx_buffer.header = 0xAAAA;
    tx_buffer.imu_ts = last_imu_ts;
    tx_buffer.enc_ts = last_imu_ts;
    tx_buffer.enc_l = snapshot_enc_l;
    tx_buffer.enc_r = snapshot_enc_r;
    tx_buffer.accel[0] = imu_accel_x;
    tx_buffer.accel[1] = imu_accel_y;
    tx_buffer.accel[2] = imu_accel_z;
    tx_buffer.gyro[0] = imu_gyro_x;
    tx_buffer.gyro[1] = imu_gyro_y;
    tx_buffer.gyro[2] = imu_gyro_z;
    tx_buffer.magnet[0] = mag_x;
    tx_buffer.magnet[1] = mag_y;
    tx_buffer.magnet[2] = mag_z;
    tx_buffer.batt_v = adc_buffer[0];
    tx_buffer.batt_curr = adc_buffer[1];
    tx_buffer.lat = gps_lat;
    tx_buffer.lon = gps_lon;
    tx_buffer.gps_fix = gps_fix;
}

void parse_gps_simple(void) {
    if (gps_rx_buffer[0] == '$') gps_fix = 1;
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM10) { HAL_IncTick(); }
}
