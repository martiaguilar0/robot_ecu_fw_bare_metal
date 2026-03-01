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

TIM_HandleTypeDef htim4;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;




//pid
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
    float out_min;
    float out_max;
} PID_TypeDef;

PID_TypeDef pid_l = {1.0f, 0.1f, 0.01f, 0.0f, 0.0f, 0.0f, 8399.0f};
PID_TypeDef pid_r = {1.0f, 0.1f, 0.01f, 0.0f, 0.0f, 0.0f, 8399.0f};





/* USER CODE BEGIN PV */
volatile uint8_t uart_ready = 1;
uint32_t last_slow_loop = 0;
uint32_t last_led_tick = 0;

uint16_t adc_buffer[2];
uint8_t gps_rx_buffer[256];

typedef struct __attribute__((packed)) {
    uint16_t header;     // 0xAAAA
    uint32_t imu_enc_ts;
    uint32_t mag_bat_ts;
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
volatile uint32_t last_mag_bat_ts = 0;
volatile int32_t snapshot_enc_l = 0;
volatile int32_t snapshot_enc_r = 0;




//for rx data
typedef struct __attribute__((packed)) {
    uint16_t header;      // 0xBBBB para diferenciar de la telemetría
    float target_vel_l;   // Consigna Izquierda
    float target_vel_r;   // Consigna Derecha
    uint8_t checksum;     // Suma de bytes
} Command_t;

Command_t rx_cmd;
volatile float setpoint_l = 0, setpoint_r = 0;
uint8_t raw_rx_buffer[sizeof(Command_t)]; // Buffer intermedio para DMA


int32_t prev_enc_l = 0;
int32_t prev_enc_r = 0;


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
static void MX_TIM4_Init(void);

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

    // 1. Seleccionar Banco 3 para configurar el I2C Maestro del ICM (si usas modo bypass es distinto)
    uint8_t reg = 0x30; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x7F, 1, &reg, 1, 10);

    // 2. Si estás usando el magnetómetro directamente (Modo Bypass), asegúrate de activarlo:
    reg = 0x00; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x7F, 1, &reg, 1, 10); // Volver a Banco 0
    reg = 0x02; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x0F, 1, &reg, 1, 10); // Activar Bypass

    // 3. Configurar Magnetómetro (AK09916) en Modo Continuo 100Hz
    // Registro CNTL2 (0x31) del Magnetómetro -> Valor 0x08 (Modo 4: 100Hz)
    uint8_t mag_init = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MAG_ADDR, 0x31, 1, &mag_init, 1, 10);


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

/* Incluye el resto de tu código (Headers, PV, etc.) hasta llegar al main */

int main(void) {
    HAL_Init();
    SystemClock_Config();

    // 1. Espera de cortesía para estabilidad eléctrica
    HAL_Delay(200);

    // 2. Inicializar GPIO y DMA (Base para todo lo demás)
    MX_GPIO_Init();
    MX_DMA_Init();

    // 3. UARTs (Configurarlas temprano)
    MX_USART2_UART_Init();
    MX_USART6_UART_Init();

    // 4. I2C con Reset para evitar bloqueos del bus
    HAL_I2C_DeInit(&hi2c1);
    HAL_Delay(10);
    MX_I2C1_Init();
    ICM20948_Init();

    // 5. Timers y ADC
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_ADC1_Init();

    /* USER CODE BEGIN 2 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);
    HAL_UART_Receive_DMA(&huart6, gps_rx_buffer, sizeof(gps_rx_buffer));

    uart_ready = 1;
    HAL_UART_Receive_DMA(&huart2, raw_rx_buffer, sizeof(Command_t));

    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_Base_Start_IT(&htim4); // Arranca el PID

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    /* USER CODE END 2 */

    while (1) {
        uint32_t current_time = HAL_GetTick();

        // Blink de diagnóstico
        if (current_time - last_led_tick >= 500) {
            last_led_tick = current_time;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }

        // Lectura de IMU y Telemetría
        if (imu_data_ready_flag == 1) {
            imu_data_ready_flag = 0;
            IMU_Read_Fast();
            if(uart_ready) {
                uart_ready = 0;
                build_telemetry_packet();
                HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&tx_buffer, sizeof(tx_buffer));
            }
        }

        // Lazo lento (Mag y GPS)
        if (current_time - last_slow_loop >= 10) {
            last_slow_loop = current_time;
            IMU_Read_Mag();
            parse_gps_simple();
        }

        // --- WATCHDOG DE UART ---
        static uint32_t last_tx_check = 0;
        if (current_time - last_tx_check > 500) {
            last_tx_check = current_time;
            if (uart_ready == 0) {
                HAL_UART_AbortTransmit(&huart2);
                uart_ready = 1;
            }
        }
    } // <--- AQUÍ FALTABA ESTA LLAVE (Cierre del while)
} // <--- AQUÍ FALTABA ESTA LLAVE (Cierre del main)

/* A partir de aquí, el resto de tus funciones MX_... Init y Callbacks se compilarán correctamente */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8399;                // 84MHz / (8399+1) = 10 kHz
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();

  // --- Aquí inicializamos el módulo PWM ---
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

  // --- Configuración básica PWM CH1 y CH2 ---
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;                     // Duty inicial = 0%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

  // --- Configuración del maestro ---
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

  // --- Break/DeadTime: activar MOE ---
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE; // <-- Habilita MOE automáticamente
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

  // --- Post init para GPIO (si usas HAL_TIM_MspPostInit) ---
  HAL_TIM_MspPostInit(&htim1);
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
  __HAL_RCC_TIM2_CLK_ENABLE(); // Activa el reloj del Timer

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;

  // Configuración Canal 1 (PA15)
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15; // Filtro máximo para evitar ruido eléctrico de 12V

  // Configuración Canal 2 (PB3)
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;

  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM5_Init(void) {
  __HAL_RCC_TIM5_CLK_ENABLE(); // Activa el reloj del Timer

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;

  // Configuración Canal 1 (PA0)
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;

  // Configuración Canal 2 (PA1)
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;

  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
}



static void MX_TIM4_Init(void) {
  __HAL_RCC_TIM4_CLK_ENABLE(); // ¡CRÍTICO! Activa el reloj del periférico

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;      // 84MHz / 84 = 1MHz
  htim4.Init.Period = 999;        // 1MHz / 1000 = 1kHz
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }

  // Configurar la interrupción en el controlador (NVIC)
  HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
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


  // --- ENCODER 1: TIM2 (PA15 y PB3) ---
    // Pin PA15 -> AF1 (TIM2_CH1_ETR)
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Los encoders suelen necesitar Pull-up
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Pin PB3 -> AF1 (TIM2_CH2)
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- ENCODER 2: TIM5 (PA0 y PA1) ---
    // Pins PA0 y PA1 -> AF2 (TIM5_CH1 y CH2)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);





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

  /* --- AQUÍ ESTÁ EL ARREGLO PARA EL PWM --- */
    /* 4. Configurar PA8 y PA9 como salidas del TIM1 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;            // MODO ALTERNATE FUNCTION
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;         // CONECTAR AL TIMER 1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


}

/* --- HELPERS --- */

float PID_Compute(PID_TypeDef *pid, float setpoint, float measured) {
    float error = setpoint - measured;

    // Proporcional
    float P = pid->Kp * error;

    // Integral (con anti-windup simple)
    pid->integral += error;
    float I = pid->Ki * pid->integral;

    // Derivativo
    float D = pid->Kd * (error - pid->last_error);
    pid->last_error = error;

    float output = P + I + D;

    // Saturación (Limitar al rango del Timer PWM)
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    return output;
}




uint8_t calculate_checksum(uint8_t* data, uint16_t len) {
    uint8_t sum = 0;
    for(uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}



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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART2) {
        Command_t *temp = (Command_t*)raw_rx_buffer;

        // 1. Validar Header
        if(temp->header == 0xBBBB) {
            // 2. Calcular checksum de los datos (excluyendo el último byte)
            uint8_t calc = calculate_checksum(raw_rx_buffer, sizeof(Command_t) - 1);

            if(calc == temp->checksum) {
                // 3. Si es correcto, actualizamos las variables que usa el PID
                setpoint_l = temp->target_vel_l;
                setpoint_r = temp->target_vel_r;
            }
        }
        // Nota: Con DMA circular no necesitas re-activar, pero si es modo normal usa:
        // HAL_UART_Receive_DMA(&huart2, raw_rx_buffer, sizeof(Command_t));
    }
}

void build_telemetry_packet(void) {
    tx_buffer.header = 0xAAAA;
    tx_buffer.imu_enc_ts = last_imu_ts;
    tx_buffer.mag_bat_ts = last_mag_bat_ts;
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
  if (htim->Instance == TIM10) {
      HAL_IncTick();
  }

  if (htim->Instance == TIM4) {
      // 1. Obtener posición actual (Lectura directa como uint32_t)
      uint32_t current_l = __HAL_TIM_GET_COUNTER(&htim2);
      uint32_t current_r = __HAL_TIM_GET_COUNTER(&htim5);

      /* 2. Calcular Delta con manejo de Rollover.
       * Al restar dos uint32_t y convertir el resultado a int32_t,
       * el compilador gestiona automáticamente si el contador dio la vuelta.
       * Ejemplo: 0 - 4294967295 resultará en 1. */
      int32_t diff_l = (int32_t)(current_l - (uint32_t)prev_enc_l);
      int32_t diff_r = (int32_t)(current_r - (uint32_t)prev_enc_r);

      // 3. Convertir a velocidad (float) para el PID
      float vel_l = (float)diff_l;
      float vel_r = (float)diff_r;

      // Guardar para la próxima iteración (cast a int32_t para mantener tu tipo de variable)
      prev_enc_l = (int32_t)current_l;
      prev_enc_r = (int32_t)current_r;

      // 4. Ejecutar PID
      //float control_l = PID_Compute(&pid_l, setpoint_l, vel_l);
      //float control_r = PID_Compute(&pid_r, setpoint_r, vel_r);

      // 5. Actualizar PWM (TIM1)
      //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)control_l);
      //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)control_r);

      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)setpoint_l);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)setpoint_r);

  }
}
