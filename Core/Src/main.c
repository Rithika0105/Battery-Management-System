/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F411RE Dual-Battery BMS - Voltage, Current (ACS712),
  *                   ADC Calibration, Power Telemetry & Safety Monitoring
  * @author         : Electronics and Communication Engineering BMS Project
  ******************************************************************************
  * @details
  * Target Hardware : STM32F411RE Nucleo-64 Board
  * System Clock    : 16 MHz HSI (or 84 MHz PLL)
  *
  * Hardware & Voltage Divider Parameters:
  * ----------------------------------------------------------------------------
  * Channel   | Resistors (R1, R2, R3) | Ratio | Full-Scale Input | Max Pin Voltage | Max ADC Count
  * ----------------------------------------------------------------------------
  * Battery 1 | 100 kOhm, 36 kOhm      | 3.7778| 12.000 V         | 3.176 V         | 3941 counts
  * Pack/Bat2 | 223 kOhm, 36 kOhm      | 7.1944| 24.000 V         | 3.336 V         | 4095 (Saturated)
  * ACS712-30A| 15k + 15k, 15k (3-Res) | 3.0000| +/-30.00 A       | 1.493 V (+30A)  | 1853 counts
  * ----------------------------------------------------------------------------
  *
  * Hardware Pin Mapping:
  * ----------------------------------------------------------------------------
  * Pin  | Function       | Connection / Description
  * ----------------------------------------------------------------------------
  * PA0  | ADC1_IN0       | Battery 1 Tap (Node 1) via Divider (100k / 36k)
  * PA1  | ADC1_IN1       | Total Pack Tap (Node 2) via Divider (223k / 36k)
  * PA4  | ADC1_IN4       | ACS712 OUT via 3x 15k Divider (15k+15k : 15k)
  * PA2  | USART2_TX      | ST-LINK Virtual COM Port TX (115200 Baud, 8N1)
  * PA3  | USART2_RX      | ST-LINK Virtual COM Port RX
  * ----------------------------------------------------------------------------
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define VREF_VOLTAGE             (3.300f)   /* Nominal ADC Reference Voltage (V) */
#define ADC_MAX_COUNT            (4095.0f)  /* 12-bit ADC Maximum Resolution */
#define ADC_OVERSAMPLE_COUNT     (64U)      /* 64x Digital Oversampling Filter */

/* Battery 1 Voltage Divider Resistors (12V Max Range) */
#define BAT1_R1                  (100.0f)   /* Upper Resistor: 100 kOhm */
#define BAT1_R2                  (36.0f)    /* Lower Resistor: 36 kOhm */
#define BAT1_DIV_RATIO           ((BAT1_R1 + BAT1_R2) / BAT1_R2) /* 136/36 = 3.777778 */

/* Total Pack / Battery 2 Voltage Divider Resistors (24V Max Range) */
#define PACK_R1                  (223.0f)   /* Upper Resistor: 223 kOhm */
#define PACK_R2                  (36.0f)    /* Lower Resistor: 36 kOhm */
#define PACK_DIV_RATIO           ((PACK_R1 + PACK_R2) / PACK_R2) /* 259/36 = 7.194444 */

/* Practical Multimeter Calibration Gain Factors */
/* PA1 reads 3.095V -> raw pack = 3.095 x 7.1944 = 22.267V */
/* Multimeter actual = 22.60V -> Gain = 22.60 / 22.267 = 1.0150 */
#define CALIB_GAIN_BAT1          (1.0000f)  /* Tuning factor for Battery 1 */
#define CALIB_GAIN_PACK          (1.0150f)  /* PA1=3.095V: 22.267V x 1.015 = 22.60V */

/* Disconnected / Floating Pin Noise Filter Threshold */
#define ADC_DISCONNECT_THRESHOLD (1400U)    /* < 1.12V on ADC pin -> 0.00V */

/* ACS712 30A Current Sensor Specifications */
#define ACS712_SENSITIVITY       (0.066f)   /* 66 mV/A for 30A version (V/A) */
#define ACS712_NOMINAL_ZERO_V    (2.500f)   /* Nominal 0A voltage at 5V VCC (V) */
#define ACS712_R1                (15.0f)    /* Resistor 1: 15 kOhm */
#define ACS712_R2                (15.0f)    /* Resistor 2: 15 kOhm */
#define ACS712_R3                (15.0f)    /* Resistor 3: 15 kOhm */
#define ACS712_DIV_RATIO         ((ACS712_R1 + ACS712_R2 + ACS712_R3) / ACS712_R3) /* 45/15 = 3.0000 */
#define CURRENT_NOISE_DEADBAND   (0.050f)   /* 50 mA noise threshold */
/* Disconnected ACS712 Threshold:
 * Quiescent 0A is ~1034 counts (0.833V).
 * When disconnected and pulled down to GND, PA4 reads near 0V (< 250 counts).
 * Any reading < 250 counts indicates the current sensor is NOT connected. */
#define ACS712_DISCONNECT_THRESHOLD (250U)  /* < 0.20V on PA4 pin -> Sensor Disconnected */

/* Battery Safety Limits */
#define BAT1_MAX_LIMIT           (12.00f)   /* 12V Max for Battery 1 */
#define BAT1_MIN_LIMIT           (8.40f)    /* 8.4V Cutoff */
#define PACK_MAX_LIMIT           (24.00f)   /* 24V Max for Total Pack */
#define PACK_MIN_LIMIT           (16.80f)   /* 16.8V Cutoff */
#define CURRENT_MAX_LIMIT        (30.00f)   /* 30A Maximum Current Limit */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

typedef struct {
  /* Voltage Channels */
  uint16_t raw_adc0;            /* Raw ADC Count for PA0 (Battery 1) */
  uint16_t raw_adc1;            /* Raw ADC Count for PA1 (Total Pack) */
  float    v_adc0;              /* Measured Voltage at PA0 Pin (Vout1) */
  float    v_adc1;              /* Measured Voltage at PA1 Pin (Vout2) */
  float    v_bat1;              /* Measured Battery 1 Voltage (V) */
  float    v_pack;              /* Measured Total Series-Pack Voltage (V) */
  float    v_bat2;              /* Calculated Battery 2 Voltage = V_pack - V_bat1 (V) */
  float    v_imbalance;         /* Imbalance = |V_bat1 - V_bat2| (V) */
  bool     bat1_connected;      /* True if Battery 1 is connected */
  bool     pack_connected;      /* True if Pack is connected */
  bool     adc0_saturated;      /* True if PA0 is near/at saturation (>3.28V) */
  bool     adc1_saturated;      /* True if PA1 is near/at saturation (>3.28V) */

  /* Current & Power Channels */
  uint16_t raw_adc4;            /* Raw ADC Count for PA4 (ACS712 Current) */
  float    v_adc4;              /* Measured Voltage at PA4 Pin (V) */
  float    v_sensor_raw;        /* Reconstructed ACS712 OUT Voltage (V) */
  float    v_zero_offset;       /* Calibrated Zero-Current Sensor Offset (V) */
  float    current_amps;        /* Measured Battery Current (+ Discharging, - Charging) (A) */
  float    power_watts;         /* Instantaneous Battery Power = V_pack * Current (W) */
  char     current_state[16];   /* "CHARGING", "DISCHARGING", "IDLE", "DISCONNECTED" */
  bool     current_sensor_connected; /* True if ACS712 is physically connected */
} BMS_DualBattery_Data_t;

BMS_DualBattery_Data_t bms_data;

/* Function Prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void BMS_ADC_Init(void);
uint16_t BMS_ADC_ReadChannel(uint8_t channel);
void BMS_CalibrateCurrentSensor(BMS_DualBattery_Data_t *bms);
void BMS_ProcessSensors(BMS_DualBattery_Data_t *bms);
void BMS_PrintTelemetry(const BMS_DualBattery_Data_t *bms);
void UART_SendString(const char *str);
void Format_Float(char *out_buf, size_t max_len, float val, int decimals);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes Flash interface and Systick */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize GPIO and UART2 peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* Initialize ADC1 for Multi-Channel Reading (PA0, PA1 & PA4) */
  BMS_ADC_Init();

  /* Welcome Banner */
  UART_SendString("\r\n========================================================\r\n");
  UART_SendString(" STM32F411RE Nucleo - Dual Battery & Current BMS Engine \r\n");
  UART_SendString(" 12V/24V Dual Pack | ACS712 30A Sensor | UART Telemetry \r\n");
  UART_SendString("========================================================\r\n");

  /* Zero-Current Offset Auto-Calibration */
  UART_SendString("[i] Checking ACS712 Current Sensor (PA4)...\r\n");
  BMS_CalibrateCurrentSensor(&bms_data);

  if (bms_data.current_sensor_connected)
  {
    char cal_msg[64];
    char str_offset[16];
    Format_Float(str_offset, sizeof(str_offset), bms_data.v_zero_offset, 3);
    snprintf(cal_msg, sizeof(cal_msg), "[OK] ACS712 Detected & Calibrated Zero-Offset: %s V\r\n\r\n", str_offset);
    UART_SendString(cal_msg);
  }
  else
  {
    UART_SendString("[i] ACS712 Sensor: NOT CONNECTED (Internal Pull-Down to GND Active -> 0.000 A)\r\n\r\n");
  }

  uint32_t sample_counter = 0;

  /* Main Infinite Loop */
  while (1)
  {
    /* 1. Acquire, filter, and calculate all voltages, currents & power */
    BMS_ProcessSensors(&bms_data);

    /* 2. Format and Transmit Telemetry over UART */
    sample_counter++;
    UART_SendString("--- [BMS SAMPLE #");
    char count_str[16];
    snprintf(count_str, sizeof(count_str), "%lu", (unsigned long)sample_counter);
    UART_SendString(count_str);
    UART_SendString("] -----------------------------------\r\n");

    BMS_PrintTelemetry(&bms_data);

    /* 3. Telemetry Interval: 1000 ms (1 Hz Refresh Rate) */
    HAL_Delay(1000);
  }
}

/**
  * @brief  Initialize ADC1 for PA0 (CH0), PA1 (CH1), and PA4 (CH4) with 480-cycle sampling
  * @retval None
  */
void BMS_ADC_Init(void)
{
  /* 1. Enable Clocks for GPIOA and ADC1 */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  /* 2. Configure PA0, PA1, and PA4 in Analog Mode (0b11) */
  GPIOA->MODER |= (0x3UL << (0 * 2)) | (0x3UL << (1 * 2)) | (0x3UL << (4 * 2));
  
  /* Configure PUPDR:
   * PA0, PA1: No pull-up/pull-down (already pulled to ground via external resistor dividers)
   * PA4: Internal Pull-Down (0b10) to GND to firmly ground pin and prevent floating when unplugged
   */
  GPIOA->PUPDR &= ~((0x3UL << (0 * 2)) | (0x3UL << (1 * 2)) | (0x3UL << (4 * 2)));
  GPIOA->PUPDR |=  (0x2UL << (4 * 2)); /* PA4 = Internal Pull-Down */

  /* 3. Set ADC Prescaler: PCLK2 / 4 */
  ADC->CCR &= ~ADC_CCR_ADCPRE;
  ADC->CCR |= ADC_CCR_ADCPRE_0;

  /* 4. Set Sampling Time on CH0, CH1, CH4 to 480 cycles (SMPR2) for high impedance */
  ADC1->SMPR2 |= (0x7UL << (0 * 3)); /* CH0: 480 cycles */
  ADC1->SMPR2 |= (0x7UL << (1 * 3)); /* CH1: 480 cycles */
  ADC1->SMPR2 |= (0x7UL << (4 * 3)); /* CH4: 480 cycles */

  /* 5. 12-bit Resolution, Single conversion mode */
  ADC1->CR1 = 0;
  ADC1->CR2 = 0;

  /* 6. Enable ADC1 (ADON bit) */
  ADC1->CR2 |= ADC_CR2_ADON;

  /* Stabilization delay */
  HAL_Delay(10);
}

/**
  * @brief  Read ADC channel with 64x digital oversampling and averaging
  * @param  channel: ADC Channel (0 for PA0, 1 for PA1, 4 for PA4)
  * @retval 12-bit averaged ADC raw count (0 - 4095)
  */
uint16_t BMS_ADC_ReadChannel(uint8_t channel)
{
  uint32_t accumulator = 0;

  /* Set 1 conversion in regular sequence (L = 0) on target channel */
  ADC1->SQR1 = 0;
  ADC1->SQR3 = (channel & 0x1F);

  for (uint32_t i = 0; i < ADC_OVERSAMPLE_COUNT; i++)
  {
    /* Clear status register */
    ADC1->SR = 0;

    /* Start conversion */
    ADC1->CR2 |= ADC_CR2_SWSTART;

    /* Wait for End of Conversion (EOC) */
    while (!(ADC1->SR & ADC_SR_EOC));

    accumulator += (uint16_t)(ADC1->DR & 0x0FFF);
  }

  return (uint16_t)(accumulator / ADC_OVERSAMPLE_COUNT);
}

/**
  * @brief  Calibrate ACS712 zero-current offset voltage at startup (averages 256 samples)
  * @param  bms: Pointer to dual-battery data structure
  * @retval None
  */
void BMS_CalibrateCurrentSensor(BMS_DualBattery_Data_t *bms)
{
  uint32_t cal_accumulator = 0;
  const uint32_t cal_samples = 64;

  for (uint32_t i = 0; i < cal_samples; i++)
  {
    cal_accumulator += BMS_ADC_ReadChannel(4);
    HAL_Delay(2);
  }

  uint16_t avg_raw = (uint16_t)(cal_accumulator / cal_samples);

  /* Check if sensor is disconnected (pin pulled down to GND < 250 counts) */
  if (avg_raw < ACS712_DISCONNECT_THRESHOLD)
  {
    bms->current_sensor_connected = false;
    bms->v_zero_offset = ACS712_NOMINAL_ZERO_V;
    bms->current_amps = 0.0f;
    bms->power_watts = 0.0f;
    strcpy(bms->current_state, "DISCONNECTED");
    return;
  }

  bms->current_sensor_connected = true;
  float v_pin = ((float)avg_raw / ADC_MAX_COUNT) * VREF_VOLTAGE;
  bms->v_zero_offset = v_pin * ACS712_DIV_RATIO;

  /* Sanity check: If zero offset is out of range, fallback to 2.50V */
  if (bms->v_zero_offset < 2.20f || bms->v_zero_offset > 2.80f)
  {
    bms->v_zero_offset = ACS712_NOMINAL_ZERO_V;
  }
}

/**
  * @brief  Process raw ADC readings, apply scaling factors, calculate current & power
  * @param  bms: Pointer to dual-battery data structure
  * @retval None
  */
void BMS_ProcessSensors(BMS_DualBattery_Data_t *bms)
{
  /* 1. Read 64x oversampled raw ADC counts */
  bms->raw_adc0 = BMS_ADC_ReadChannel(0); /* PA0 -> Battery 1 */
  bms->raw_adc1 = BMS_ADC_ReadChannel(1); /* PA1 -> Total Pack */
  bms->raw_adc4 = BMS_ADC_ReadChannel(4); /* PA4 -> ACS712 Current */

  /* 2. Process Battery 1 Tap (PA0) */
  if (bms->raw_adc0 < ADC_DISCONNECT_THRESHOLD)
  {
    bms->v_adc0 = 0.0f;
    bms->v_bat1 = 0.0f;
    bms->bat1_connected = false;
    bms->adc0_saturated = false;
  }
  else
  {
    bms->bat1_connected = true;
    bms->v_adc0 = ((float)bms->raw_adc0 / ADC_MAX_COUNT) * VREF_VOLTAGE;
    bms->v_bat1 = bms->v_adc0 * BAT1_DIV_RATIO * CALIB_GAIN_BAT1;
    bms->adc0_saturated = (bms->v_adc0 >= 3.28f);
  }

  /* 3. Process Total Pack Tap (PA1) */
  if (bms->raw_adc1 < ADC_DISCONNECT_THRESHOLD)
  {
    bms->v_adc1 = 0.0f;
    bms->v_pack = 0.0f;
    bms->v_bat2 = 0.0f;
    bms->pack_connected = false;
    bms->adc1_saturated = false;
  }
  else
  {
    bms->pack_connected = true;
    bms->v_adc1 = ((float)bms->raw_adc1 / ADC_MAX_COUNT) * VREF_VOLTAGE;
    bms->v_pack = bms->v_adc1 * PACK_DIV_RATIO * CALIB_GAIN_PACK;
    bms->adc1_saturated = (bms->v_adc1 >= 3.28f);

    /* 4. Calculate Individual Battery 2 Voltage: V_bat2 = V_pack - V_bat1 */
    if (bms->bat1_connected)
    {
      bms->v_bat2 = bms->v_pack - bms->v_bat1;
      if (bms->v_bat2 < 0.0f) bms->v_bat2 = 0.0f;
    }
    else
    {
      bms->v_bat2 = bms->v_pack;
    }
  }

  /* 5. Calculate Imbalance between Battery 1 and Battery 2 */
  /* Only compute imbalance when both batteries have valid positive readings */
  if (bms->bat1_connected && bms->pack_connected && (bms->v_bat2 > 0.0f))
  {
    bms->v_imbalance = (bms->v_bat1 > bms->v_bat2) ? (bms->v_bat1 - bms->v_bat2) : (bms->v_bat2 - bms->v_bat1);
  }
  else
  {
    bms->v_imbalance = 0.0f;
  }

  /* 6. Process ACS712 Current Sensor (PA4) */
  if (bms->raw_adc4 < ACS712_DISCONNECT_THRESHOLD)
  {
    /* Sensor NOT connected: Pin pulled down to GND (0V) */
    bms->v_adc4 = 0.0f;
    bms->v_sensor_raw = 0.0f;
    bms->current_amps = 0.0f;
    bms->power_watts = 0.0f;
    bms->current_sensor_connected = false;
    strcpy(bms->current_state, "DISCONNECTED");
  }
  else
  {
    bms->current_sensor_connected = true;
    bms->v_adc4 = ((float)bms->raw_adc4 / ADC_MAX_COUNT) * VREF_VOLTAGE;
    bms->v_sensor_raw = bms->v_adc4 * ACS712_DIV_RATIO;

    /* Current formula: I = (V_sensor - V_zero) / Sensitivity */
    float raw_current = (bms->v_sensor_raw - bms->v_zero_offset) / ACS712_SENSITIVITY;

    /* Apply Noise Deadband Filter */
    if (fabsf(raw_current) < CURRENT_NOISE_DEADBAND)
    {
      bms->current_amps = 0.0f;
      strcpy(bms->current_state, "IDLE");
    }
    else
    {
      bms->current_amps = raw_current;
      if (bms->current_amps > 0.0f)
      {
        strcpy(bms->current_state, "DISCHARGING");
      }
      else
      {
        strcpy(bms->current_state, "CHARGING");
      }
    }

    /* 7. Calculate Instantaneous Power: P = V_pack * I (Watts) */
    if (bms->pack_connected)
    {
      bms->power_watts = bms->v_pack * bms->current_amps;
    }
    else
    {
      bms->power_watts = 0.0f;
    }
  }
}

/**
  * @brief  Robust float-to-string formatter (independent of newlib-nano printf limitation)
  * @param  out_buf: Target character buffer
  * @param  max_len: Buffer size
  * @param  val: Floating-point number (can be negative)
  * @param  decimals: Number of decimal places (1, 2, 3, or 4)
  * @retval None
  */
void Format_Float(char *out_buf, size_t max_len, float val, int decimals)
{
  char sign[2] = "";
  if (val < 0.0f) {
    sign[0] = '-';
    sign[1] = '\0';
    val = -val;
  }

  int int_part = (int)val;
  float remainder = val - (float)int_part;

  if (decimals == 4)
  {
    int frac_part = (int)(remainder * 10000.0f + 0.5f);
    if (frac_part >= 10000) { int_part++; frac_part = 0; }
    snprintf(out_buf, max_len, "%s%d.%04d", sign, int_part, frac_part);
  }
  else if (decimals == 3)
  {
    int frac_part = (int)(remainder * 1000.0f + 0.5f);
    if (frac_part >= 1000) { int_part++; frac_part = 0; }
    snprintf(out_buf, max_len, "%s%d.%03d", sign, int_part, frac_part);
  }
  else /* Default 2 decimals */
  {
    int frac_part = (int)(remainder * 100.0f + 0.5f);
    if (frac_part >= 100) { int_part++; frac_part = 0; }
    snprintf(out_buf, max_len, "%s%d.%02d", sign, int_part, frac_part);
  }
}

/**
  * @brief  Print formatted telemetry to UART serial monitor
  * @param  bms: Pointer to dual-battery data structure
  * @retval None
  */
void BMS_PrintTelemetry(const BMS_DualBattery_Data_t *bms)
{
  char buffer[128];
  char str_vadc0[16], str_vadc1[16];
  char str_vbat1[16], str_vbat2[16], str_vpack[16], str_vimb[16];
  char str_curr[16], str_power[16];

  Format_Float(str_vadc0, sizeof(str_vadc0), bms->v_adc0, 3);
  Format_Float(str_vadc1, sizeof(str_vadc1), bms->v_adc1, 3);
  Format_Float(str_vbat1, sizeof(str_vbat1), bms->v_bat1, 3);
  Format_Float(str_vbat2, sizeof(str_vbat2), bms->v_bat2, 3);
  Format_Float(str_vpack, sizeof(str_vpack), bms->v_pack, 3);
  Format_Float(str_vimb,  sizeof(str_vimb),  bms->v_imbalance, 3);
  Format_Float(str_curr,  sizeof(str_curr),  bms->current_amps, 3);
  Format_Float(str_power, sizeof(str_power), fabsf(bms->power_watts), 3);

  /* 1. Raw ADC Counts (original format) */
  snprintf(buffer, sizeof(buffer), " Raw ADC Counts       : CH0(PA0)=%4u  | CH1(PA1)=%4u\r\n",
           bms->raw_adc0, bms->raw_adc1);
  UART_SendString(buffer);

  /* 2. Divider Vout Pin Voltages (original format) */
  snprintf(buffer, sizeof(buffer), " Divider Vout (Pins)  : Vout1(PA0) = %s V | Vout2(PA1) = %s V\r\n",
           str_vadc0, str_vadc1);
  UART_SendString(buffer);

  /* 3. Battery 1 Voltage (original format) */
  if (bms->bat1_connected) {
    snprintf(buffer, sizeof(buffer), " Battery 1 Voltage    : %s V (Divider Ratio: 3.7778)\r\n", str_vbat1);
  } else {
    snprintf(buffer, sizeof(buffer), " Battery 1 Voltage    : 0.000 V [0V / DISCONNECTED]\r\n");
  }
  UART_SendString(buffer);

  /* 4. Battery 2 Voltage & Total Pack (original format) */
  if (bms->pack_connected) {
    snprintf(buffer, sizeof(buffer), " Battery 2 Voltage    : %s V (Calculated: V_pack - V_bat1)\r\n", str_vbat2);
    UART_SendString(buffer);
    snprintf(buffer, sizeof(buffer), " Total Pack Voltage   : %s V (Divider Ratio: 7.1944)\r\n", str_vpack);
    UART_SendString(buffer);
  } else {
    snprintf(buffer, sizeof(buffer), " Battery 2 Voltage    : 0.000 V [0V / DISCONNECTED]\r\n");
    UART_SendString(buffer);
    snprintf(buffer, sizeof(buffer), " Total Pack Voltage   : 0.000 V [0V / DISCONNECTED]\r\n");
    UART_SendString(buffer);
  }

  /* 5. Cell Imbalance (original format) */
  snprintf(buffer, sizeof(buffer), " Cell Imbalance       : %s V\r\n", str_vimb);
  UART_SendString(buffer);

  /* 6. ADDED: Current & Power from ACS712 (after Cell Imbalance) */
  if (bms->current_sensor_connected) {
    snprintf(buffer, sizeof(buffer), " Battery Current      : %s A [%s]\r\n", str_curr, bms->current_state);
    UART_SendString(buffer);

    snprintf(buffer, sizeof(buffer), " Instantaneous Power  : %s W\r\n", str_power);
    UART_SendString(buffer);
  } else {
    UART_SendString(" Battery Current      : 0.000 A [DISCONNECTED]\r\n");
    UART_SendString(" Instantaneous Power  : 0.000 W [DISCONNECTED]\r\n");
  }

  /* 7. System Status (original format) */
  UART_SendString(" System Status        : ");
  if (!bms->bat1_connected && !bms->pack_connected) {
    UART_SendString("[i] IDLE - NO BATTERIES CONNECTED (0.00V)\r\n");
  } else if (bms->adc1_saturated) {
    UART_SendString("[!] WARNING: PA1 (PACK) SATURATED (>23.74V)!\r\n");
  } else if (bms->v_bat1 > BAT1_MAX_LIMIT || bms->v_pack > PACK_MAX_LIMIT) {
    UART_SendString("[!] WARNING: OVERVOLTAGE DETECTED!\r\n");
  } else {
    UART_SendString("[OK] NORMAL - ALL VOLTAGES WITHIN RANGE\r\n");
  }
  UART_SendString("--------------------------------------------------------------------\r\n\r\n");
}

/**
  * @brief  Helper to transmit a null-terminated string over USART2
  * @param  str: Null-terminated C-string
  * @retval None
  */
void UART_SendString(const char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)str, (uint16_t)strlen(str), 100);
}

/**
  * @brief System Clock Configuration (16 MHz HSI with PLL to 84 MHz or HSI Direct)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

/**
  * @brief USART2 Initialization Function (115200 Baud, 8N1)
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* Enable GPIO Clocks */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/**
  * @brief  Error Handler
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
