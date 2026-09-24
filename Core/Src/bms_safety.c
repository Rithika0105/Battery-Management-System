#include "bms_safety.h"
#include "edf_scheduler.h"
#include "bms_ui.h"
#include <string.h>
#include <math.h>

/* Battery Capacity for Coulomb Counting */
#define NOMINAL_CAPACITY_AH      (5.0f)   /* 5.0 Ah Nominal Pack Capacity */
#define INITIAL_SOC_PERCENT      (100.0f) /* Initial State of Charge */

BMS_Data_Package_t g_bms_package;
static BMS_Data_Package_t s_mailbox_buffer;
static bool s_mailbox_fresh = false;

static float s_accumulated_ah = 0.0f;
static float s_current_soc = INITIAL_SOC_PERCENT;

/* Extern functions from main.c / drivers */
extern void BMS_ProcessSensors(BMS_DualBattery_Data_t *bms);
extern void BMS_PrintTelemetry(const BMS_DualBattery_Data_t *bms);
extern BMS_DualBattery_Data_t bms_data;

void BMS_Safety_Init(void)
{
  /* 1. Initialize Relay Control GPIO (PA8) */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = RELAY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_PORT, &GPIO_InitStruct);

  /* Start with Relay CLOSED (Active HIGH: Power Connected) */
  BMS_Relay_Set(true);

  /* 2. Initialize Package Structure */
  memset(&g_bms_package, 0, sizeof(g_bms_package));
  g_bms_package.safety_stage = BMS_STAGE_1_NOMINAL;
  g_bms_package.throttle_limit_pct = 100;
  g_bms_package.relay_state_closed = true;
  g_bms_package.soc_percent = INITIAL_SOC_PERCENT;
}

void BMS_Relay_Set(bool closed)
{
  if (closed)
  {
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET);   /* Relay Closed / Power ON */
  }
  else
  {
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET); /* Relay Open / Power CUT */
  }
}

/**
  * @brief  Coulomb Counting Integration: Ah = Integral(I * dt)
  */
void BMS_CoulombCounting_Update(float current_amps, float delta_time_sec)
{
  /* Convert Amps * Seconds into Ampere-Hours (Ah = I * dt / 3600) */
  float delta_ah = (current_amps * delta_time_sec) / 3600.0f;
  s_accumulated_ah += delta_ah;

  /* SOC = Initial_SOC - (Ah_consumed / Nominal_Capacity) * 100 */
  s_current_soc = INITIAL_SOC_PERCENT - (s_accumulated_ah / NOMINAL_CAPACITY_AH) * 100.0f;

  if (s_current_soc > 100.0f) s_current_soc = 100.0f;
  if (s_current_soc < 0.0f)   s_current_soc = 0.0f;
}

/**
  * @brief  3-Stage Functional Degradation Decision Matrix (Automotive Safety)
  */
void BMS_Evaluate_Safety_Stages(BMS_Data_Package_t *pkg)
{
  /* Stage 3: Catastrophic Abort (Absolute Emergency) */
  /* Condition: Temp >= 50°C OR Critical Overvoltage / Short Circuit */
  if ((pkg->temperature_c >= 50.0f && pkg->temperature_c < 120.0f) ||
      (pkg->v_pack > 25.0f) || (fabsf(pkg->current_amps) >= 28.0f))
  {
    pkg->safety_stage = BMS_STAGE_3_CATASTROPHIC_ABORT;
    pkg->throttle_limit_pct = 0;
    pkg->relay_state_closed = false;

    /* Drop GPIO Pin Immediately to Open Relay Contact */
    BMS_Relay_Set(false);

    /* Trigger Asynchronous Preemptive Override in EDF Scheduler */
    EDF_PreemptiveEmergencyOverride(xSafetyGuardTaskHandle);
  }
  /* Stage 2: Power Throttling Mode (The Warning Zone / Limp-Home / Turtle Mode) */
  /* Condition: 40°C <= Temp < 50°C OR Low Voltage (< 17.5V) OR Moderate Imbalance (> 1.0V) */
  else if ((pkg->temperature_c >= 40.0f && pkg->temperature_c < 50.0f) ||
           (pkg->v_pack > 0.0f && pkg->v_pack < 17.5f) ||
           (pkg->v_imbalance >= 1.0f))
  {
    pkg->safety_stage = BMS_STAGE_2_WARNING_THROTTLE;
    pkg->throttle_limit_pct = 30; /* Throttle motor torque by 70% to cool cells down */
    pkg->relay_state_closed = true; /* KEEP RELAY CLOSED so car does not stall on highway! */
    BMS_Relay_Set(true);
  }
  /* Stage 1: Nominal Safe Zone */
  else
  {
    pkg->safety_stage = BMS_STAGE_1_NOMINAL;
    pkg->throttle_limit_pct = 100; /* Full 100% Power */
    pkg->relay_state_closed = true; /* Relay firmly closed */
    BMS_Relay_Set(true);
  }
}

void BMS_Mailbox_Send(const BMS_Data_Package_t *src)
{
  memcpy(&s_mailbox_buffer, src, sizeof(BMS_Data_Package_t));
  s_mailbox_fresh = true;
}

bool BMS_Mailbox_Receive(BMS_Data_Package_t *dest)
{
  if (s_mailbox_fresh)
  {
    memcpy(dest, &s_mailbox_buffer, sizeof(BMS_Data_Package_t));
    s_mailbox_fresh = false;
    return true;
  }
  return false;
}

/* ============================================================================
 * 3 PERIODIC EDF FREERTOS TASKS
 * ============================================================================ */

/**
  * @brief  Task 1: Sensor Data Acquisition
  * @note   Period: 50 ms | Relative Deadline: 50 ms
  */
void vSensorTask(void)
{
  /* 1. Acquire raw signals from ADC (PA0, PA1, PA4) and DHT11 (PB0) */
  BMS_ProcessSensors(&bms_data);

  /* 2. Bundle into BMS Data Package */
  g_bms_package.v_bat1 = bms_data.v_bat1;
  g_bms_package.v_bat2 = bms_data.v_bat2;
  g_bms_package.v_pack = bms_data.v_pack;
  g_bms_package.v_imbalance = bms_data.v_imbalance;
  g_bms_package.current_amps = bms_data.current_amps;
  g_bms_package.power_watts = bms_data.power_watts;
  g_bms_package.temperature_c = bms_data.temperature_c;
  g_bms_package.humidity_pct = bms_data.humidity_pct;

  /* 3. Push to Mailbox Queue */
  BMS_Mailbox_Send(&g_bms_package);
}

extern void UART_SendString(const char *str);

/**
  * @brief  Task 2: Telemetry Dashboard & Coulomb Counting
  * @note   Period: 500 ms | Relative Deadline: 500 ms
  */
void vDashboardTask(void)
{
  BMS_Data_Package_t local_pkg;

  if (BMS_Mailbox_Receive(&local_pkg) || true)
  {
    /* 1. Coulomb Counting (Ah = Integral I * dt) */
    BMS_CoulombCounting_Update(g_bms_package.current_amps, 0.5f);
    g_bms_package.soc_percent = s_current_soc;
    g_bms_package.ah_consumed = s_accumulated_ah;

    /* 2. Format & Print UART Telemetry */
    static uint32_t sample_counter = 0;
    sample_counter++;
    char header_str[64];
    snprintf(header_str, sizeof(header_str), "--- [BMS SAMPLE #%lu] -----------------------------------\r\n", (unsigned long)sample_counter);
    UART_SendString(header_str);

    BMS_PrintTelemetry(&bms_data);

    /* 3. Update ST7735 TFT Display with 3-Stage Safety Layout */
    BMS_UI_UpdateTelemetry(&bms_data);
  }
}

/**
  * @brief  Task 3: Preemptive Safety Guard
  * @note   Period: 10 ms | Relative Deadline: 10 ms
  */
void vSafetyGuardTask(void)
{
  /* Intercept data and evaluate 3-Stage degradation safety rules */
  BMS_Evaluate_Safety_Stages(&g_bms_package);
}
