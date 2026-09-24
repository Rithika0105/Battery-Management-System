#ifndef __BMS_SAFETY_H
#define __BMS_SAFETY_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Relay Hardware Configuration (STM32F411RE Pin PA8 / CN10 Pin 23) */
#define RELAY_PORT               GPIOA
#define RELAY_PIN                GPIO_PIN_8

/* 3-Stage Functional Degradation Modes */
typedef enum {
  BMS_STAGE_1_NOMINAL = 1,       /* Safe Zone: Temp < 40°C, Relay CLOSED, Full Power */
  BMS_STAGE_2_WARNING_THROTTLE,  /* Warning Zone: 40°C <= Temp < 50°C, Relay CLOSED, Turtle Mode */
  BMS_STAGE_3_CATASTROPHIC_ABORT /* Emergency Zone: Temp >= 50°C, Relay OPEN, Hard Stop */
} BMS_SafetyStage_t;

/* Mailbox Data Package Structure */
typedef struct {
  float    v_bat1;               /* Battery 1 Voltage (V) */
  float    v_bat2;               /* Battery 2 Voltage (V) */
  float    v_pack;               /* Total Pack Voltage (V) */
  float    v_imbalance;          /* Cell Imbalance (V) */
  float    current_amps;         /* Measured Current (A) */
  float    temperature_c;        /* DHT11 Temperature (°C) */
  float    humidity_pct;         /* DHT11 Humidity (% RH) */
  float    power_watts;          /* Power (W) */
  float    soc_percent;          /* Coulomb-Counted State of Charge (%) */
  float    ah_consumed;          /* Accumulated Ah */
  BMS_SafetyStage_t safety_stage;/* Active Functional Safety Stage */
  uint8_t  throttle_limit_pct;   /* Available Motor Torque (100% in Stage 1, 30% in Stage 2, 0% in Stage 3) */
  bool     relay_state_closed;   /* True = Power Connected, False = Isolated */
  uint32_t timestamp_ms;
} BMS_Data_Package_t;

/* Global Mailbox Package */
extern BMS_Data_Package_t g_bms_package;

/* Function Prototypes */
void BMS_Safety_Init(void);
void BMS_Relay_Set(bool closed);
void BMS_CoulombCounting_Update(float current_amps, float delta_time_sec);
void BMS_Evaluate_Safety_Stages(BMS_Data_Package_t *pkg);
void BMS_Mailbox_Send(const BMS_Data_Package_t *src);
bool BMS_Mailbox_Receive(BMS_Data_Package_t *dest);

/* 3 FreeRTOS/EDF Tasks */
void vSensorTask(void);
void vDashboardTask(void);
void vSafetyGuardTask(void);

#endif /* __BMS_SAFETY_H */
