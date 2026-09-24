/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
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

  /* Environmental / Thermal (DHT11) */
  float    temperature_c;       /* Ambient / Pack Temperature (°C) */
  float    humidity_pct;        /* Relative Humidity (% RH) */
  bool     dht11_connected;     /* True if DHT11 is responding */
  uint8_t  dht11_error_code;    /* 0=OK, 1=No Response, 2=Checksum Error, 3=Timeout */
} BMS_DualBattery_Data_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define BAT1_MAX_LIMIT           (12.00f)   /* 12V Max for Battery 1 */
#define BAT1_MIN_LIMIT           (8.40f)    /* 8.4V Cutoff */
#define PACK_MAX_LIMIT           (24.00f)   /* 24V Max for Total Pack */
#define PACK_MIN_LIMIT           (16.80f)   /* 16.8V Cutoff */
#define CURRENT_MAX_LIMIT        (30.00f)   /* 30A Maximum Current Limit */
#define TEMP_MAX_LIMIT           (45.00f)   /* 45 °C Temperature Limit */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define DHT11_PIN                GPIO_PIN_0
#define DHT11_PORT               GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
