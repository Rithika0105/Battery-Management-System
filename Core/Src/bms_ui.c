#include "bms_ui.h"
#include "bms_safety.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

void BMS_UI_Init(void)
{
  ST7735_Init();
  BMS_UI_DrawStaticLayout();
}

void BMS_UI_DrawStaticLayout(void)
{
  /* Full Background Clear */
  ST7735_FillScreen(COLOR_BG);

  /* 1. Header Bar */
  ST7735_FillRect(0, 0, 160, 15, COLOR_HEADER_BG);
  ST7735_DrawString(4, 3, "BMS HYBRID EDF", Font_7x10, COLOR_TEXT_WHITE, COLOR_HEADER_BG);
  ST7735_DrawLineH(0, 15, 160, COLOR_CARD_BORDER);

  /* 2. Top-Left Card: Cells (X: 2, Y: 18, W: 76, H: 50) */
  ST7735_DrawCard(2, 18, 76, 50, "CELLS", COLOR_TEXT_MUTED);

  /* 3. Bottom-Left Card: Pack & SOC (X: 2, Y: 70, W: 76, H: 40) */
  ST7735_DrawCard(2, 70, 76, 40, "PACK/SOC", COLOR_TEXT_MUTED);

  /* 4. Top-Right Card: Current & Power (X: 81, Y: 18, W: 77, H: 50) */
  ST7735_DrawCard(81, 18, 77, 50, "CURRENT", COLOR_TEXT_MUTED);

  /* 5. Bottom-Right Card: Environment (X: 81, Y: 70, W: 77, H: 40) */
  ST7735_DrawCard(81, 70, 77, 40, "TEMP/HUM", COLOR_TEXT_MUTED);

  /* 6. Bottom Status Bar Container */
  ST7735_DrawRect(2, 112, 156, 14, COLOR_CARD_BORDER);
  ST7735_FillRect(3, 113, 154, 12, COLOR_CARD_BG);
}

/* Helper to format floating point values without requiring newlib-nano _printf_float */
static void UI_FormatFloat(char *out_buf, size_t max_len, float val, int decimals, const char *prefix, const char *suffix)
{
  char sign[2] = "";
  if (val < 0.0f) {
    sign[0] = '-';
    sign[1] = '\0';
    val = -val;
  }
  int int_part = (int)val;
  float remainder = val - (float)int_part;

  if (decimals == 2) {
    int frac_part = (int)(remainder * 100.0f + 0.5f);
    if (frac_part >= 100) { int_part++; frac_part = 0; }
    snprintf(out_buf, max_len, "%s%s%d.%02d%s", prefix, sign, int_part, frac_part, suffix);
  } else if (decimals == 1) {
    int frac_part = (int)(remainder * 10.0f + 0.5f);
    if (frac_part >= 10) { int_part++; frac_part = 0; }
    snprintf(out_buf, max_len, "%s%s%d.%01d%s", prefix, sign, int_part, frac_part, suffix);
  } else {
    snprintf(out_buf, max_len, "%s%s%d%s", prefix, sign, int_part, suffix);
  }
}

void BMS_UI_UpdateTelemetry(const BMS_DualBattery_Data_t *bms)
{
  char str[24];

  /* 1. Cells Card Values */
  if (bms->bat1_connected)
    UI_FormatFloat(str, sizeof(str), bms->v_bat1, 2, "B1:", "V");
  else
    snprintf(str, sizeof(str), "B1: ----V");
  ST7735_DrawString(5, 30, str, Font_7x10, COLOR_CYAN, COLOR_CARD_BG);

  if (bms->pack_connected)
    UI_FormatFloat(str, sizeof(str), bms->v_bat2, 2, "B2:", "V");
  else
    snprintf(str, sizeof(str), "B2: ----V");
  ST7735_DrawString(5, 42, str, Font_7x10, COLOR_LIME, COLOR_CARD_BG);

  if (bms->bat1_connected && bms->pack_connected)
    UI_FormatFloat(str, sizeof(str), bms->v_imbalance, 2, "d :", "V");
  else
    snprintf(str, sizeof(str), "d : ----V");
  ST7735_DrawString(5, 54, str, Font_7x10, COLOR_AMBER, COLOR_CARD_BG);

  /* 2. Total Pack Card Values & Coulomb-Counted SOC */
  if (bms->pack_connected)
    UI_FormatFloat(str, sizeof(str), bms->v_pack, 2, " ", "V");
  else
    snprintf(str, sizeof(str), "  ----V");
  ST7735_DrawString(8, 82, str, Font_7x10, COLOR_EMERALD, COLOR_CARD_BG);

  /* Mini SOC / Voltage Progress Bar (Width 60 px) based on Coulomb Counting */
  uint8_t bar_w = (uint8_t)((g_bms_package.soc_percent / 100.0f) * 60.0f);
  if (bar_w > 60) bar_w = 60;

  ST7735_DrawRect(8, 97, 64, 6, COLOR_CARD_BORDER);
  ST7735_FillRect(10, 99, bar_w, 2, COLOR_EMERALD);
  ST7735_FillRect(10 + bar_w, 99, 60 - bar_w, 2, COLOR_BG);

  /* 3. Current & Power Card Values */
  if (bms->current_sensor_connected)
    UI_FormatFloat(str, sizeof(str), bms->current_amps, 2, (bms->current_amps >= 0 ? "+" : ""), "A");
  else
    snprintf(str, sizeof(str), " ----A");
  ST7735_DrawString(84, 30, str, Font_7x10, COLOR_YELLOW, COLOR_CARD_BG);

  /* State Badge */
  if (bms->current_sensor_connected)
  {
    if (strcmp(bms->current_state, "CHARGING") == 0)
      snprintf(str, sizeof(str), "[CHG] ");
    else if (strcmp(bms->current_state, "DISCHARGING") == 0)
      snprintf(str, sizeof(str), "[DIS] ");
    else
      snprintf(str, sizeof(str), "[IDL] ");
  }
  else
  {
    snprintf(str, sizeof(str), "[DISC]");
  }
  ST7735_DrawString(84, 42, str, Font_7x10, COLOR_TEXT_WHITE, COLOR_CARD_BG);

  /* Power */
  if (bms->current_sensor_connected && bms->pack_connected)
    UI_FormatFloat(str, sizeof(str), fabsf(bms->power_watts), 1, "P:", "W");
  else
    snprintf(str, sizeof(str), "P:---W");
  ST7735_DrawString(84, 54, str, Font_7x10, COLOR_TEXT_MUTED, COLOR_CARD_BG);

  /* 4. Temperature & Humidity Card */
  if (bms->dht11_connected)
  {
    UI_FormatFloat(str, sizeof(str), bms->temperature_c, 1, "", " C");
    ST7735_DrawString(84, 82, str, Font_7x10, COLOR_ORANGE, COLOR_CARD_BG);

    UI_FormatFloat(str, sizeof(str), bms->humidity_pct, 1, "", " %");
    ST7735_DrawString(84, 94, str, Font_7x10, COLOR_SKY, COLOR_CARD_BG);
  }
  else
  {
    ST7735_DrawString(84, 82, "--.- C", Font_7x10, COLOR_TEXT_MUTED, COLOR_CARD_BG);
    ST7735_DrawString(84, 94, "--.- %", Font_7x10, COLOR_TEXT_MUTED, COLOR_CARD_BG);
  }

  /* 5. 3-Stage Functional Degradation Safety Banner */
  if (g_bms_package.safety_stage == BMS_STAGE_3_CATASTROPHIC_ABORT)
  {
    /* Stage 3: Immediate Red Hazard Banner */
    ST7735_FillRect(3, 113, 154, 12, COLOR_RED);
    ST7735_DrawString(6, 115, "STAGE 3: ABORT (0%)", Font_7x10, COLOR_TEXT_WHITE, COLOR_RED);
  }
  else if (g_bms_package.safety_stage == BMS_STAGE_2_WARNING_THROTTLE)
  {
    /* Stage 2: Limp-Home / Turtle Mode Banner */
    ST7735_FillRect(3, 113, 154, 12, COLOR_CARD_BG);
    ST7735_DrawString(6, 115, "STAGE 2: TURTLE 30%", Font_7x10, COLOR_AMBER, COLOR_CARD_BG);
  }
  else
  {
    /* Stage 1: Nominal Safe Zone Banner */
    ST7735_FillRect(3, 113, 154, 12, COLOR_CARD_BG);
    if (!bms->bat1_connected && !bms->pack_connected)
    {
      ST7735_DrawString(6, 115, "IDLE: NO BATTERY    ", Font_7x10, COLOR_TEXT_MUTED, COLOR_CARD_BG);
    }
    else
    {
      ST7735_DrawString(6, 115, "STAGE 1: NOMINAL OK ", Font_7x10, COLOR_EMERALD, COLOR_CARD_BG);
    }
  }
}
