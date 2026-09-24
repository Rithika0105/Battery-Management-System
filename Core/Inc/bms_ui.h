#ifndef __BMS_UI_H
#define __BMS_UI_H

#include "main.h"
#include "st7735.h"

/* Function Prototypes */
void BMS_UI_Init(void);
void BMS_UI_DrawStaticLayout(void);
void BMS_UI_UpdateTelemetry(const BMS_DualBattery_Data_t *bms);

#endif /* __BMS_UI_H */
