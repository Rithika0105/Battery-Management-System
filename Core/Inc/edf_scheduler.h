#ifndef __EDF_SCHEDULER_H
#define __EDF_SCHEDULER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define MAX_EDF_TASKS            (4)
#define SYSTEM_PRIORITY_IDLE     (0)
#define SYSTEM_PRIORITY_LOW      (1)
#define SYSTEM_PRIORITY_MED      (2)
#define SYSTEM_PRIORITY_HIGH     (3)
#define SYSTEM_PRIORITY_MAX      (4)

typedef enum {
  TASK_STATE_SUSPENDED = 0,
  TASK_STATE_READY,
  TASK_STATE_RUNNING,
  TASK_STATE_BLOCKED
} EDF_TaskState_t;

typedef struct {
  const char*      name;
  void             (*task_func)(void);
  uint32_t         period_ms;         /* Task Period T_i */
  uint32_t         deadline_ms;       /* Relative Deadline D_i */
  uint32_t         release_time_ms;   /* Last arrival/release timestamp r_i */
  int32_t          remaining_slack;   /* d_i = D_i - (t - r_i) */
  uint8_t          dynamic_priority;  /* Priority assigned by EDF supervisor */
  EDF_TaskState_t  state;             /* Current task lifecycle state */
  uint32_t         execution_count;   /* Diagnostics / Profile counter */
} EDF_TaskControlBlock_t;

/* Task Handles */
extern EDF_TaskControlBlock_t* xSensorTaskHandle;
extern EDF_TaskControlBlock_t* xDashboardTaskHandle;
extern EDF_TaskControlBlock_t* xSafetyGuardTaskHandle;

/* Function Prototypes */
void EDF_Scheduler_Init(void);
EDF_TaskControlBlock_t* EDF_TaskCreate(const char* name, void (*task_func)(void), uint32_t period_ms, uint32_t deadline_ms);
void EDF_SysTick_Supervisor(void);
void EDF_TaskPrioritySet(EDF_TaskControlBlock_t* task, uint8_t priority);
void EDF_PreemptiveEmergencyOverride(EDF_TaskControlBlock_t* emergency_task);
void EDF_Scheduler_Run(void);

#endif /* __EDF_SCHEDULER_H */
