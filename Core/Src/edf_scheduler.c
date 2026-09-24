#include "edf_scheduler.h"
#include <string.h>

static EDF_TaskControlBlock_t s_task_pool[MAX_EDF_TASKS];
static uint8_t s_task_count = 0;
static volatile uint32_t s_system_time_ms = 0;
static volatile bool s_emergency_override_active = false;

EDF_TaskControlBlock_t* xSensorTaskHandle = NULL;
EDF_TaskControlBlock_t* xDashboardTaskHandle = NULL;
EDF_TaskControlBlock_t* xSafetyGuardTaskHandle = NULL;

void EDF_Scheduler_Init(void)
{
  s_task_count = 0;
  s_system_time_ms = 0;
  s_emergency_override_active = false;
  memset(s_task_pool, 0, sizeof(s_task_pool));
}

EDF_TaskControlBlock_t* EDF_TaskCreate(const char* name, void (*task_func)(void), uint32_t period_ms, uint32_t deadline_ms)
{
  if (s_task_count >= MAX_EDF_TASKS) return NULL;

  EDF_TaskControlBlock_t* tcb = &s_task_pool[s_task_count++];
  tcb->name = name;
  tcb->task_func = task_func;
  tcb->period_ms = period_ms;
  tcb->deadline_ms = deadline_ms;
  tcb->release_time_ms = 0;
  tcb->remaining_slack = (int32_t)deadline_ms;
  tcb->dynamic_priority = SYSTEM_PRIORITY_LOW;
  tcb->state = TASK_STATE_READY;
  tcb->execution_count = 0;

  return tcb;
}

void EDF_TaskPrioritySet(EDF_TaskControlBlock_t* task, uint8_t priority)
{
  if (task != NULL)
  {
    task->dynamic_priority = priority;
  }
}

void EDF_PreemptiveEmergencyOverride(EDF_TaskControlBlock_t* emergency_task)
{
  s_emergency_override_active = true;
  for (uint8_t i = 0; i < s_task_count; i++)
  {
    if (&s_task_pool[i] == emergency_task)
    {
      s_task_pool[i].dynamic_priority = SYSTEM_PRIORITY_MAX;
      s_task_pool[i].state = TASK_STATE_READY;
    }
    else
    {
      s_task_pool[i].dynamic_priority = SYSTEM_PRIORITY_IDLE;
    }
  }
}

/**
  * @brief  1ms Hybrid EDF Supervisor Interrupt Hook (Called from SysTick or 1ms Timer)
  */
void EDF_SysTick_Supervisor(void)
{
  s_system_time_ms++;

  if (s_emergency_override_active) return; /* Preemptive lock during stage 3 abort */

  EDF_TaskControlBlock_t* winning_task = NULL;
  int32_t smallest_slack = 0x7FFFFFFF;

  for (uint8_t i = 0; i < s_task_count; i++)
  {
    EDF_TaskControlBlock_t* task = &s_task_pool[i];

    /* Check periodic arrival */
    if ((s_system_time_ms - task->release_time_ms) >= task->period_ms)
    {
      task->release_time_ms = s_system_time_ms;
      task->state = TASK_STATE_READY;
    }

    if (task->state == TASK_STATE_READY)
    {
      /* d_i = D_i - (t - r_i) */
      uint32_t elapsed = s_system_time_ms - task->release_time_ms;
      task->remaining_slack = (int32_t)task->deadline_ms - (int32_t)elapsed;

      /* Task with smallest remaining slack wins the EDF priority boost */
      if (task->remaining_slack < smallest_slack)
      {
        smallest_slack = task->remaining_slack;
        winning_task = task;
      }
      task->dynamic_priority = SYSTEM_PRIORITY_LOW;
    }
  }

  if (winning_task != NULL)
  {
    /* Dynamic Priority Elevation */
    winning_task->dynamic_priority = SYSTEM_PRIORITY_MAX;
  }
}

/**
  * @brief  Main EDF Task Dispatcher
  */
void EDF_Scheduler_Run(void)
{
  while (1)
  {
    EDF_TaskControlBlock_t* highest_priority_task = NULL;
    uint8_t max_prio = SYSTEM_PRIORITY_IDLE;

    /* Scan task pool for highest priority ready task */
    for (uint8_t i = 0; i < s_task_count; i++)
    {
      if (s_task_pool[i].state == TASK_STATE_READY && s_task_pool[i].dynamic_priority > max_prio)
      {
        max_prio = s_task_pool[i].dynamic_priority;
        highest_priority_task = &s_task_pool[i];
      }
    }

    if (highest_priority_task != NULL)
    {
      highest_priority_task->state = TASK_STATE_RUNNING;
      highest_priority_task->task_func();
      highest_priority_task->execution_count++;
      highest_priority_task->state = TASK_STATE_BLOCKED;
    }
    else
    {
      /* Sleep until next 1ms SysTick interrupt */
      __WFI();
    }
  }
}
