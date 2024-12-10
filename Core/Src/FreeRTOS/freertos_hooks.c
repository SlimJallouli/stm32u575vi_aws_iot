/*
 * Copyright of Amazon Web Services, Inc. (AWS) 2022
 *
 * This code is licensed under the AWS Intellectual Property License, which can
 * be found here: https://aws.amazon.com/legal/aws-ip-license-terms/; provided
 * that AWS grants you a limited, royalty-free, revocable, non-exclusive,
 * non-sublicensable, non-transferrable license to modify the code and
 * distribute binaries produced from the code (or modified versions of the code)
 * within hardware modules provided to third parties as long as such hardware
 * modules are qualified for AWS IoT ExpressLink under the AWS Device
 * Qualification Program (https://aws.amazon.com/partners/programs/dqp/). Your
 * receipt of this code is subject to any non-disclosure (or similar) agreement
 * between you and AWS.
 */
#include "main.h"

#include "logging_levels.h"
#define LOG_LEVEL    LOG_ERROR
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "freertos_hooks.h"
#include "hw_defs.h"
#include <string.h>

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
  static StaticTask_t timerTaskTCB;
  static StackType_t timerTaskStack[configTIMER_TASK_STACK_DEPTH];

  *ppxTimerTaskTCBBuffer = &timerTaskTCB;
  *ppxTimerTaskStackBuffer = timerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  static StaticTask_t idleTaskTCB;
  static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];

  *ppxIdleTaskTCBBuffer = &idleTaskTCB;
  *ppxIdleTaskStackBuffer = idleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

#if configUSE_IDLE_HOOK == 1
__weak void vApplicationIdleHook( void )
{
  vPetWatchdog();
//  __WFI();
}
#endif

#if configUSE_MALLOC_FAILED_HOOK
__weak void vApplicationMallocFailedHook(void)
{
	LogError("Malloc Fail\n");
}
#endif

#if (configCHECK_FOR_STACK_OVERFLOW > 0)
__weak void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
    taskENTER_CRITICAL();

    LogSys( "Stack overflow in %s", pcTaskName );
    ( void ) xTask;

    vDoSystemReset();

    taskEXIT_CRITICAL();
}
#endif

void vDoSystemReset(void)
{
  vPetWatchdog();

  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
  {
    vTaskSuspendAll();
  }

  LogSys("System Reset in progress.");

  /* Drain log buffers */
  vDyingGasp();

  NVIC_SystemReset();
}

#if  configGENERATE_RUN_TIME_STATS
void configureTimerForRunTimeStats(void)
{
  HAL_TIM_Base_Stop(pRunTimeStats_Timer);
  pRunTimeStats_Timer->Instance->CNT = 0;
  HAL_TIM_Base_Start(pRunTimeStats_Timer);
}

unsigned long getRunTimeCounterValue(void)
{
  static configRUN_TIME_COUNTER_TYPE counter = 0;
  static uint32_t last_cnt_val = 0;
  uint32_t current_cnt_val = 0;
  uint32_t difference;

  current_cnt_val = pRunTimeStats_Timer->Instance->CNT;

  difference = current_cnt_val - last_cnt_val;

  last_cnt_val = current_cnt_val;

  counter += difference;

  return counter;
}

static configRUN_TIME_COUNTER_TYPE IRQ_Timer = 0;

void incrementIRQRunTime(configRUN_TIME_COUNTER_TYPE Initial_time)
{
  IRQ_Timer += getRunTimeCounterValue() - Initial_time;
}

configRUN_TIME_COUNTER_TYPE getIRQTimeCounterValue(void)
{
  return IRQ_Timer;
}
#endif

