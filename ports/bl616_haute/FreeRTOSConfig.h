/*
 * FreeRTOS configuration for the BL616 Haute target. Based on the Bouffalo
 * SDK's BL616 FreeRTOS examples. The heap is the SDK's (heap_3 -> malloc), so
 * configTOTAL_HEAP_SIZE is not what bounds allocation.
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
#include <stdio.h>

#if __riscv_xlen == 64
#define configMTIME_BASE_ADDRESS    (0)
#define configMTIMECMP_BASE_ADDRESS ((0xE4000000UL) + 0x4000UL)
#else
#define configMTIME_BASE_ADDRESS    ((0xE0000000UL) + 0xBFF8UL)
#define configMTIMECMP_BASE_ADDRESS ((0xE0000000UL) + 0x4000UL)
#endif

#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCPU_CLOCK_HZ                      ((uint32_t)(1 * 1000 * 1000))
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    (31)
#define configMINIMAL_STACK_SIZE                ((unsigned short)128)
#define configTOTAL_HEAP_SIZE                   ((size_t)32 * 1024)
#define configMAX_TASK_NAME_LEN                 (16)
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    1
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 0
#define configUSE_MUTEXES                       1
#define configQUEUE_REGISTRY_SIZE               8
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_APPLICATION_TASK_TAG          1
#define configUSE_COUNTING_SEMAPHORES           1
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configUSE_POSIX_ERRNO                   1

#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         (2)

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                8
#define configTIMER_TASK_STACK_DEPTH            (1024)

#define INCLUDE_vTaskPrioritySet         1
#define INCLUDE_uxTaskPriorityGet        1
#define INCLUDE_vTaskDelete              1
#define INCLUDE_vTaskCleanUpResources    1
#define INCLUDE_vTaskSuspend             1
#define INCLUDE_vTaskDelayUntil          1
#define INCLUDE_vTaskDelay               1
#define INCLUDE_eTaskGetState            1
#define INCLUDE_xTimerPendFunctionCall   1
#define INCLUDE_xTaskAbortDelay          1
#define INCLUDE_xTaskGetHandle           1
#define INCLUDE_xSemaphoreGetMutexHolder 1
#define INCLUDE_xTaskGetSchedulerState   1
#define INCLUDE_uxTaskGetStackHighWaterMark 1

#ifdef __cplusplus
extern "C" {
#endif
void vApplicationMallocFailedHook(void);
void vAssertCalled(void);
#ifdef __cplusplus
}
#endif

#define configASSERT(x)                        \
    if ((x) == 0) {                            \
        printf("file [%s]\r\n", __FILE__);     \
        printf("func [%s]\r\n", __FUNCTION__); \
        printf("line [%d]\r\n", __LINE__);     \
        printf("%s\r\n", (const char *)(#x));  \
        vAssertCalled();                       \
    }

#endif
