#include "task_public.h"
#include "main.h"
#include "cmsis_os.h"

void DebugTask(void *argument) {
    while (true) {
        osDelay(pdMS_TO_TICKS(2000));
    }
}