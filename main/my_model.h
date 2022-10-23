#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "model.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#pragma once

#ifndef MY_MODEL_HEADER
#define MY_MODEL_HEADER
namespace my_model {
void init();
extern TfLiteTensor* input;
extern TfLiteTensor* output;
static TaskHandle_t model_task_handle;
}
#endif
