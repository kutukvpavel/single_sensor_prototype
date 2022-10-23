#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "model.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "my_model.h"

namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
int inference_count = 0;

// create an area of memory to use for input, output, and intermediate arrays.
// minimum arena size, at the time of writing. after allocating tensors
// you can retrieve this value by invoking interpreter.arena_used_bytes().
const int kModelArenaSize = 2468 * 4;
// Extra headroom for model + alignment + future interpreter changes.
const int kExtraArenaSize = 560 + 16 + 100;
const int kTensorArenaSize = kModelArenaSize + kExtraArenaSize;
uint8_t tensor_arena[kTensorArenaSize];
} // namespace

namespace my_model {

TfLiteTensor* input;
TfLiteTensor* output;
static void prvInvokeInterpreter()
{
    interpreter->Invoke();
    ESP_LOGI("MODEL", "y: %f", output->data.f[0]);
}

static void model_task(void* pvParameter)
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(500);
    BaseType_t xResult;
    uint32_t ulNotifiedValue;

    for (;;) {
        xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, xMaxBlockTime);
        if (xResult == pdTRUE) {
            prvInvokeInterpreter();
        }
    }
}

void init()
{
    model = tflite::GetModel(model_tflite);
    static tflite::MicroErrorReporter micro_error_reporter;
    error_reporter = &micro_error_reporter;
    static tflite::AllOpsResolver resolver;

    // Build an interpreter to run the model with.
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
    interpreter = &static_interpreter;

    // Allocate memory from the tensor_arena for the model's tensors.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
        return;
    }

    // Obtain pointers to the model's input and output tensors.
    input = interpreter->input(0);
    output = interpreter->output(0);

    // Keep track of how many inferences we have performed.
    inference_count = 0;
    xTaskCreatePinnedToCore(model_task, "model_task", 4096, NULL, 1, &model_task_handle, 0);
    assert(model_task_handle);
}
}
