#include "my_uart.h"

#include <string.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "rom/crc.h"

#define CYCLE_LENGTH 600 //pts
#define CDC_CHANNEL ((tinyusb_cdcacm_itf_t)TINYUSB_CDC_ACM_0)

static const uint8_t preamble = 0x7E;
static const uint8_t postamble = 0x81;
static const uint8_t escape = 0x55;
static const uint8_t resp_cmd = 0x01;

static float transmit_buffer1[CYCLE_LENGTH * 2];
static uint32_t crc_1 = ~0;
static float transmit_buffer2[CYCLE_LENGTH * 2];
static uint32_t crc_2 = ~0;
static float receive_buffer[CYCLE_LENGTH] = {};
static uint8_t receive_raw[sizeof(receive_buffer) + 100];
static size_t cycle_counter = 0;
static float* current_tx_buf = transmit_buffer1;
static uint32_t* current_crc = &crc_1;
static bool operate = false;
static TaskHandle_t parser_task_handle;
static QueueHandle_t parser_queue_handle;
static SemaphoreHandle_t parser_semaphore;

static const char* TAG = "USB_CDC";

enum parser_state
{
    searching_for_preamble,
    reading_cmd,
    reading_args,
    postamble_encountered
};

void write_immedeately(uint8_t* buf, size_t sz)
{
    tinyusb_cdcacm_write_queue(CDC_CHANNEL, buf, sz);
    tinyusb_cdcacm_write_flush(CDC_CHANNEL, 0);
}

void parse_input(size_t sz)
{
    static parser_state state = parser_state::searching_for_preamble;
    static uint8_t cmd;
    static bool escape_state = false;
    static size_t argument_index;
    for (size_t stream_index = 0; stream_index < sz; stream_index++)
    {
        escape_state = (escape_state ? false : receive_raw[stream_index] == escape);
        if (escape_state) 
        {
            stream_index++;
            ESP_LOGI(TAG, "Escape encountered.");
        }
        else
        {
            if (receive_raw[stream_index] == postamble) state = parser_state::postamble_encountered;
        }

        switch (state)
        {
        case parser_state::searching_for_preamble:
            if (receive_raw[stream_index] == preamble && !escape_state)
            {
                state = parser_state::reading_cmd;
            }
            break;
        case parser_state::reading_cmd:
            cmd = receive_raw[stream_index];
            state = parser_state::reading_args;
            argument_index = 0;
            break;
        case parser_state::reading_args:
        {
            switch (cmd)
            {
            case 'I': //INFO
            {
                ESP_LOGI(TAG, "Info command received. Skipping argument #%i", argument_index);
                float test_endianess = 100.1;
                write_immedeately(reinterpret_cast<uint8_t*>(&test_endianess), sizeof(test_endianess));
                break;
            }
            case 0x07: //DAC
                if (argument_index >= sizeof(receive_buffer))
                {
                    ESP_LOGE(TAG, "DAC buffer overrun prevented!");
                    state = parser_state::searching_for_preamble;
                    break;
                }
                ESP_LOGI(TAG, "DAC voltages received #%i", argument_index);
                *(reinterpret_cast<uint8_t*>(&(receive_buffer[argument_index / 4])) + argument_index % 4) = receive_raw[stream_index];
                break;
            case 0x01: //STOP
                operate = false;
                ESP_LOGI(TAG, "Cycle STOP.");
                state = parser_state::searching_for_preamble;
                break;
            case 0x02: //START
                operate = true;
                ESP_LOGI(TAG, "Cycle START.");
                state = parser_state::searching_for_preamble;
                break;
            default:
                break;
            }
            argument_index++;
            break;
        }
        case parser_state::postamble_encountered:
        default:
            state = parser_state::searching_for_preamble;
            //TODO error handling
            break;
        }
    }
}

void parser_task(void* arg)
{
    while (1)
    {
        size_t b;
        auto r = xQueueReceive(parser_queue_handle, &b, portMAX_DELAY);
        if (r == pdTRUE) 
        {
            parse_input(b);
            xSemaphoreGive(parser_semaphore);
        }
    }
}

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    static size_t rx_size = 0;

    xSemaphoreTake(parser_semaphore, portMAX_DELAY);
    /* read */
    esp_err_t ret = tinyusb_cdcacm_read((tinyusb_cdcacm_itf_t)itf, receive_raw, sizeof(receive_raw), &rx_size);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Data from channel %d: %i bytes.", itf, rx_size);
    }
    else
    {
        ESP_LOGE(TAG, "Read error");
    }
    xQueueSend(parser_queue_handle, &rx_size, portMAX_DELAY);
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
}

void queue_for_transmit(float* buffer, size_t sz, uint32_t* crc)
{
    static uint8_t wdt_counter = 0;
    wdt_counter++;
    tinyusb_cdcacm_write_queue(CDC_CHANNEL, &preamble, 1);
    tinyusb_cdcacm_write_queue(CDC_CHANNEL, &resp_cmd, 1);
    tinyusb_cdcacm_write_queue(CDC_CHANNEL, reinterpret_cast<uint8_t*>(buffer), sz);
    tinyusb_cdcacm_write_queue(CDC_CHANNEL, &wdt_counter, 1);
    *crc = crc32_le(*crc, &wdt_counter, sizeof(wdt_counter));
    tinyusb_cdcacm_write_queue(CDC_CHANNEL, reinterpret_cast<uint8_t*>(crc), sizeof(crc));
    tinyusb_cdcacm_write_queue(CDC_CHANNEL, &postamble, 1);
    tinyusb_cdcacm_write_flush(CDC_CHANNEL, 0);
    ESP_LOGI(TAG, "Sent a data packet.");
}

namespace my_uart
{
    bool get_operate()
    {
        return operate;
    }
    float next(float temp, float res)
    {
        if (cycle_counter >= CYCLE_LENGTH) 
        {
            cycle_counter = 0;
            *current_crc = ~*current_crc;
            queue_for_transmit(current_tx_buf, sizeof(transmit_buffer1), current_crc);
            current_tx_buf = (current_tx_buf == transmit_buffer1 ? transmit_buffer2 : transmit_buffer1);
            current_crc = (current_crc == &crc_1 ? &crc_2 : &crc_1);
            *current_crc = ~0;
        }
        current_tx_buf[cycle_counter * 2] = temp;
        current_tx_buf[cycle_counter * 2 + 1] = res;
        *current_crc = crc32_le(*current_crc, reinterpret_cast<uint8_t*>(current_tx_buf + cycle_counter * 2), sizeof(float) * 2);
        float ret = receive_buffer[cycle_counter++];
        return ret;
    }
    void init()
    {
        ESP_LOGI(TAG, "USB initialization");
        const tinyusb_config_t tusb_cfg = {}; // the configuration using default values
        ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

        tinyusb_config_cdcacm_t amc_cfg = {
            .usb_dev = TINYUSB_USBDEV_0,
            .cdc_port = TINYUSB_CDC_ACM_0,
            .rx_unread_buf_sz = 64,
            .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
            .callback_rx_wanted_char = NULL,
            .callback_line_state_changed = NULL,
            .callback_line_coding_changed = NULL
        };

        ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
        ESP_LOGI(TAG, "USB initialization DONE");

        parser_semaphore = xSemaphoreCreateBinary();
        assert(parser_semaphore);
        xSemaphoreGive(parser_semaphore);
        parser_queue_handle = xQueueCreate(10, sizeof(size_t));
        assert(parser_queue_handle);
        xTaskCreatePinnedToCore(parser_task, "input_parser", 4096, NULL, 1, &parser_task_handle, 0);
        assert(parser_task_handle);
    }

}