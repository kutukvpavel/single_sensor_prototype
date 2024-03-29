#include "my_uart.h"
#include "my_params.h"

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "rom/crc.h"
#include "esp_task_wdt.h"

/***
 * Protocol
 */

static const uint8_t preamble = 0x7E;
static const uint8_t postamble = 0x81;
static const uint8_t escape = 0x55;
static const float debug_prefix = -100.1;

/***
 * Commands 
 */
#define RSP_OK 0x00
#define RSP_BAD_CRC 0xFE
#define NO_STD_RSP 0xFF

#define CMD_STOP 0x01
#define CMD_START 0x02
#define RSP_ALREADY_IN_REQUESTED_STATE 0x01
#define RSP_STATE_SWITCH_ERROR 0x02

#define CMD_GET_DATA 0x03

#define CMD_GET_ERROR 0x04
//Responses are located in the header (enum-bitfield)

#define CMD_SET_HEATER_PARAMS 0x05
#define CMD_SET_MEASURE_PARAMS 0x06
#define CMD_SET_TEMP_CYCLE 0x07
#define RSP_SET_FAILED 0x01

#define CMD_GET_HAVE_DATA 0x08
#define RSP_NO_DATA 0x01

#define CMD_SET_PID_PARAMS 0x09
#define CMD_SET_ADC_CAL 0x10
#define CMD_SET_DAC_CAL 0x11
#define CMD_SAVE_NVS 0x20
#define CMD_GET_NVS 0xA0
#define CMD_ENABLE_PID_DBG 0xA1

// Alive indicator
#define ENABLE_DEBUG_INFO_CMD 1 //Falls through standard communication because lacks start/end flags

/***
 * Internal defines
 */
#define CYCLE_LENGTH 600 //pts
#define FLOATS_PER_POINT 2
#define TRANSMIT_BUFFER_SIZE (CYCLE_LENGTH * FLOATS_PER_POINT) //pts
#define CDC_CHANNEL ((tinyusb_cdcacm_itf_t)TINYUSB_CDC_ACM_0)

static const char* TAG = "USB_CDC";

/***
 * Decls
 */

namespace my_uart
{
    static bool operate = false;
    static my_error_codes error_codes = my_error_codes::none;
}

namespace receiver
{
    enum parser_state
    {
        searching_for_preamble,
        reading_cmd,
        reading_args,
        reading_counter,
        reading_crc,
        postamble_encountered
    };

    //Receiver state
    static float buffer1[CYCLE_LENGTH] = {};
    static float buffer2[CYCLE_LENGTH] = {};
    static size_t cycle_counter = 0;
    static float* current_buffer = buffer1;
    static float* current_element = current_buffer;
    static float* next_buffer = buffer2;
    static uint8_t receive_raw[sizeof(buffer1) + 100];
    static uint8_t receiver_wdt = 0;
    static uint32_t receiver_crc;
    static TaskHandle_t parser_task_handle;
    static QueueHandle_t parser_queue_handle;
    static SemaphoreHandle_t parser_semaphore;

    void parse_input(size_t sz);
    void parser_task(void* arg);
    float get_next();
    void cycle_end();
    void init();
}

namespace transmitter
{
    struct buffer_t
    {
        float* const start;
        float* current;
        uint32_t crc;
    };

    //Transmission state
    static size_t cycle_counter = 0;
    static uint8_t wdt_counter = 0;
    static const size_t buffers_count = 2;
    static float data_buffer1[TRANSMIT_BUFFER_SIZE];
    static float data_buffer2[TRANSMIT_BUFFER_SIZE];
    static buffer_t buffer1 = { data_buffer1, data_buffer1, ~0u };
    static buffer_t buffer2 = { data_buffer2, data_buffer2, ~0u };
    static buffer_t* const buffers[] = { &buffer1, &buffer2 };
    static buffer_t* empty_buffer = buffers[0];
    static buffer_t* full_buffer = buffers[1];
    static SemaphoreHandle_t transmit_mutex;
    static bool have_data = false;

    void write_immedeately(const uint8_t* buf, size_t sz);
    void write_dbg(float val);
    void send_buffer(uint8_t cmd, uint8_t* buffer, size_t sz);
    void send_precalc_buffer(uint8_t cmd, uint8_t* buffer, size_t sz, uint32_t crc);
    void send_cmd_response(uint8_t cmd, uint8_t rsp);
    void enqueue_next(float res, float temp);
    void send_cycle_data();
    void cycle_end();
    void init();
}

/***
 * Private methods: RECEIVE
 */

namespace receiver
{
    struct heater_params
    {
        float tempco;
        float rt_resistance;
        float rt_temp;
    };
    struct measure_params
    {
        float ref_resistance;
    };

    float get_next()
    {
        if (++cycle_counter >= CYCLE_LENGTH) cycle_end();
        return *current_element++;
    }

    void cycle_end()
    {
        cycle_counter = 0;
        current_element = current_buffer;
    }

    void process_args(uint8_t cmd, size_t& argument_index, size_t& stream_index, parser_state& state, uint8_t& response)
    {
        size_t lim = 0; //Last byte index (count - 1)
        switch (cmd)
        {
        case CMD_SET_TEMP_CYCLE: // DAC
            lim = sizeof(buffer1) - 1;
            reinterpret_cast<uint8_t*>(next_buffer)[argument_index] = receive_raw[stream_index];
            if (argument_index == lim)
            {
                auto temp = current_buffer;
                current_buffer = next_buffer;
                next_buffer = temp;
                cycle_end();
                transmitter::cycle_end();
                ESP_LOGI(TAG, "DAC loading finished");
                break;
            }
            break;
        case CMD_SET_HEATER_PARAMS:
        {
            static heater_params heater = {};
            lim = sizeof(heater) - 1;
            reinterpret_cast<uint8_t*>(&heater)[argument_index] = receive_raw[stream_index];
            if (argument_index == lim)
            {
                my_params::set_heater_coef(heater.tempco);
                my_params::set_rt_resistance(heater.rt_resistance, heater.rt_temp);
            }
            break;
        }
        case CMD_SET_MEASURE_PARAMS:
        {
            static measure_params measure = {};
            lim = sizeof(measure) - 1;
            reinterpret_cast<uint8_t*>(&measure)[argument_index] = receive_raw[stream_index];
            if (argument_index == lim)
            {
                my_params::set_ref_resistance(measure.ref_resistance);
            }
            break;
        }
        case CMD_SET_PID_PARAMS:
        {
            static my_pid_params_t pid = {};
            lim = sizeof(pid) - 1;
            reinterpret_cast<uint8_t*>(&pid)[argument_index] = receive_raw[stream_index];
            if (argument_index == lim)
            {
                my_params::set_pid_params(&pid);
            }
            break;
        }
        case CMD_SET_ADC_CAL:
        {
            static my_adc_cal_t cal = {};
            static uint8_t index = 0;
            lim = sizeof(cal); // plus 1 byte for index
            if (argument_index == 0)
            {
                index = receive_raw[stream_index];
            }
            else
            {
                reinterpret_cast<uint8_t *>(&cal)[argument_index - 1] = receive_raw[stream_index];
                if (argument_index == lim)
                {
                    my_params::set_adc_channel_cal(index, &cal);
                }
            }
            break;
        }
        case CMD_SET_DAC_CAL:
        {
            static my_dac_cal_t cal = {};
            lim = sizeof(cal) - 1;
            reinterpret_cast<uint8_t *>(&cal)[argument_index] = receive_raw[stream_index];
            if (argument_index == lim)
            {
                my_params::set_dac_cal(&cal);
            }
            break;
        }
        default:
            my_uart::raise_error(my_error_codes::unknown_cmd);
            state = parser_state::searching_for_preamble;
            return;
        }
        if (argument_index == lim)
        {
            state = parser_state::reading_counter;
            response = RSP_OK;
        }
    }

    void process_cmd(uint8_t cmd, size_t& stream_index, parser_state& state, uint8_t& response)
    {
        switch (cmd)
        {
#if ENABLE_DEBUG_INFO_CMD
        case 'I': // INFO
        {
            ESP_LOGI(TAG, "Info command received");
            transmitter::write_immedeately(reinterpret_cast<const uint8_t *>(&debug_prefix), sizeof(debug_prefix));
            break;
        }
#endif
        case CMD_STOP:
            if (my_uart::operate)
            {
                my_uart::operate = false;
                cycle_end();
                transmitter::cycle_end();
                response = RSP_OK;
                ESP_LOGI(TAG, "Cycle STOP.");
            }
            else
            {
                response = RSP_ALREADY_IN_REQUESTED_STATE;
            }
            break;
        case CMD_START: // START
            if (my_uart::operate)
            {
                response = RSP_ALREADY_IN_REQUESTED_STATE;
            }
            else
            {
                my_uart::operate = true;
                response = RSP_OK;
                ESP_LOGI(TAG, "Cycle START.");
            }
            break;
        case CMD_GET_DATA:
            transmitter::send_cycle_data();
            break;
        case CMD_GET_HAVE_DATA:
            response = transmitter::have_data ? RSP_OK : RSP_NO_DATA;
            break;
        case CMD_GET_ERROR:
            ESP_LOGI(TAG, "Current error flags: %x", static_cast<uint32_t>(my_uart::error_codes));
            transmitter::send_buffer(CMD_GET_ERROR,
                                     reinterpret_cast<uint8_t *>(&my_uart::error_codes),
                                     sizeof(my_uart::error_codes));
            my_uart::error_codes = my_error_codes::none;
            state = parser_state::reading_counter;
            break;
        case CMD_GET_NVS:
        {
            size_t len;
            uint8_t* buf = my_params::get_nvs_dump(&len);
            transmitter::send_buffer(CMD_GET_NVS, buf, len);
            break;
        }
        case CMD_SAVE_NVS:
            response = (my_params::save() == ESP_OK) ? RSP_OK : RSP_SET_FAILED;
            break;
        case CMD_ENABLE_PID_DBG:
            my_params::enable_pid_dbg = !my_params::enable_pid_dbg;
            response = my_params::enable_pid_dbg ? RSP_OK : RSP_NO_DATA;
            break;
        default:
            state = parser_state::reading_args;
            return;
        }
        state = parser_state::reading_counter;
    }

    void parse_input(size_t sz)
    {
        static parser_state state = parser_state::searching_for_preamble;
        static uint8_t cmd;
        static bool escape_state = false;
        static size_t argument_index;
        static bool crc_check = true;
        static uint8_t response = NO_STD_RSP;
        static bool cmd_complete;
        for (size_t stream_index = 0; stream_index < sz; stream_index++)
        {
            if (escape_state)
            {
                escape_state = false;
            }
            else
            {
                if (receive_raw[stream_index] == escape)
                {
                    escape_state = true;
                    ESP_LOGI(TAG, "Escape encountered: next byte %x", receive_raw[stream_index]);
                    continue;
                }
                else
                {
                    if (receive_raw[stream_index] == postamble) state = parser_state::postamble_encountered;
                    if (receive_raw[stream_index] == preamble) state = parser_state::searching_for_preamble;
                }
            }

            switch (state)
            {
            case parser_state::searching_for_preamble:
                if (receive_raw[stream_index] == preamble && !escape_state)
                {
                    state = parser_state::reading_cmd;
                    receiver_crc = ~0;
                    response = NO_STD_RSP;
                    cmd_complete = false;
                    ESP_LOGI(TAG, "Preamble encountered");
                }
                break;
            case parser_state::reading_cmd:
            {
                cmd = receive_raw[stream_index];
                argument_index = 0;
                crc_check = true;
                receiver_crc = crc32_le(receiver_crc, &cmd, sizeof(cmd));
                ESP_LOGI(TAG, "CMD ecnountered: %u", cmd);
                process_cmd(cmd, stream_index, state, response);
                break;
            }
            case parser_state::reading_args:
            {
                receiver_crc = crc32_le(receiver_crc, &(receive_raw[stream_index]), sizeof(receive_raw[stream_index]));
                process_args(cmd, argument_index, stream_index, state, response);
                argument_index++;
                break;
            }
            case parser_state::reading_counter:
            {
                uint8_t wdt = receive_raw[stream_index];
                if (wdt != ++receiver_wdt)
                {
                    receiver_wdt = wdt;
                    my_uart::raise_error(my_error_codes::missed_packet);
                    ESP_LOGI(TAG, "WDT error detected");
                }
                receiver_crc = ~crc32_le(receiver_crc, &receiver_wdt, sizeof(receiver_wdt));
                ESP_LOGI(TAG, "Calculated inbound CRC: %x", receiver_crc);
                ESP_LOGI(TAG, "WDT: %u", receiver_wdt);
                argument_index = 0;
                state = parser_state::reading_crc;
                break;
            }
            case parser_state::reading_crc:
                if (cmd_complete)
                {
                    my_uart::raise_error(my_error_codes::incorrect_command_format);
                    state = parser_state::searching_for_preamble;
                }
                else
                {
                    //ESP_LOGI(TAG, "CRC check: %x, index: %i", receive_raw[stream_index], argument_index);
                    crc_check = crc_check && 
                        (reinterpret_cast<uint8_t *>(&receiver_crc)[argument_index] == receive_raw[stream_index]);
                    if (++argument_index == sizeof(receiver_crc))
                    {
                        if (crc_check)
                        {
                            ESP_LOGI(TAG, "CRC OK");
                        }
                        else
                        {
                            response = RSP_BAD_CRC;
                            ESP_LOGW(TAG, "CRC ERROR");
                        }
                        if (response != NO_STD_RSP) transmitter::send_cmd_response(cmd, response);
                        cmd_complete = true;
                        //Should encounter the postamble byte next and automatically switch the state
                    }
                }
                break;
            case parser_state::postamble_encountered:
                if (!cmd_complete)
                {
                    my_uart::raise_error(my_error_codes::incorrect_command_format);
                }
                state = parser_state::searching_for_preamble;
                cmd_complete = false;
                ESP_LOGI(TAG, "Postamble encountered");
                break;
            default:
                my_uart::raise_error(my_error_codes::uart_parser_error);
                state = parser_state::searching_for_preamble;
                // TODO error handling
                break;
            }
        }
    }

    void init()
    {
        parser_semaphore = xSemaphoreCreateBinary();
        assert(parser_semaphore);
        xSemaphoreGive(parser_semaphore);
        parser_queue_handle = xQueueCreate(10, sizeof(size_t));
        assert(parser_queue_handle);
        xTaskCreatePinnedToCore(parser_task, "input_parser", 4096, NULL, 1, &parser_task_handle, 0);
        assert(parser_task_handle);
    }

    void parser_task(void *arg)
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
}

/***
 * Private methods: TRANSMIT
 */

namespace transmitter
{
    static uint32_t crc_dump_init_value;

    void write_dbg(float val)
    {
        write_immedeately(reinterpret_cast<uint8_t*>(&val), sizeof(val));
    }

    void write_immedeately(const uint8_t* buf, size_t sz)
    {
        tinyusb_cdcacm_write_queue(CDC_CHANNEL, buf, sz);
        tinyusb_cdcacm_write_flush(CDC_CHANNEL, 0);
    }
    
    void send_buffer(uint8_t cmd, uint8_t* buffer, size_t sz)
    {
        uint32_t crc = crc32_le(~0, &cmd, sizeof(cmd));
        crc = crc32_le(crc, buffer, sz);
        send_precalc_buffer(cmd, buffer, sz, crc);
    }

    void escape_helper(uint8_t*& current, uint8_t value)
    {
        switch (value)
        {
        case preamble:
        case postamble:
        case escape:
            *current++ = escape;
            break;
        default:
            break;
        }
        *current++ = value;
    }

    void send_precalc_buffer(uint8_t cmd, uint8_t* buffer, size_t sz, uint32_t crc)
    {
        static uint8_t escape_buffer[TRANSMIT_BUFFER_SIZE * sizeof(float) * 2];
        crc = ~crc32_le(crc, &wdt_counter, sizeof(wdt_counter));
        ESP_LOGI(TAG, "Outbound CRC: %x", crc);
        uint8_t* current = escape_buffer;
        *current++ = preamble;
        escape_helper(current, cmd);
        for (size_t i = 0; i < sz; i++)
        {
            escape_helper(current, buffer[i]);
        }
        escape_helper(current, wdt_counter);
        for (size_t i = 0; i < sizeof(crc); i++)
        {
            escape_helper(current, reinterpret_cast<uint8_t*>(&crc)[i]);
        }
        *current++ = postamble;
        tinyusb_cdcacm_write_queue(CDC_CHANNEL, escape_buffer, current - escape_buffer);
        tinyusb_cdcacm_write_flush(CDC_CHANNEL, 0);
        wdt_counter++;
        ESP_LOGI(TAG, "Sent a data packet.");
    }

    void send_cmd_response(uint8_t cmd, uint8_t rsp)
    {
        send_buffer(cmd, &rsp, sizeof(rsp));
    }

    void send_cycle_data()
    {
        xSemaphoreTake(transmit_mutex, portMAX_DELAY);

        send_precalc_buffer(CMD_GET_DATA, reinterpret_cast<uint8_t*>(full_buffer->start), sizeof(data_buffer1), full_buffer->crc);
        have_data = false;

        xSemaphoreGive(transmit_mutex);
    }

    void enqueue_next(float res, float temp)
    {
        if (++cycle_counter >= CYCLE_LENGTH) cycle_end();
        *empty_buffer->current++ = temp;
        *empty_buffer->current++ = res;
        empty_buffer->crc = crc32_le(empty_buffer->crc,
            reinterpret_cast<uint8_t*>(empty_buffer->current - FLOATS_PER_POINT), 
            sizeof(float) * FLOATS_PER_POINT);
        /*ESP_LOGI(TAG, "Incremental CRC: %x, float1: %x, float2: %x", ~empty_buffer->crc, 
            *reinterpret_cast<uint32_t*>(&temp), *reinterpret_cast<uint32_t*>(&res));*/
    }

    void cycle_end()
    {
        xSemaphoreTake(transmit_mutex, portMAX_DELAY);

        cycle_counter = 0;
        auto temp = full_buffer;
        full_buffer = empty_buffer;
        empty_buffer = temp;
        empty_buffer->crc = crc_dump_init_value;
        empty_buffer->current = empty_buffer->start;
        have_data = true;

        xSemaphoreGive(transmit_mutex);
    }

    void init()
    {
        static const uint8_t cmd_designator = CMD_GET_DATA;

        assert((sizeof(buffers) / sizeof(buffer_t*)) == buffers_count);
        assert(sizeof(data_buffer1) == sizeof(data_buffer2));
        transmit_mutex = xSemaphoreCreateMutex();
        assert(transmit_mutex);
        crc_dump_init_value = crc32_le(~0, &cmd_designator, sizeof(cmd_designator));
    }
}

/*****
 * Driver Callbacks
 */

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    static size_t rx_size = 0;

    xSemaphoreTake(receiver::parser_semaphore, portMAX_DELAY);
    /* read */
    esp_err_t ret = tinyusb_cdcacm_read((tinyusb_cdcacm_itf_t)itf, receiver::receive_raw, sizeof(receiver::receive_raw), &rx_size);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Data from channel %d: %i bytes.", itf, rx_size);
    }
    else
    {
        ESP_LOGE(TAG, "Read error");
    }
    xQueueSend(receiver::parser_queue_handle, &rx_size, portMAX_DELAY);
}

/***
 * Public
 */

namespace my_uart
{
    void fill_buffer_dbg(float start, float end) //linear interp
    {
        float inc = (end - start) / CYCLE_LENGTH;
        for (size_t i = 0; i < CYCLE_LENGTH; i++)
        {
            receiver::current_buffer[i] = start;
            start += inc;
        }
    }
    void raise_error(my_error_codes err)
    {
        error_codes |= err;
        ESP_LOGI(TAG, "Error raised: %x", static_cast<uint32_t>(error_codes));
    }
    bool get_operate()
    {
        return operate;
    }
    float first()
    {
        return receiver::get_next();
    }
    float next(float temp, float res)
    {
        transmitter::enqueue_next(res, temp);
        float ret = receiver::get_next();
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

        receiver::init();
        transmitter::init();
    }
    void send_pid_dbg(float temp, float voltage)
    {
        transmitter::write_dbg(debug_prefix);
        transmitter::write_dbg(temp);
        transmitter::write_dbg(voltage);
    }
}