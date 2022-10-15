#include "my_dbg_menu.h"
#include "my_params.h"
#include "my_uart.h"
#include "macros.h"

#include "esp_log.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "esp_chip_info.h"
#include "esp_sleep.h"
#include "esp_flash.h"
#include "sdkconfig.h"
#include "argtable3/argtable3.h"

#define PROMPT_STR CONFIG_IDF_TARGET

static const char* TAG = "DBG_MENU";
const char* prompt;
TaskHandle_t parser_task_handle;

static void initialize_console();

namespace my_dbg_commands
{
    /* 'version' command */
    static int get_version(int argc, char **argv)
    {
        const char *model;
        esp_chip_info_t info;
        uint32_t flash_size;
        esp_chip_info(&info);

        switch (info.model)
        {
        case CHIP_ESP32:
            model = "ESP32";
            break;
        case CHIP_ESP32S2:
            model = "ESP32-S2";
            break;
        case CHIP_ESP32S3:
            model = "ESP32-S3";
            break;
        case CHIP_ESP32C3:
            model = "ESP32-C3";
            break;
        case CHIP_ESP32H2:
            model = "ESP32-H2";
            break;
        default:
            model = "Unknown";
            break;
        }
        if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
        {
            printf("Get flash size failed");
            return 1;
        }
        printf("IDF Version:%s\r\n", esp_get_idf_version());
        printf("Chip info:\r\n");
        printf("\tmodel:%s\r\n", model);
        printf("\tcores:%d\r\n", info.cores);
        printf("\tfeature:%s%s%s%s%d%s\r\n",
               info.features & CHIP_FEATURE_WIFI_BGN ? "/802.11bgn" : "",
               info.features & CHIP_FEATURE_BLE ? "/BLE" : "",
               info.features & CHIP_FEATURE_BT ? "/BT" : "",
               info.features & CHIP_FEATURE_EMB_FLASH ? "/Embedded-Flash:" : "/External-Flash:",
               flash_size / (1024 * 1024), " MB");
        printf("\trevision number:%d\r\n", info.revision);
        return 0;
    }

    static int set_pid(int argc, char** argv)
    {
        my_pid_params_t buf = *my_params::get_pid_params();
        float* vals[] = { &buf.kPE, &buf.kPD, &buf.kI, &buf.limI, &buf.ambient_temp, &buf.timing_factor, &buf.setpoint_tolerance };
        if (argc > (ARRAY_SIZE(vals) + 1)) argc = ARRAY_SIZE(vals) + 1;
        int res = 0;
        for (size_t i = 1; i < argc; i++)
        {
            res += sscanf(argv[i], "%f", vals[i - 1]);
        }
        if (res > 0)
        {
            my_params::set_pid_params(&buf);
            return 0;
        }
        return 1;
    }

    static int set_rt_res(int argc, char** argv)
    {
        if (argc < 2) return 1;
        float res, temp;
        if (sscanf(argv[1], "%f,%f", &res, &temp) == 2)
        {
            my_params::set_rt_resistance(res, temp);
            return 0;
        }
        else
        {
            return 2;
        }
    }

    static int dump_nvs(int argc, char** argv)
    {
        for (size_t i = 0; i < MY_ADC_CHANNEL_NUM; i++)
        {
            auto ch = my_params::get_adc_channel_cal(i);
            printf("    ADC cal #%u: g=%f, o=%f", i, ch->gain, ch->offset);
        }
        auto pid = my_params::get_pid_params();
        auto dac = my_params::get_dac_cal();
        printf("    RT heater res: %f\n"
            "   Heater alpha: %f\n"
            "   DAC cal: g=%f, o=%f\n"
            "   PID coefs: I=%f, limI=%f, PE=%f, PD=%f, amb=%f, tim=%f, tol=%f\n",
            my_params::get_rt_resistance(),
            my_params::get_heater_coef(),
            dac->gain, dac->offset,
            pid->kI, pid->limI, pid->kPE, pid->kPD, pid->ambient_temp, pid->timing_factor, pid->setpoint_tolerance);
        return 0;
    }

    static int operate(int argc, char** argv)
    {
        my_dbg_menu::operate = !my_dbg_menu::operate;
        return 0;
    }

    static int set_profile(int argc, char** argv)
    {
        if (argc < 2) return 1;
        float start, end;
        if (sscanf(argv[1], "%f,%f", &start, &end) == 2)
        {
            my_uart::fill_buffer_dbg(start, end);
            return 0;
        }
        else
        {
            return 2;
        }
    }

    static int reset_nvs(int argc, char** argv)
    {
        return my_params::factory_reset();
    }

    static int save_nvs(int argc, char** argv)
    {
        return my_params::save();
    }
}

const esp_console_cmd_t commands[] = 
{
    {
        .command = "version",
        .help = "Get version of chip and SDK",
        .hint = NULL,
        .func = &my_dbg_commands::get_version
    },
    {
        .command = "set_pid",
        .help = "Set PID coefficients",
        .hint = NULL,
        .func = &my_dbg_commands::set_pid
    },
    {
        .command = "dump_nvs",
        .help = "Dump NVS data",
        .hint = NULL,
        .func = &my_dbg_commands::dump_nvs
    },
    {
        .command = "set_rt_res",
        .help = "Set RT heater resistance",
        .hint = NULL,
        .func = &my_dbg_commands::set_rt_res
    },
    {
        .command = "operate",
        .help = "Toggle operation",
        .hint = NULL,
        .func = &my_dbg_commands::operate
    },
    {
        .command = "set_profile",
        .help = "Set temperature profile (linear interpolation of 2 endpoints)",
        .hint = NULL,
        .func = &my_dbg_commands::set_profile
    },
    {
        .command = "reset_nvs",
        .help = "Erase NVS storage section (reset required to load defaults)",
        .hint = NULL,
        .func = &my_dbg_commands::reset_nvs
    },
    {
        .command = "save_nvs",
        .help = "Save configuration to NVS",
        .hint = NULL,
        .func = &my_dbg_commands::save_nvs
    }
};

static void initialize_console()
{
    setvbuf(stdin, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM,
						256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    /* Initialize the console */
    esp_console_config_t console_config = {
            .max_cmdline_length = 256,
            .max_cmdline_args = 8,
#if CONFIG_LOG_COLORS
            .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK( esp_console_init(&console_config) );

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(32);

    /* Set command maximum length */
    linenoiseSetMaxLineLen(console_config.max_cmdline_length);

    /* Don't return empty lines */
    linenoiseAllowEmpty(false);

#if CONFIG_STORE_HISTORY
    /* Load command history from filesystem */
    linenoiseHistoryLoad(HISTORY_PATH);
#endif

    /* Register commands */
    esp_console_register_help_command();
    for (auto &&i : commands)
    {
        esp_console_cmd_register(&i);
    }

    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    prompt = LOG_COLOR_I PROMPT_STR "> " LOG_RESET_COLOR;

    printf("\n"
           "Type 'help' to get the list of commands.\n"
           "Use UP/DOWN arrows to navigate through command history.\n"
           "Press TAB when typing command name to auto-complete.\n");

    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = PROMPT_STR "> ";
#endif //CONFIG_LOG_COLORS
    }

}

void parser_task(void *arg)
{
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed.
         */
        char *line = linenoise(prompt);
        if (line == NULL)
        { /* Break on EOF or error */
            continue;
        }
        /* Add the command to the history if not empty*/
        if (strlen(line) > 0)
        {
            linenoiseHistoryAdd(line);
#if CONFIG_STORE_HISTORY
            /* Save command history to filesystem */
            linenoiseHistorySave(HISTORY_PATH);
#endif
        }

        /* Try to run the command */
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGW(TAG, "Unrecognized command\n");
        }
        else if (err == ESP_ERR_INVALID_ARG)
        {
            // command was empty
        }
        else if (err == ESP_OK && ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
        }
        else if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Internal error: %s\n", esp_err_to_name(err));
        }
        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }
}

namespace my_dbg_menu
{
    bool operate = false;

    void init()
    {
        initialize_console();
        xTaskCreate(parser_task, "my_dbg_parser", 10000, NULL, 1, &parser_task_handle);
    }
}