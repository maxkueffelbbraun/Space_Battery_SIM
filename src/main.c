#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <strings.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"

// Schematic net names are from the battery's view (TX_ESP = battery TX -> ESP RX).
#define PIN_BAT_RX      GPIO_NUM_0
#define PIN_BAT_TX      GPIO_NUM_1
#define PIN_PWR_12V     GPIO_NUM_2
#define PIN_VMEAS       GPIO_NUM_3
#define PIN_ICONST      GPIO_NUM_4
#define PIN_ROUTE       GPIO_NUM_5   // high = battery <-> ESP, low = battery <-> PC (U4)
#define PIN_Q3          GPIO_NUM_14
#define PIN_Q4          GPIO_NUM_15
#define PIN_LED2        GPIO_NUM_18
#define PIN_LED1        GPIO_NUM_19
#define PIN_DIP1        GPIO_NUM_20
#define PIN_DIP2        GPIO_NUM_21
#define PIN_DIP3        GPIO_NUM_22

#define BAT_UART        UART_NUM_1
#define VMEAS_DIVIDER   3.0f         // (R5 20k + R7 10k) / R7
#define LED_PULSE_MS    50

static QueueHandle_t uart_evt_q;
static uart_config_t ucfg = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};
static bool line_inverted;

static volatile bool quiet;
static volatile uint32_t cnt_bytes, cnt_errs;
static volatile int64_t led_rx_until, led_tx_until;

static adc_oneshot_unit_handle_t adc;
static adc_channel_t adc_ch;
static adc_cali_handle_t adc_cali;

static void dump(const char *tag, const uint8_t *d, int n)
{
    long long ms = esp_timer_get_time() / 1000;
    for (int off = 0; off < n; off += 16) {
        char hex[16 * 3 + 1] = {0}, asc[17] = {0};
        int m = n - off < 16 ? n - off : 16;
        for (int i = 0; i < m; i++) {
            uint8_t b = d[off + i];
            sprintf(&hex[i * 3], "%02X ", b);
            asc[i] = isprint(b) ? (char)b : '.';
        }
        printf("[%8lld] %s %3d: %-48s|%s|\n", ms, tag, m, hex, asc);
    }
}

static void rx_task(void *arg)
{
    uart_event_t ev;
    uint8_t buf[128];
    for (;;) {
        if (!xQueueReceive(uart_evt_q, &ev, portMAX_DELAY)) {
            continue;
        }
        switch (ev.type) {
        case UART_DATA: {
            size_t left = ev.size;
            while (left) {
                int r = uart_read_bytes(BAT_UART, buf, left < sizeof(buf) ? left : sizeof(buf), 0);
                if (r <= 0) {
                    break;
                }
                left -= r;
                cnt_bytes += r;
                if (!quiet) {
                    dump("RX", buf, r);
                }
            }
            led_rx_until = esp_timer_get_time() / 1000 + LED_PULSE_MS;
            break;
        }
        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
            cnt_errs++;
            if (!quiet) {
                printf("! %s error\n", ev.type == UART_FRAME_ERR ? "frame" : "parity");
            }
            break;
        case UART_BREAK:
            if (!quiet) {
                printf("! break\n");
            }
            break;
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            printf("! rx overflow, flushing\n");
            uart_flush_input(BAT_UART);
            xQueueReset(uart_evt_q);
            break;
        default:
            break;
        }
    }
}

static void bat_send(const uint8_t *d, int n)
{
    uart_write_bytes(BAT_UART, d, n);
    dump("TX", d, n);
    led_tx_until = esp_timer_get_time() / 1000 + LED_PULSE_MS;
}

static int vbat_mv(void)
{
    int sum = 0, raw, mv;
    for (int i = 0; i < 16; i++) {
        adc_oneshot_read(adc, adc_ch, &raw);
        sum += raw;
    }
    raw = sum / 16;
    if (adc_cali) {
        adc_cali_raw_to_voltage(adc_cali, raw, &mv);
    } else {
        mv = raw * 3300 / 4095;
    }
    return (int)(mv * VMEAS_DIVIDER);
}

static void hw_init(void)
{
    const gpio_num_t outs[] = {PIN_PWR_12V, PIN_ICONST, PIN_ROUTE, PIN_Q3, PIN_Q4, PIN_LED1, PIN_LED2};
    uint64_t out_mask = 0;
    for (int i = 0; i < sizeof(outs) / sizeof(outs[0]); i++) {
        gpio_set_level(outs[i], 0);
        out_mask |= 1ULL << outs[i];
    }
    // INPUT_OUTPUT so gpio_get_level() reports the driven state in `status`.
    gpio_config_t oc = {.pin_bit_mask = out_mask, .mode = GPIO_MODE_INPUT_OUTPUT};
    gpio_config(&oc);

    // DIP switches have external 10k pulldowns.
    gpio_config_t ic = {
        .pin_bit_mask = (1ULL << PIN_DIP1) | (1ULL << PIN_DIP2) | (1ULL << PIN_DIP3),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&ic);

    adc_unit_t unit;
    adc_oneshot_io_to_channel(PIN_VMEAS, &unit, &adc_ch);
    adc_oneshot_unit_init_cfg_t uc = {.unit_id = unit};
    adc_oneshot_new_unit(&uc, &adc);
    adc_oneshot_chan_cfg_t cc = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    adc_oneshot_config_channel(adc, adc_ch, &cc);
    adc_cali_curve_fitting_config_t cal = {
        .unit_id = unit, .chan = adc_ch, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal, &adc_cali) != ESP_OK) {
        adc_cali = NULL;
    }

    static const int dip_baud[8] = {9600, 19200, 38400, 57600, 115200, 2400, 4800, 1200};
    int dip = gpio_get_level(PIN_DIP1) | gpio_get_level(PIN_DIP2) << 1 | gpio_get_level(PIN_DIP3) << 2;
    ucfg.baud_rate = dip_baud[dip];

    uart_driver_install(BAT_UART, 2048, 1024, 32, &uart_evt_q, 0);
    uart_param_config(BAT_UART, &ucfg);
    uart_set_pin(BAT_UART, PIN_BAT_TX, PIN_BAT_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    gpio_set_level(PIN_ROUTE, 1);
}

static void print_status(void)
{
    const char par = ucfg.parity == UART_PARITY_EVEN ? 'E' : ucfg.parity == UART_PARITY_ODD ? 'O' : 'N';
    printf("UART1 %d 8%c1%s | route=%s | 12V=%d load=%d q3=%d q4=%d | Vbat=%d mV | rx=%lu err=%lu\n",
           ucfg.baud_rate, par, line_inverted ? " inverted" : "",
           gpio_get_level(PIN_ROUTE) ? "ESP" : "PC",
           gpio_get_level(PIN_PWR_12V), gpio_get_level(PIN_ICONST),
           gpio_get_level(PIN_Q3), gpio_get_level(PIN_Q4), vbat_mv(),
           (unsigned long)cnt_bytes, (unsigned long)cnt_errs);
}

static void print_help(void)
{
    printf("Commands:\n"
           "  status              show settings and counters\n"
           "  baud <n>            set battery UART baud rate\n"
           "  parity n|e|o        set parity\n"
           "  inv 0|1             invert RX/TX line levels\n"
           "  scan [ms]           count bytes/errors at common baud rates (battery must be sending)\n"
           "  tx <hex..>          send bytes, e.g. tx 55 AA 01 or tx 55AA01\n"
           "  txt <text>          send ASCII text (no line ending)\n"
           "  route esp|pc        U3: battery UART to ESP or isolated PC adapter\n"
           "  pwr 0|1             Q1: +12V to battery connector\n"
           "  load 0|1            constant current sink\n"
           "  q3 0|1, q4 0|1      low-side switches\n"
           "  vbat                measure battery voltage\n");
}

static int parse_onoff(const char *s)
{
    return s && (strcmp(s, "1") == 0 || strcasecmp(s, "on") == 0);
}

static void cmd_scan(int dwell_ms)
{
    static const int bauds[] = {1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200, 230400};
    int saved = ucfg.baud_rate;
    quiet = true;
    printf("baud      bytes   errors\n");
    for (int i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
        uart_set_baudrate(BAT_UART, bauds[i]);
        vTaskDelay(pdMS_TO_TICKS(30));
        uart_flush_input(BAT_UART);
        cnt_bytes = 0;
        cnt_errs = 0;
        vTaskDelay(pdMS_TO_TICKS(dwell_ms));
        printf("%-8d %6lu %8lu\n", bauds[i], (unsigned long)cnt_bytes, (unsigned long)cnt_errs);
    }
    uart_set_baudrate(BAT_UART, saved);
    cnt_bytes = 0;
    cnt_errs = 0;
    quiet = false;
    printf("Best candidate: many bytes, zero errors. Restored %d baud.\n", saved);
}

static void cmd_tx_hex(char *args)
{
    uint8_t out[256];
    int n = 0;
    for (char *tok = strtok(args, " ,"); tok && n < sizeof(out); tok = strtok(NULL, " ,")) {
        if (strncasecmp(tok, "0x", 2) == 0) {
            tok += 2;
        }
        size_t len = strlen(tok);
        if (len % 2) {
            out[n++] = (uint8_t)strtoul(tok, NULL, 16);
            continue;
        }
        for (size_t i = 0; i < len && n < sizeof(out); i += 2) {
            char pair[3] = {tok[i], tok[i + 1], 0};
            char *end;
            out[n++] = (uint8_t)strtoul(pair, &end, 16);
            if (*end) {
                printf("bad hex: %s\n", tok);
                return;
            }
        }
    }
    if (n) {
        bat_send(out, n);
    }
}

static void handle_line(char *line)
{
    char *cmd = strtok(line, " ");
    if (!cmd) {
        return;
    }
    char *rest = strtok(NULL, "");
    char *arg = rest;

    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
        print_help();
    } else if (!strcmp(cmd, "status")) {
        print_status();
    } else if (!strcmp(cmd, "baud") && arg) {
        int b = atoi(arg);
        if (b >= 300 && b <= 5000000 && uart_set_baudrate(BAT_UART, b) == ESP_OK) {
            ucfg.baud_rate = b;
        }
        print_status();
    } else if (!strcmp(cmd, "parity") && arg) {
        ucfg.parity = tolower((int)arg[0]) == 'e' ? UART_PARITY_EVEN
                    : tolower((int)arg[0]) == 'o' ? UART_PARITY_ODD : UART_PARITY_DISABLE;
        uart_set_parity(BAT_UART, ucfg.parity);
        print_status();
    } else if (!strcmp(cmd, "inv")) {
        line_inverted = parse_onoff(arg);
        uart_set_line_inverse(BAT_UART, line_inverted ? (UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV)
                                                      : UART_SIGNAL_INV_DISABLE);
        print_status();
    } else if (!strcmp(cmd, "scan")) {
        int ms = arg ? atoi(arg) : 1000;
        cmd_scan(ms > 0 ? ms : 1000);
    } else if (!strcmp(cmd, "tx") && arg) {
        cmd_tx_hex(arg);
    } else if (!strcmp(cmd, "txt") && arg) {
        bat_send((const uint8_t *)arg, strlen(arg));
    } else if (!strcmp(cmd, "route") && arg) {
        gpio_set_level(PIN_ROUTE, strcasecmp(arg, "pc") != 0);
        print_status();
    } else if (!strcmp(cmd, "pwr")) {
        gpio_set_level(PIN_PWR_12V, parse_onoff(arg));
        print_status();
    } else if (!strcmp(cmd, "load")) {
        gpio_set_level(PIN_ICONST, parse_onoff(arg));
        print_status();
    } else if (!strcmp(cmd, "q3")) {
        gpio_set_level(PIN_Q3, parse_onoff(arg));
        print_status();
    } else if (!strcmp(cmd, "q4")) {
        gpio_set_level(PIN_Q4, parse_onoff(arg));
        print_status();
    } else if (!strcmp(cmd, "vbat")) {
        printf("Vbat = %d mV\n", vbat_mv());
    } else {
        printf("unknown command, type 'help'\n");
    }
}

static void console_init(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&cfg);
    usb_serial_jtag_vfs_use_driver();
    setvbuf(stdout, NULL, _IONBF, 0);
}

void app_main(void)
{
    hw_init();
    console_init();
    xTaskCreate(rx_task, "bat_rx", 4096, NULL, 10, NULL);

    vTaskDelay(pdMS_TO_TICKS(500));
    printf("\nB.Braun battery UART test - ESP32-C6\n");
    print_status();
    print_help();
    printf("> ");

    char line[160];
    int len = 0;
    for (;;) {
        uint8_t c;
        if (usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(10)) == 1) {
            if (c == '\r' || c == '\n') {
                if (len) {
                    printf("\n");
                    line[len] = 0;
                    handle_line(line);
                    len = 0;
                    printf("> ");
                }
            } else if ((c == 0x08 || c == 0x7F) && len) {
                len--;
                printf("\b \b");
            } else if (isprint(c) && len < sizeof(line) - 1) {
                line[len++] = (char)c;
                putchar(c);
            }
        }

        int64_t now = esp_timer_get_time() / 1000;
        gpio_set_level(PIN_LED1, now < led_rx_until);
        gpio_set_level(PIN_LED2, now < led_tx_until);
    }
}
