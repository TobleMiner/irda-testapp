#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_event.h>
#include <driver/uart.h>

#include "irda/irhal/irhal.h"

esp_err_t event_handler(void *ctx, system_event_t *event) {
    return ESP_OK;
}

#define TAG "IRDA"

#define IRDA_UART UART_NUM_2

const char* str = "This is an IRDA SIR message\n\r";

static uint64_t app_get_time(void* priv) {
  return esp_timer_get_time();
}

struct {
  esp_timer_handle_t timer;
  struct irhal hal;
  irhal_alarm_cb cb;
} irda;

static void app_irda_timer_cb(void* arg) {
  ESP_LOGI(TAG, "Calling alarm cb");
  irda.cb(&irda.hal);
}

static int app_start_irda_timer(struct irhal* hal, irhal_alarm_cb cb, uint64_t timeout, void* arg) {
  ESP_LOGI(TAG, "Scheduling timer in %llu µs", timeout);
  irda.cb = cb;
  return -esp_timer_start_once(irda.timer, timeout);
}

static int app_clear_irda_timer(void* arg) {
  return -esp_timer_stop(irda.timer);
}

time_ns_t timeout = { .sec = 0, .nsec = 500000000UL };


static void timer_cb(void* ctx);

static void timer_cb(void* ctx) {
  ESP_LOGI("IRDA TIMER", "Timer fired");
  uart_write_bytes(IRDA_UART, str, strlen(str));
  int timerid = irhal_set_timer(&irda.hal, &timeout, timer_cb, NULL);
  ESP_LOGI("IRDA TIMER", "Timer id: %d", timerid);
}

#define IRDA_LOG_LEVEL IRHAL_LOG_LEVEL_VERBOSE

static char print_buf[1024];
static void irda_log(void* priv, int level, const char* tag, const char* fmt, ...) {
  if(level > IRDA_LOG_LEVEL) {
    return;
  }
  va_list arg;
  va_start(arg, fmt);
  vsnprintf(print_buf, sizeof(print_buf), fmt, arg);
  va_end(arg);
  print_buf[sizeof(print_buf) - 1] = 0;
  switch(level) {
    case IRHAL_LOG_LEVEL_ERROR:
      ESP_LOGE(tag, "%s", print_buf);
      break;
    case IRHAL_LOG_LEVEL_WARNING:
      ESP_LOGW(tag, "%s", print_buf);
      break;
    case IRHAL_LOG_LEVEL_INFO:
      ESP_LOGI(tag, "%s", print_buf);
      break;
    case IRHAL_LOG_LEVEL_DEBUG:
      ESP_LOGI(tag, "%s", print_buf);
      break;
    case IRHAL_LOG_LEVEL_VERBOSE:
      ESP_LOGI(tag, "%s", print_buf);
      break;
  }
}

int app_main() {
  esp_timer_create_args_t timer_args = {
    .callback = app_irda_timer_cb,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "IRDA timer"
  };


  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &irda.timer));

  struct irhal_hal_ops hal_ops = {
    .get_time = app_get_time,
    .set_alarm = app_start_irda_timer,
    .clear_alarm = app_clear_irda_timer,
    .log = irda_log,
  };

  ESP_ERROR_CHECK(irhal_init(&irda.hal, &hal_ops, 1000000000ULL, 1000));

  int timerid = irhal_set_timer(&irda.hal, &timeout, timer_cb, NULL);
  ESP_LOGI("IRDA TIMER", "Timer id: %d", timerid);

  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  ESP_ERROR_CHECK(uart_param_config(IRDA_UART, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(IRDA_UART, 23, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(IRDA_UART, 256, 1024, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_set_mode(IRDA_UART, UART_MODE_IRDA));

  UART2.conf0.irda_tx_en = 1;

  while(1) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
