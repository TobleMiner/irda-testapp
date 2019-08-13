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
#include "irda/irphy/irphy.h"


#define IRDA_TX_GPIO 23
#define IRDA_RX_GPIO 22

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
  irhal_alarm_cb alarm_cb;
  QueueHandle_t uart_queue;
  struct irphy phy;
  irphy_rx_cb rx_cb;
  TaskHandle_t cd_task;
  TaskHandle_t uart_event_task;
  irphy_carrier_cb cd_cb;
  bool rx_enabled;
  bool tx_enabled;
  bool cd_enabled;
  bool uart_enabled;
} irda;

static void app_irda_timer_cb(void* arg) {
  ESP_LOGI(TAG, "Calling alarm cb");
  irda.alarm_cb(&irda.hal);
}

static int app_start_irda_timer(struct irhal* hal, irhal_alarm_cb cb, uint64_t timeout, void* arg) {
  ESP_LOGI(TAG, "Scheduling timer in %llu µs", timeout);
  irda.alarm_cb = cb;
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

static void uart_event_task(void* arg);

static int enable_uart() {
  int err;
  err = -uart_set_pin(IRDA_UART, IRDA_TX_GPIO, IRDA_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if(err) {
    goto fail;
  }
  err = -uart_driver_install(IRDA_UART, 256, 1024, 0, NULL, 0);
  if(err) {
    goto fail;
  }
  err = -uart_set_mode(IRDA_UART, UART_MODE_IRDA);
  if(err) {
    goto fail_driver;
  }

  UART2.conf0.irda_tx_en = 1;

  if(xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, &irda.uart_event_task) != pdPASS) {
    err = -ENOMEM;
    goto fail_driver;
  }

  return 0;

fail_driver:
  uart_driver_delete(IRDA_UART);
fail:
  return err;
}

static int disable_uart() {
  if(&irda.uart_event_task != NULL) {
    vTaskDelete(&irda.uart_event_task);
  } else {
    ESP_LOGW(TAG,"UART task is NULL, skipping task deletion. BUG?");
  }
  uart_driver_delete(IRDA_UART);
  return 0;
}

static int update_uart_state() {
  int err;
  if((irda.rx_enabled || irda.tx_enabled) && !irda.cd_enabled) {
    if(irda.uart_enabled) {
      return 0;
    }
    err = enable_uart();
    if(!err) {
      irda.uart_enabled = true;
    }
    return err;
  } else {
    if(!irda.uart_enabled) {
      return 0;
    }
    err = disable_uart();
    if(!err) {
      irda.uart_enabled = false;
    }
    return err;
  }
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

static int irda_set_baudrate(uint32_t rate, void* priv) {
  return -uart_set_baudrate(IRDA_UART, rate);
}

static int irda_tx_enable(void* priv) {
  int err;
  irda.tx_enabled = true;
  err = update_uart_state();
  irda.tx_enabled = !err;
  return err;
}

static int irda_tx_disable(void* priv) {
  int err;
  irda.tx_enabled = false;
  err = update_uart_state();
  irda.tx_enabled = !!err;
  return err;
}

static ssize_t irda_tx(const void* data, size_t len, void* priv) {
  return uart_write_bytes(IRDA_UART, (char*)data, len);
}

static int irda_rx_enable(const struct irphy* phy, irphy_rx_cb cb, void* priv) {
  int err;
  irda.rx_cb = cb;
  irda.rx_enabled = true;
  err = update_uart_state();
  irda.rx_enabled = !err;
  return err;
}

static int irda_rx_disable(void* priv) {
  int err;
  irda.rx_cb = NULL;
  irda.rx_enabled = false;
  err = update_uart_state();
  irda.rx_enabled = !!err;
  return err;
}

static ssize_t irda_rx(void* data, size_t len, void* priv) {
  size_t buffered_len;
  uart_get_buffered_data_len(IRDA_UART, &buffered_len);
  if(buffered_len < len) {
    len = buffered_len;
  }
  return uart_read_bytes(IRDA_UART, (unsigned char*)data, len, 0);
}

static void uart_event_task(void* arg) {
  while(1) {
    uart_event_t event;
    if(xQueueReceive(&irda.uart_queue, (void*)&event, 1/pdMS_TO_TICKS());
      switch(event.type) {
        case UART_DATA:
          if(irda.rx_cb) {
            size_t buffered_len;
            uart_get_buffered_data_len(IRDA_UART, &buffered_len);
            irda.rx_cb(&irda.phy, buffered_len);
          }
          break;
        case UART_FIFO_OVF:
          ESP_LOGW(TAG, "hw fifo overflow");
          uart_flush_input(IRDA_UART);
          xQueueReset(&irda.uart_queue);
          break;
        case UART_BUFFER_FULL:
          ESP_LOGW(TAG, "ring buffer full");
          uart_flush_input(IRDA_UART);
          xQueueReset(&irda.uart_queue);
          break;
        default:
          ESP_LOGW(TAG, "Unhandled uart event %d", event.type);
          break;
      }
    } else {
      ESP_LOGI(TAG, "Failed to get event from uart queue, killed? Bailing out");
      break;
    }
  }
  vTaskDelete(NULL);
}

static void cd_task(void* arg) {
  while(1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(irda.cd_cb) {
      irda.cd_cb(&irda.phy);
    }
  }
}

static void IRAM_ATTR cd_isr(void* priv) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(&irda.cd_task, &xHigherPriorityTaskWoken);
  if(xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

static int irda_cd_enable(const struct irphy* phy, irphy_carrier_cb cb, void* priv) {
  int err;
  gpio_config_t gpio_conf = {
    .intr_type = GPIO_PIN_INTR_NEGEDGE,
    .pin_bit_mask = 1ULL << IRDA_RX_GPIO,
    .mode = GPIO_MODE_INPUT,
  };
  irda.cd_cb = cb;
  irda.cd_enabled = true;
  err = update_uart_state();
  if(err) {
    irda.cd_enabled = false;
    goto fail;
  }

  err = gpio_config(&gpio_conf);
  if(err) {
    goto fail_update_state;
  }
  err = gpio_isr_handler_add(IRDA_RX_GPIO, cd_isr, NULL);
  if(err) {
    goto fail_update_state;
  }

  return 0;

fail_update_state:
  irda.cd_enabled = false;
  err = update_uart_state();
  irda.cd_enabled = !!err;
fail:
  return -err;
}

static int irda_cd_disable(void* priv) {
  int err;
  err = gpio_isr_handler_remove(IRDA_RX_GPIO);
  if(err) {
    goto fail;
  }

  irda.cd_enabled = false;
  err = update_uart_state();
  irda.cd_enabled = !!err;

fail:
  return -err;
}

static void irda_carrier_cb(bool detected, void* arg) {
  ESP_LOGI(TAG, "carrier detect cb");
  ESP_LOGI(TAG, "Carrier detection finished, %s", detected ? "carrier detected" : "no carrier detected");
  ESP_ERROR_CHECK(irda_cd_disable(NULL));
}

int app_main() {
  memset(&irda, 0, sizeof(irda));

  gpio_install_isr_service(0);

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

//  int timerid = irhal_set_timer(&irda.hal, &timeout, timer_cb, NULL);
//  ESP_LOGI("IRDA TIMER", "Timer id: %d", timerid);

  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  ESP_ERROR_CHECK(uart_param_config(IRDA_UART, &uart_config));
  irda_tx_enable(NULL);

  xTaskCreate(cd_task, "carrier_detect_task", 4096, NULL, 12, &irda.cd_task);

  struct irphy_hal_ops phy_hal_ops = {
    .set_baudrate = irda_set_baudrate,
    .tx_enable = irda_tx_enable,
    .tx = irda_tx,
    .tx_disable = irda_tx_disable,
    .rx_enable = irda_rx_enable,
    .rx = irda_rx,
    .rx_disable = irda_rx_disable,
    .cd_enable = irda_cd_enable,
    .cd_disable = irda_cd_disable,
  };

  ESP_ERROR_CHECK(irphy_init(&irda.phy, &irda.hal, &phy_hal_ops));

  while(1) {
    time_ns_t cd_duration = { .sec = 1, .nsec = 0 };
    ESP_LOGI(TAG, "Starting carrier detection");
    ESP_ERROR_CHECK(irphy_run_cd(&irda.phy, &cd_duration, irda_carrier_cb, NULL));
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}
