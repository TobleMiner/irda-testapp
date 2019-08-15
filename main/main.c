#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_event.h>
#include <driver/pcnt.h>
#include <driver/uart.h>

#include "irda/irhal/irhal.h"
#include "irda/irphy/irphy.h"
#include "irda/irlap/irlap.h"
#include "irda/util/util.h"


#define IRDA_TX_GPIO 23
#define IRDA_RX_GPIO 22
#define IRDA_PC_GPIO 19

#define IRDA_UART    UART_NUM_2
#define IRDA_PC      PCNT_UNIT_0
#define IRDA_PC_CHAN PCNT_CHANNEL_0
#define IRDA_PC_TRSH 32
#define IRDA_PC_FLTR 20

esp_err_t event_handler(void *ctx, system_event_t *event) {
    return ESP_OK;
}

#define TAG "IRDA"


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
  TaskHandle_t main_task;
  uint32_t baudrate;
  struct irlap lap;
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

static int app_get_random_bytes(uint8_t* buf, size_t len, void* arg) {
  esp_fill_random(buf, len);
  return 0;
};

time_ns_t timeout = { .sec = 0, .nsec = 500000000UL };


static ssize_t irda_tx(const void* data, size_t len, void* priv);

static void timer_cb(void* ctx) {
  ESP_LOGI("IRDA TIMER", "Timer fired");
  irda_tx(str, strlen(str), NULL);
}

static void uart_event_task(void* arg);

static int enable_uart() {
  int err;
  uart_config_t uart_config = {
    .baud_rate = irda.baudrate,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  ESP_LOGI(TAG, "Enabling uart");
  err = -uart_param_config(IRDA_UART, &uart_config);
  if(err) {
    goto fail;
  }
  err = -uart_set_pin(IRDA_UART, IRDA_TX_GPIO, IRDA_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if(err) {
    goto fail;
  }
  err = -uart_driver_install(IRDA_UART, 256, 1024, 32, &irda.uart_queue, 0);
  if(err) {
    goto fail;
  }
  err = -uart_set_mode(IRDA_UART, UART_MODE_IRDA);
  if(err) {
    goto fail_driver;
  }

  UART2.conf0.irda_tx_en = 1;
  UART2.conf0.irda_dplx = 1;

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
  int err;
  err = uart_set_baudrate(IRDA_UART, rate);
  if(!err) {
    irda.baudrate = rate;
  }
  return -err;
}

static int irda_tx_enable(void* priv) {
  irda.tx_enabled = true;
  return 0;
}

static int irda_tx_disable(void* priv) {
  irda.tx_enabled = false;
  return 0;
}

static ssize_t irda_tx(const void* data, size_t len, void* priv) {
  return uart_write_bytes(IRDA_UART, (char*)data, len);
}

static int irda_rx_enable(const struct irphy* phy, irphy_rx_cb cb, void* priv) {
  irda.rx_cb = cb;
  irda.rx_enabled = true;
  return 0;
}

static int irda_rx_disable(void* priv) {
  irda.rx_enabled = false;
  irda.rx_cb = NULL;
  return 0;
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
    if(xQueueReceive(irda.uart_queue, (void*)&event, portMAX_DELAY)) {
      ESP_LOGI(TAG, "Got uart event");
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
          xQueueReset(irda.uart_queue);
          break;
        case UART_BUFFER_FULL:
          ESP_LOGW(TAG, "ring buffer full");
          uart_flush_input(IRDA_UART);
          xQueueReset(irda.uart_queue);
          break;
        case UART_EVENT_MAX:
          ESP_LOGI(TAG, "Terminating uart event task, MAX event received");
          xTaskNotifyGive(irda.main_task);
          break;
        default:
          ESP_LOGW(TAG, "Unhandled uart event %d", event.type);
          break;
      }
    }
  }
  
  vTaskDelete(NULL);
}

static void cd_task(void* arg) {
  while(1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Got carrier detect notification");
    if(irda.cd_cb) {
      irda.cd_cb(&irda.phy);
    }
  }
}

static void IRAM_ATTR cd_isr(void* priv) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(irda.cd_task, &xHigherPriorityTaskWoken);
  if(xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

static int irda_cd_enable(const struct irphy* phy, irphy_carrier_cb cb, void* priv) {
  int err;
  pcnt_config_t pc_conf = {
    .pulse_gpio_num = IRDA_PC_GPIO,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .pos_mode = PCNT_COUNT_DIS,
    .neg_mode = PCNT_COUNT_INC,
    .counter_h_lim = SHRT_MAX,
    .unit = IRDA_PC,
    .channel = IRDA_PC_CHAN,
  };
  irda.cd_cb = cb;
  irda.cd_enabled = true;

  err = pcnt_unit_config(&pc_conf);
  if(err) {
    irda.cd_enabled = false;
    goto fail;
  }
  pcnt_counter_clear(IRDA_PC);
  pcnt_set_event_value(IRDA_PC, PCNT_EVT_THRES_0, IRDA_PC_TRSH);
  pcnt_set_filter_value(IRDA_PC,IRDA_PC_FLTR);
  pcnt_intr_enable(IRDA_PC);
  pcnt_event_enable(IRDA_PC, PCNT_EVT_THRES_0);

  err = pcnt_isr_handler_add(IRDA_PC, cd_isr, NULL);
  if(err) {
    goto fail;
  }

  return 0;

fail:
  return -err;
}

static int irda_cd_disable(void* priv) {
  int err;
  pcnt_config_t pc_conf = {
    .pulse_gpio_num = PCNT_PIN_NOT_USED,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .pos_mode = PCNT_COUNT_DIS,
    .neg_mode = PCNT_COUNT_INC,
    .counter_h_lim = SHRT_MAX,
    .unit = IRDA_PC,
    .channel = IRDA_PC_CHAN,
  };
  err = pcnt_isr_handler_remove(IRDA_PC);
  if(err) {
    goto fail;
  }

  // Deconfigure pulse counter
  pcnt_unit_config(&pc_conf);

  irda.cd_enabled = false;

fail:
  return -err;
}

static void irda_carrier_cb(bool detected, void* arg) {
  ESP_LOGI(TAG, "carrier detect cb");
  ESP_LOGI(TAG, "Carrier detection finished, %s", detected ? "carrier detected" : "no carrier detected");
  ESP_ERROR_CHECK(irda_cd_disable(NULL));
}

int irda_tx_wait(void* arg) {
  return uart_wait_tx_done(IRDA_UART, portMAX_DELAY);
}

void irda_rx_cb(struct irphy* phy, size_t len) {
  uint8_t data[32];
  uint8_t* dptr = data;
  ESP_LOGI(TAG, "Received %zu bytes", len);
  ssize_t len_ = irda_rx(data, min(sizeof(data), len), NULL);
  while(len_-- > 0) {
    printf("%02x ", *dptr++);
  }
  printf("\n");
}

struct irda_lock {
  SemaphoreHandle_t lock;
};

int irda_lock_alloc(void** lock, void* priv) {
  int err;
  struct irda_lock* irdalock = malloc(sizeof(struct irda_lock));
  if(!irdalock) {
    err = -ENOMEM;
    goto fail;
  }

  irdalock->lock = xSemaphoreCreateMutex();
  if(!irdalock->lock) {
    err = -ENOMEM;
    goto fail_alloc;
  }

  *lock = irdalock;
  return 0;

fail_alloc:
  free(lock);
fail:
  return err;
}

void irda_lock_take(void* lock, void* priv) {
  struct irda_lock* irdalock = lock;
  xSemaphoreTake(irdalock->lock, portMAX_DELAY);
}

void irda_lock_put(void* lock, void* priv) {
  struct irda_lock* irdalock = lock;
  xSemaphoreGive(irdalock->lock);
}

void irda_lock_free(void* lock, void* priv) {
  struct irda_lock* irdalock = lock;
  vSemaphoreDelete(irdalock->lock);
  free(irdalock);
}

int app_main() {
  memset(&irda, 0, sizeof(irda));
  irda.main_task = xTaskGetCurrentTaskHandle();
  irda.baudrate = 9600;

  irda_set_baudrate(9600, NULL);

  esp_timer_create_args_t timer_args = {
    .callback = app_irda_timer_cb,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "IRDA timer"
  };

  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(pcnt_isr_service_install(0));

  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &irda.timer));

  struct irhal_hal_ops hal_ops = {
    .get_time = app_get_time,
    .set_alarm = app_start_irda_timer,
    .clear_alarm = app_clear_irda_timer,
    .log = irda_log,
    .get_random_bytes = app_get_random_bytes,
    .lock_alloc = irda_lock_alloc,
    .lock_free = irda_lock_free,
    .lock_take = irda_lock_take,
    .lock_put = irda_lock_put,
  };

  ESP_ERROR_CHECK(irhal_init(&irda.hal, &hal_ops, 1000000000ULL, 1000));
  enable_uart();

  irda_tx_enable(NULL);

  xTaskCreate(cd_task, "carrier_detect_task", 4096, NULL, 12, &irda.cd_task);

  struct irphy_hal_ops phy_hal_ops = {
    .set_baudrate = irda_set_baudrate,
    .tx_enable = irda_tx_enable,
    .tx = irda_tx,
    .tx_wait = irda_tx_wait,
    .tx_disable = irda_tx_disable,
    .rx_enable = irda_rx_enable,
    .rx = irda_rx,
    .rx_disable = irda_rx_disable,
    .cd_enable = irda_cd_enable,
    .cd_disable = irda_cd_disable,
  };

  ESP_ERROR_CHECK(irphy_init(&irda.phy, &irda.hal, &phy_hal_ops));

  struct irlap_ops lap_ops = {

  };

  ESP_ERROR_CHECK(irlap_init(&irda.lap, &irda.phy, &lap_ops, NULL));

//  irda_rx_enable(&irda.phy, irda_rx_cb, NULL);

  while(1) {
    char info[18] = "\004\000libirda on ESP32";
    time_ns_t cd_duration = { .sec = 1, .nsec = 0 };
//    irda_tx_enable(NULL);
//    int timerid = irhal_set_timer(&irda.hal, &timeout, timer_cb, NULL);
//    ESP_LOGI("IRDA TIMER", "Timer id: %d", timerid);
//    vTaskDelay(3000 / portTICK_PERIOD_MS);
//    ESP_LOGI(TAG, "Starting carrier detection");
//    ESP_ERROR_CHECK(irphy_run_cd(&irda.phy, &cd_duration, irda_carrier_cb, NULL));
//    irda_tx_disable(NULL);
//    ESP_ERROR_CHECK(irlap_send_xir(&irda.lap));
    ESP_ERROR_CHECK(irlap_discovery_request(&irda.lap.discovery, 6, (uint8_t*)info, sizeof(info)));
    vTaskDelay(5600 / portTICK_PERIOD_MS);
  }
}
