#include <Arduino.h>
#include <AsyncElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>

#include "Sensor.hpp"
#include "Update.h"  // Somehow fixes failure to find "Update.h" in AsyncElegantOTA
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "index_html.h"
#include "packets.h"
#include "pins.h"

// #define WIFI_EN
#ifdef WIFI_EN
#include "credentials.h"
#endif

const char *TAG = "main";

const char *WIFI_AP_SSID = "sensor";
const char *WIFI_AP_PASSWORD = "absolutely";

AsyncWebServer server(80);

struct {
  uint8_t target_status[64];
  int16_t distance[64];
} sensor_data;

packet_status sensor_status = initialising;

uint8_t node_id = 0;

Preferences prefs;

void uart_task(void *arg) {
  QueueHandle_t uart_queue;

  uart_config_t uart_config = {
      .baud_rate = UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
      .source_clk = UART_SCLK_APB,
  };

  ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RX_BUFFER_LEN, 0, 16, &uart_queue, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM, PIN_TXD, PIN_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_set_mode(UART_NUM, UART_MODE_RS485_HALF_DUPLEX));
  uart_enable_pattern_det_baud_intr(UART_NUM, UART_RX_PAT_CHR, UART_RX_PAT_COUNT, 9, 0, 0);

  while (1) {
    char buf[UART_RX_BUFFER_LEN];
    packet_t *pkt;
    packet_t pktr;

    uart_event_t evt;
    if (xQueueReceive(uart_queue, (void *)&evt, portMAX_DELAY) == pdTRUE) {
      if (evt.type == UART_PATTERN_DET) {
        int pos = uart_pattern_pop_pos(UART_NUM);
        if (pos == -1) {
          ESP_LOGE(TAG, "uart queue overflow");
          uart_flush_input(UART_NUM);
          continue;
        }

        uart_read_bytes(UART_NUM, buf, pos + UART_RX_PAT_COUNT, 0);
        // ESP_LOGI(TAG, "read %d bytes", pos);

        pkt = (packet_t *)buf;
        if (pkt->header != PACKET_HEADER) {
          ESP_LOGW(TAG, "expected header of %08x but got %08x instead", PACKET_HEADER, pkt->header);
          continue;
        }

        if (pkt->destination != 10 + node_id) {
          continue;
        }

        if (pkt->type != request) {
          continue;
        }

        // ESP_LOGI(TAG, "read request received: %d", pkt->request.type);

        switch (pkt->request.type) {
          case status:
            pktr.destination = 0xFF;
            pktr.type = status;
            pktr.status.status = sensor_status;

            uart_write_bytes(UART_NUM, &pktr, PACKET_COMMON_LEN + sizeof(packet_status_t));
            uart_write_bytes(UART_NUM, &PACKET_TRAILER, sizeof(PACKET_TRAILER));
            break;
          case data:
            pktr.destination = 0xFF;
            pktr.type = data;

            pktr.data.status = sensor_status;
            memcpy(&pktr.data.target_status, &sensor_data.target_status, sizeof(sensor_data.target_status));
            memcpy(&pktr.data.distances, &sensor_data.distance, sizeof(sensor_data.distance));

            uart_write_bytes(UART_NUM, &pktr, PACKET_COMMON_LEN + sizeof(packet_data_t));
            uart_write_bytes(UART_NUM, &PACKET_TRAILER, sizeof(PACKET_TRAILER));
            break;
          default:
            break;
        }
      }
    }
  }
}

void sensor_task(void *arg) {
  pinMode(PIN_I2C_RST, OUTPUT);
  sensor_status = initialising;

  while (1) {
    digitalWrite(PIN_I2C_RST, HIGH);
    delay(50);
    digitalWrite(PIN_I2C_RST, LOW);
    delay(50);

    if (Sensor::initialize() != 0) {
      sensor_status = initialising_failure;
      delay(500);
    } else {
      break;
    }
  }

  sensor_status = active;

  while (1) {
    Sensor::read(sensor_data.target_status, sensor_data.distance);
    delay(10);
  }
}

void led_task(void *arg) {
  constexpr int LEDC_CHANNEL = 0;
  constexpr int LEDC_FREQ = 1000;
  constexpr int LEDC_RESOLUTION = 8;
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ, LEDC_RESOLUTION);
  ledcAttachPin(PIN_LED, LEDC_CHANNEL);

  int state = 0;
  while (1) {
    if (state) {
      ledcWrite(LEDC_CHANNEL, 255);
    } else {
      ledcWrite(LEDC_CHANNEL, 0);
    }

    if (sensor_status == initialising) {
      delay(500);
    } else if (sensor_status == initialising_failure) {
      delay(100);
    } else {
      ledcWrite(LEDC_CHANNEL, 4);
      vTaskDelete(nullptr);
    }

    state = !state;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);

  prefs.begin("prefs", false);
  node_id = prefs.getUChar("id", 0);

  ESP_LOGI(TAG, "id=%d", node_id);

  xTaskCreate(uart_task, "uart", 8192, nullptr, 10, nullptr);
  xTaskCreate(sensor_task, "sensor", 8192, nullptr, 15, nullptr);
  xTaskCreate(led_task, "led", 8192, nullptr, 5, nullptr);

#ifdef WIFI_EN
  WiFi.mode(WIFI_STA);
  ESP_LOGI(TAG, "Trying to connect to %s...", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    ESP_LOGW(TAG, "WiFi connection failed!");
    WiFi.disconnect();

    char ssid[32];
    snprintf(ssid, sizeof(ssid), "%s%d", WIFI_AP_SSID, node_id);

    ESP_LOGI(TAG, "Starting AP %s", ssid);
    WiFi.softAP(ssid, WIFI_AP_PASSWORD);
  } else {
    ESP_LOGI(TAG, "IP: %s", WiFi.localIP().toString());
  }

  MDNS.begin("sensor");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  server.on("/sensor", HTTP_GET, [](AsyncWebServerRequest *request) {
    char buf[1024];
    uint16_t len = 0;

    buf[len++] = '[';

    for (uint8_t i = 0; i < 64; i++) {
      len += snprintf(buf + len, sizeof(buf) - len, "[%d,%d],", sensor_data.target_status[i], sensor_data.distance[i]);
    }

    // Replace last , with ]
    buf[len - 1] = ']';

    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", buf);
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
  });

  AsyncElegantOTA.begin(&server);
  server.begin();
#endif
}

void loop() {
  constexpr int LEN_BUF = 64;
  static char buf[LEN_BUF];
  static uint8_t buf_count = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      buf[buf_count] = '\0';
      int id;
      if (sscanf(buf, "id=%d", &id) == 1) {
        prefs.putUChar("id", id);
        node_id = id;
        ESP_LOGI(TAG, "set id=%d", id);
      }

      buf_count = 0;
    } else {
      buf[buf_count++] = c;
    }
  }
}
