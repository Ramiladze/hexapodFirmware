#include "esp_wifi.h"
#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Определения GPIO для камеры
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// SSID и пароль для подключения к Wi-Fi
char* ssid = "Duck-Duck";
const char* password = "DD3AC-bX";

// Объявление функций
void initCamera();
void connectToWiFi();
void startCameraServer();

// Функция установки
void setup() {
  // Отключение сброса при отключении питания
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Настройка последовательного порта для отладки
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  ledcSetup(7, 5000, 8);
  ledcAttachPin(4, 7);  // Пин 4 - светодиод.

  // Инициализация камеры, подключение к Wi-Fi и запуск сервера камеры
  initCamera();
  connectToWiFi();
  startCameraServer();
}

// Функция инициализации камеры
void initCamera() {
  // Конфигурация камеры
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Инициализация камеры
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Инициализация камеры завершилась ошибкой 0x%x", err);
    return;
  }

  // Настройка параметров камеры
  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
}

// Функция подключения к Wi-Fi
void connectToWiFi() {
  Serial.println("Подключение к Wi-Fi...");
  Serial.print("SSID: ");
  Serial.println(ssid);

  // Попытка подключения к Wi-Fi
  WiFi.begin(ssid, password);

  // Ожидание подключения к сети в течение 10 секунд
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  // Если подключение удалось
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nУспешно подключено к Wi-Fi!");
    Serial.print("IP адрес сервера: ");
    Serial.println(WiFi.localIP());
  } else {
    // Если подключение не удалось, активируйте режим точки доступа
    Serial.println("\nНе удалось подключиться к Wi-Fi. Включите точку доступа и перезагрузите устройство");
  }
}

void loop() {
  // delay(1000);
  // Serial.printf("RSSi: %ld dBm\n", WiFi.RSSI());
}
