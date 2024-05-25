#include "dl_lib_matrix3d.h"
#include <esp32-hal-ledc.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  int64_t fr_start = esp_timer_get_time();
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Не удалось захватить камеру");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  size_t out_len, out_width, out_height;
  uint8_t *out_buf;
  bool s;
  {
    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_JPEG) {
      fb_len = fb->len;
      res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
      jpg_chunking_t jchunk = { req, 0 };
      res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
      httpd_resp_send_chunk(req, NULL, 0);
      fb_len = jchunk.len;
    }
    esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
    return res;
  }
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  if (!image_matrix) {
    esp_camera_fb_return(fb);
    Serial.println("dl_matrix3du_alloc failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  out_buf = image_matrix->item;
  out_len = fb->width * fb->height * 3;
  out_width = fb->width;
  out_height = fb->height;
  s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
  esp_camera_fb_return(fb);
  if (!s) {
    dl_matrix3du_free(image_matrix);
    Serial.println("to rgb888 failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  jpg_chunking_t jchunk = { req, 0 };
  s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
  dl_matrix3du_free(image_matrix);
  if (!s) {
    Serial.println("Сбой сжатия JPEG");
    return ESP_FAIL;
  }
  int64_t fr_end = esp_timer_get_time();
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];
  dl_matrix3du_t *image_matrix = NULL;
  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Не удалось захватить камеру");
      res = ESP_FAIL;
    } else {
      {
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            Serial.println("Сбой сжатия JPEG");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
  }
  last_frame = 0;
  return res;
}

enum state { fwd, rev, stp };
state actstate = stp;

static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf;
  size_t buf_len;
  char variable[32] = { 0, };
  char value[32] = { 0, };
  char value2[32] = { 0, };
  char value3[32] = { 0, };
  char value4[32] = { 0, };
  char value5[32] = { 0, };
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK && httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK || httpd_query_key_value(buf, "x2", value2, sizeof(value2)) == ESP_OK && httpd_query_key_value(buf, "y2", value3, sizeof(value3)) == ESP_OK || (httpd_query_key_value(buf, "x1", value4, sizeof(value4)) == ESP_OK && httpd_query_key_value(buf, "y1", value5, sizeof(value5)) == ESP_OK)) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  int val = atoi(value);
  int val2 = atoi(value2);
  int val3 = atoi(value3);
  int val4 = atoi(value4);
  int val5 = atoi(value5);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;
  if (!strcmp(variable, "framesize")) {
    Serial.println("framesize");
    if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
  } else if (!strcmp(variable, "quality")) {
    Serial.println("quality");
    res = s->set_quality(s, val);
  } else if (!strcmp(variable, "gamepad")) {
    Serial.print("x1: ");
    Serial.println(val4);
    Serial.print("y1: ");
    Serial.println(val5);
    Serial.print("x2: ");
    Serial.println(val2);
    Serial.print("y2: ");
    Serial.println(val3);
  } else if (!strcmp(variable, "flash")) {
    Serial.println(val);
    ledcWrite(7, val);
  } else if (val == 6) {
    Serial.println("Перезагрузка");
    ESP.restart();
  } else {
    Serial.println("variable");
    res = -1;
  }
  if (res) {
    return httpd_resp_send_500(req);
  }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];
  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  *p++ = '{';
  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
    <!DOCTYPE html>
    <html lang="ru">
    <head>
    <meta charset="utf-8">
    <!-- Добавлен метатег для корректного масштабирования на мобильных устройствах -->
    <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
    <title>Hexapod web interface</title>
    <!-- Внешние стили -->
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css" rel="stylesheet">
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.11.3/font/bootstrap-icons.min.css">
    <!-- Внутренние стили -->
    <style>
        /* Общие стили */
        body {
            color: #ffffff;
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 0;
        }
        /* Стили для блока с кнопками */
        #buttons {
            width: 70vw;
            /* Ширина 70% от ширины экрана */
            margin: 0 auto;
            /* Центрирование */
            margin-bottom: 20px;
            /* Отступ между блоками */
        }
        /* Кнопки */
        button {
            border-radius: 15px;
            width: 100%;
        }
        button:hover {
            background-color: #666666;
        }
        /* Стили для ползунков */
        input[type="range"] {
            -webkit-appearance: none;
            width: 100%;
            margin: 8px 0;
        }
        input[type="range"]:focus {
            outline: none;
        }
        input[type="range"]::-webkit-slider-runnable-track {
            width: 100%;
            height: 10px;
            cursor: pointer;
            background: #333333;
            /* Цвет полосы */
        }
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            background: #ffffff;
            /* Цвет бегунка */
            cursor: pointer;
            border-radius: 50%;
            margin-top: -5px;
            box-shadow: 0px 0px 5px #ffffff;
            /* Тень */
        }
        input[type="range"]::-moz-range-track {
            width: 100%;
            height: 10px;
            cursor: pointer;
            background: #333333;
            /* Цвет полосы */
        }
        input[type="range"]::-moz-range-thumb {
            width: 20px;
            height: 20px;
            background: #ffffff;
            /* Цвет бегунка */
            cursor: pointer;
            border-radius: 50%;
            margin-top: -5px;
            box-shadow: 0px 0px 5px #ffffff;
            /* Тень */
        }
        .offcanvas {
            --bs-offcanvas-height: 40vh;
        }
        /* Стили для блока с джойстиками */
        .joyDiv {
            width: 200px;
            height: 200px;
            margin: 50px 0;
        }
        .joyStickCanvas {
            border: 1px solid;
        }
        /* Стили для блока с видео */
        #stream-container {
            margin-top: 50px;
            margin-bottom: 50px;
            text-align: center;
            width: 70vh;
            height: 40vh;
        }
        #stream-container img {
            max-width: 100%;
            max-height: 100%;
        }
        #stream img {
            width: 70vh;
            height: 40vh;
        }
        #stream {
            width: 70vh;
            height: 40vh;
        }
        /* Дополнительные стили для маленьких экранов */
        @media (max-width: 768px) {
            .joyDiv {
                width: 150px;
                height: 150px;
                margin: 5% auto;
            }
            #stream-container {
                margin-top: 25px;
                margin-bottom: 25px;
            }
        }
        @media (max-height: 770px) {
            .offcanvas {
                --bs-offcanvas-height: 90vh;
                /* Измените значение высоты, как вам нужно */
            }
        }
        /* Стили для выравнивания джойстиков по краям */
        .row.justify-content-between {
            justify-content: space-between !important;
            margin-left: 0;
            margin-right: 0;
        }
        .row.justify-content-center {
            margin-left: 0;
            margin-right: 0;
            /* Установка нулевого отступа */
        }
    </style></head><body data-bs-theme="dark">
    <nav class="navbar navbar-expand bg-body-tertiary">
        <div class="container-fluid">
            <a class="navbar-brand" href="#">Панель управления</a>
            <button class="navbar-toggler" type="button" data-bs-toggle="collapse"
                data-bs-target="#navbarSupportedContent" aria-controls="navbarSupportedContent" aria-expanded="false"
                aria-label="Toggle navigation">
                <span class="navbar-toggler-icon"></span>
            </button>
            <ul class="navbar-nav ms-auto mb-2 mb-lg-0">
                <li class="nav-item">
                    <button class="btn btn-primary float-end" type="button" data-bs-toggle="offcanvas"
                        data-bs-target="#offcanvasTop" aria-controls="offcanvasTop">Управление камерой</button>
                </li>
            </ul>
        </div>
    </nav>
        <div class="row d-flex justify-content-between">
            <!-- Левый джойстик -->
            <div class="col-sm">
                <div class="joyDiv" id="joy1Div"></div>
                <input id="joy1X" type="text" class="form-control mb-2" placeholder="X">
                <input id="joy1Y" type="text" class="form-control" placeholder="Y">
            </div>
            <!-- Блок с видео -->
            <div class="flex-grow-1" id="stream-container">
                    <img id="stream" src="">
            </div>
            <!-- Правый джойстик -->
            <div class="col-sm">
                <div class="joyDiv" id="joy2Div"></div>
                <input id="joy2X" type="text" class="form-control mb-2" placeholder="X">
                <input id="joy2Y" type="text" class="form-control" placeholder="Y">
            </div>
        </div>
    <!-- Основная секция -->
    <section class="main mt-3">
        <div class="offcanvas offcanvas-top custom-offcanvas" tabindex="-1" id="offcanvasTop"
            aria-labelledby="offcanvasTopLabel">
            <div class="offcanvas-header">
                <h5 class="offcanvas-title" id="offcanvasTopLabel">Управление камерой</h5>
                <button type="button" class="btn-close" data-bs-dismiss="offcanvas" aria-label="Close"></button>
            </div>
            <div class="offcanvas-body">
                <!-- Секция с кнопками -->
                <section id="buttons">
                    <div class="container-fluid d-flex flex-column">
                        <!-- Изменена сетка для кнопок управления -->
                        <div class="row gap-2">
                            <!-- Кнопки управления -->
                            <div class="col-sm d-flex justify-content-center">
                                <button class="btn btn-sm btn-outline-warning" id="get-still"><i
                                        class="bi bi-camera"></i>Фото</button>
                            </div>
                            <div class="col-sm d-flex justify-content-center">
                                <button class="btn btn-sm btn-outline-primary" id="restart"
                                    onclick="fetch(document.location.origin+'/control?var=car&val=6');"><i
                                        class="bi bi-arrow-clockwise"></i>Перезагрузка</button>
                            </div>
                            <div class="col-sm d-flex justify-content-center">
                                <button class="btn btn-sm btn-outline-danger" id="toggle-stream"><i
                                        class="bi bi-camera-video-fill"></i>Вкл.стрим</button>
                            </div>
                        </div>
                    </div>
                </section>
                <!-- Таблица настроек -->
                <div class="container-fluid d-flex justify-content-center">
                    <table class="table table-striped" style="width: 70vw;">
                        <tbody>
                            <tr>
                                <td colspan="3">Свет</td>
                            </tr>
                            <tr>
                                <td align="center" colspan="3">
                                    <input type="range" id="flash" min="0" max="255" value="0"
                                        onchange="try{fetch(document.location.origin+'/control?var=flash&val='+this.value);}catch(e){}">
                                </td>
                            </tr>
                            <tr>
                                <td colspan="3">Разрешение</td>
                            </tr>
                            <tr>
                                <td align="center" colspan="3">
                                    <input type="range" id="framesize" min="0" max="6" value="6"
                                        onchange="try{fetch(document.location.origin+'/control?var=framesize&val='+this.value);}catch(e){}">
                                </td>
                            </tr>
                        </tbody>
                    </table>
                </div>
            </div>
        </div>
    </section>
    <!-- Скрипты -->
    <script>
        document.addEventListener('DOMContentLoaded', function () {
            // Функция для отправки запроса на сервер
            function sendRequest(element) {
                let valueToSend;
                // Определение значения в зависимости от типа элемента
                switch (element.type) {
                    case 'button':
                    case 'submit':
                        valueToSend = '1';
                        break;
                    default:
                        return;
                }
                // Формирование URL для запроса
                const url = `${baseUrl}/control?var=${element.id}&val=${valueToSend}`;
                // Отправка запроса
                fetch(url)
                    .then(response => {
                        console.log(`Request to ${url} finished, status: ${response.status}`);
                    });
            }
            // Определение базового URL
            const baseUrl = document.location.origin;
            // Функции для управления видимостью элементов
            const hideElement = element => {
                element.classList.add('hidden');
            };
            const showElement = element => {
                element.classList.remove('hidden');
            };
            // Функции для управления состоянием элементов
            const disableElement = element => {
                element.classList.add('disabled');
                element.disabled = true;
            };
            const enableElement = element => {
                element.classList.remove('disabled');
                element.disabled = false;
            };
            // Обработчики для кнопок закрытия потока видео
            document.querySelectorAll('.close').forEach(closeButton => {
                closeButton.onclick = () => {
                    hideElement(closeButton.parentNode);
                };
            });
            // Получение данных о статусе и их применение к элементам по умолчанию
            fetch(`${baseUrl}/status`)
                .then(response => response.json())
                .then(data => {
                    document.querySelectorAll('.default-action').forEach(element => {
                        applyStatus(element, data[element.id], false);
                    });
                });
            // Функция для применения статуса к элементу
            function applyStatus(element, status, isCheckbox) {
                if (isCheckbox) {
                    element.checked = status;
                } else {
                    element.value = status;
                }
            }
            // Определение элементов на странице
            const streamElement = document.getElementById('stream');
            const streamContainer = document.getElementById('stream-container');
            const getStillButton = document.getElementById('get-still');
            const toggleStreamButton = document.getElementById('toggle-stream');
            // Обработчик для кнопки получения фото
            getStillButton.onclick = () => {
                stopStream();
                streamElement.src = `${baseUrl}/capture?_cb=${Date.now()}`;
                showElement(streamContainer);
            };
            // Обработчик для кнопки переключения потока видео
            toggleStreamButton.onclick = () => {
                const isStreaming = toggleStreamButton.innerHTML === 'Выкл.стрим';
                isStreaming ? stopStream() : startStream();
            };
            // Функция для остановки потока видео
            function stopStream() {
                window.stop();
                toggleStreamButton.innerHTML = '<i class="bi bi-camera-video-fill"></i>Вкл.стрим';
            }
            // Функция для запуска потока видео
            function startStream() {
                streamElement.src = `${baseUrl}:81/stream`;
                showElement(streamContainer);
                toggleStreamButton.innerHTML = '<i class="bi bi-camera-video-off"></i>Выкл.стрим';
                stopStream();
            }
            // Обработчики для элементов управления настройками
            document.querySelectorAll('.default-action').forEach(element => {
                element.onchange = () => sendRequest(element);
            });
            // Обработчики для элементов управления автоматическими настройками
            const agcCheckbox = document.getElementById('agc');
            const agcGainGroup = document.getElementById('agc_gain-group');
            const gainCeilingGroup = document.getElementById('gainceiling-group');
            agcCheckbox.onchange = () => {
                sendRequest(agcCheckbox);
                if (agcCheckbox.checked) {
                    showElement(gainCeilingGroup);
                    hideElement(agcGainGroup);
                } else {
                    hideElement(gainCeilingGroup);
                    showElement(agcGainGroup);
                }
            };
            const aecCheckbox = document.getElementById('aec');
            const aecValueGroup = document.getElementById('aec_value-group');
            aecCheckbox.onchange = () => {
                sendRequest(aecCheckbox);
                aecCheckbox.checked ? hideElement(aecValueGroup) : showElement(aecValueGroup);
            };
            const awbGainCheckbox = document.getElementById('awb_gain');
            const wbModeGroup = document.getElementById('wb_mode-group');
            awbGainCheckbox.onchange = () => {
                sendRequest(awbGainCheckbox);
                awbGainCheckbox.checked ? showElement(wbModeGroup) : hideElement(wbModeGroup);
            };
            // Обработчик для выбора разрешения
            const frameSizeInput = document.getElementById('framesize');
            frameSizeInput.onchange = () => {
                sendRequest(frameSizeInput);
                if (frameSizeInput.value > 5) {
                    applyStatus(document.getElementById('face_enroll'), false, true);
                    applyStatus(document.getElementById('toggle-stream'), false, true);
                }
            };
        });
    </script>
    <script>
        let StickStatus = {
            x: 0,
            y: 0
        };
        var JoyStick = (function (container, parameters, callback) {
            parameters = parameters || {};
            var title = parameters.title || "joystick",
                width = parameters.width || 0,
                height = parameters.height || 0,
                internalFillColor = parameters.internalFillColor || "#00AA00",
                internalLineWidth = parameters.internalLineWidth || 2,
                internalStrokeColor = parameters.internalStrokeColor || "#003300",
                externalLineWidth = parameters.externalLineWidth || 2,
                externalStrokeColor = parameters.externalStrokeColor || "#008000",
                autoReturnToCenter = parameters.autoReturnToCenter !== undefined ? parameters.autoReturnToCenter : true;
            callback = callback || function (StickStatus) { };
            var objContainer = document.getElementById(container);
            objContainer.style.touchAction = "none";
            var canvas = document.createElement("canvas");
            canvas.id = title;
            width = width || objContainer.clientWidth;
            height = height || objContainer.clientHeight;
            canvas.width = width;
            canvas.height = height;
            objContainer.appendChild(canvas);
            var context = canvas.getContext("2d");
            var pressed = 0,
                circumference = 2 * Math.PI,
                internalRadius = (canvas.width - ((canvas.width / 2) + 10)) / 2,
                maxMoveStick = internalRadius + 5,
                externalRadius = internalRadius + 30,
                centerX = canvas.width / 2,
                centerY = canvas.height / 2,
                directionHorizontalLimitPos = canvas.width / 10,
                directionHorizontalLimitNeg = directionHorizontalLimitPos * -1,
                directionVerticalLimitPos = canvas.height / 10,
                directionVerticalLimitNeg = directionVerticalLimitPos * -1,
                movedX = centerX,
                movedY = centerY;
            if ("ontouchstart" in document.documentElement) {
                canvas.addEventListener("touchstart", onTouchStart, false);
                document.addEventListener("touchmove", onTouchMove, false);
                document.addEventListener("touchend", onTouchEnd, false);
            } else {
                canvas.addEventListener("mousedown", onMouseDown, false);
                document.addEventListener("mousemove", onMouseMove, false);
                document.addEventListener("mouseup", onMouseUp, false);
            }
            drawExternal();
            drawInternal();
            function drawExternal() {
                context.beginPath();
                context.arc(centerX, centerY, externalRadius, 0, circumference, false);
                context.lineWidth = externalLineWidth;
                context.strokeStyle = externalStrokeColor;
                context.stroke();
            }
            function drawInternal() {
                context.beginPath();
                movedX = Math.min(Math.max(movedX, maxMoveStick), canvas.width - maxMoveStick);
                movedY = Math.min(Math.max(movedY, maxMoveStick), canvas.height - maxMoveStick);
                context.arc(movedX, movedY, internalRadius, 0, circumference, false);
                var grd = context.createRadialGradient(centerX, centerY, 5, centerX, centerY, 200);
                grd.addColorStop(0, internalFillColor);
                grd.addColorStop(1, internalStrokeColor);
                context.fillStyle = grd;
                context.fill();
                context.lineWidth = internalLineWidth;
                context.strokeStyle = internalStrokeColor;
                context.stroke();
            }
            let touchId = null;
            function onTouchStart(event) {
                pressed = 1;
                touchId = event.targetTouches[0].identifier;
            }
            function onTouchMove(event) {
                if (pressed === 1 && event.targetTouches[0].target === canvas) {
                    movedX = event.targetTouches[0].pageX - canvas.offsetLeft;
                    movedY = event.targetTouches[0].pageY - canvas.offsetTop;
                    context.clearRect(0, 0, canvas.width, canvas.height);
                    drawExternal();
                    drawInternal();
                    StickStatus.x = ((movedX - centerX) / maxMoveStick * 100).toFixed();
                    StickStatus.y = ((movedY - centerY) / maxMoveStick * -100).toFixed();
                    callback(StickStatus);
                }
            }
            function onTouchEnd(event) {
                if (event.changedTouches[0].identifier !== touchId) return;
                pressed = 0;
                if (autoReturnToCenter) {
                    movedX = centerX;
                    movedY = centerY;
                }
                context.clearRect(0, 0, canvas.width, canvas.height);
                drawExternal();
                drawInternal();
                StickStatus.x = ((movedX - centerX) / maxMoveStick * 100).toFixed();
                StickStatus.y = ((movedY - centerY) / maxMoveStick * -100).toFixed();
                callback(StickStatus);
            }
            function onMouseDown(event) {
                pressed = 1;
            }
            function onMouseMove(event) {
                if (pressed === 1) {
                    movedX = event.pageX - canvas.offsetLeft;
                    movedY = event.pageY - canvas.offsetTop;
                    context.clearRect(0, 0, canvas.width, canvas.height);
                    drawExternal();
                    drawInternal();
                    StickStatus.x = ((movedX - centerX) / maxMoveStick * 100).toFixed();
                    StickStatus.y = ((movedY - centerY) / maxMoveStick * -100).toFixed();
                    callback(StickStatus);
                }
            }
            function onMouseUp(event) {
                pressed = 0;
                if (autoReturnToCenter) {
                    movedX = centerX;
                    movedY = centerY;
                }
                context.clearRect(0, 0, canvas.width, canvas.height);
                drawExternal();
                drawInternal();
                StickStatus.x = ((movedX - centerX) / maxMoveStick * 100).toFixed();
                StickStatus.y = ((movedY - centerY) / maxMoveStick * -100).toFixed();
                callback(StickStatus);
            }
        });
    </script>
    <script>
        var joy1X = document.getElementById("joy1X");
        var joy1Y = document.getElementById("joy1Y");
        var joy2X = document.getElementById("joy2X");
        var joy2Y = document.getElementById("joy2Y");
        JoyStick("joy1Div", {}, function (joy1Status) {
            joy1X.value = joy1Status.x;
            joy1Y.value = joy1Status.y;
                         try{fetch(document.location.origin+'/control?var=gamepad&x1='+joy1X.value+'&y1='+joy1Y.value);}catch(e){}
        });
        JoyStick("joy2Div", {}, function (joy2Status) {
            joy2X.value = joy2Status.x;
            joy2Y.value = joy2Status.y;
                        console.log("x: ",joy2X.value);
                        console.log("y: ",joy2Y.value);
                        try{fetch(document.location.origin+'/control?var=gamepad&x2='+joy2X.value+'&y2='+joy2Y.value);}catch(e){}
        });
    </script>
    <!-- Внешние скрипты -->
    <script src="https://cdn.jsdelivr.net/npm/@popperjs/core@2.11.8/dist/umd/popper.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/js/bootstrap.min.js"></script>
</body>
</html>
)rawliteral";
static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };
  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
  };
  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
  };
  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
  };
  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };
  Serial.printf("Запуск веб-сервера на порту: '%d'\n", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
  }
  config.server_port += 1;
  config.ctrl_port += 1;
  Serial.printf("Запуск потокового сервера на порту: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}
