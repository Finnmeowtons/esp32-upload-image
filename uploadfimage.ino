//======================================== Including the libraries.
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"

//========================================

//======================================== CAMERA_MODEL_AI_THINKER GPIO.
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
//========================================

// LED Flash PIN (GPIO 4)
#define FLASH_LED_PIN 4

//======================================== Insert your network credentials.
const char *ssid = "tp-link";
const char *password = "09270734452";

const char* mqtt_server = "157.245.204.46";
const int mqtt_port = 1883;
//========================================


//======================================== Variables for Timer/Millis.
unsigned long previousMillis = 0;
const int Interval = 60000;  //--> Photo capture every 60 seconds.
//========================================

// Server Address or Server IP.
String serverName = "157.245.204.46";  //--> Change with your server computer's IP address or your Domain name.
// The file path "upload_img.php" on the server folder.
String serverPath = "/upload?device=device4"; //TODO Change Device Number
// Server Port.
const int serverPort = 3000;

// Variable to set capture photo with LED Flash.
// Set to "false", then the Flash LED will not light up when capturing a photo.
// Set to "true", then the Flash LED lights up when capturing a photo.
bool LED_Flash_ON = true;

// Initialize WiFiClient.
WiFiClient client;

// Initialize HTTPClient.
HTTPClient http;

// HTTP server handle
httpd_handle_t server = NULL;

PubSubClient mqttClient(client);


bool streaming_enabled = true;

//________________________________________________________________________________ sendPhotoToServer()
void sendPhotoToServer() {
  
  String AllData;
  String DataBody;

  Serial.println();
  Serial.println("-----------");

  Serial.println("Taking a photo...");

  if (LED_Flash_ON) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(1000);
  }

  for (int i = 0; i <= 3; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      ESP.restart();
      return;
    }
    esp_camera_fb_return(fb);
    delay(200);
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    ESP.restart();
    return;
  }

  if (LED_Flash_ON) digitalWrite(FLASH_LED_PIN, LOW);

  Serial.println("Taking a photo was successful.");
  Serial.println("Connecting to server: " + serverName);

  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");

    String post_data = "--dataMarker\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"ESP32CAMDevice4.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n"; //TODO Change Device Number
    String boundary = "\r\n--dataMarker--\r\n";

    uint32_t totalLen = fb->len + post_data.length() + boundary.length();

    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=dataMarker");
    client.println();
    client.print(post_data);

    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n += 1024) {
      size_t len = (n + 1024 < fbLen) ? 1024 : fbLen - n;
      client.write(fbBuf + n, len);
    }

    client.print(boundary);
    esp_camera_fb_return(fb);

    int timeout = 10000;
    long start = millis();
    boolean state = false;
    Serial.println("Response : ");
    while ((start + timeout) > millis()) {
      Serial.print(".");
      delay(200);
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (AllData.length() == 0) state = true;
          AllData = "";
        } else if (c != '\r') {
          AllData += c;
        }
        if (state) DataBody += c;
        start = millis();
      }
      if (DataBody.length() > 0) break;
    }

    client.stop();
    Serial.println(DataBody);
    Serial.println("-----------");
  } else {
    client.stop();
    Serial.println("Connection to " + serverName + " failed.");
    Serial.println("-----------");
  }
}
//________________________________________________________________________________

//________________________________________________________________________________  Stream Handler
// MJPEG stream handler
static esp_err_t stream_handler(httpd_req_t *req) {
  if (!streaming_enabled) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
  static const char *_STREAM_BOUNDARY = "\r\n--123456789000000000000987654321\r\n";
  static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }

    if (fb->format != PIXFORMAT_JPEG) {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      if (!jpeg_converted) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
      }
    } else {
      _jpg_buf = fb->buf;
      _jpg_buf_len = fb->len;
    }

    res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if (fb->format != PIXFORMAT_JPEG) free(_jpg_buf);
    esp_camera_fb_return(fb);

    if (res != ESP_OK) break;
  }

  return res;
}
//________________________________________________________________________________

//________________________________________________________________________________sendIpAddress()
void sendIpAddress() {
  http.begin("http://157.245.204.46:3001/api/register-ip");
  http.addHeader("Content-Type", "application/json");
  String json = "{\"device_id\": \"device4\", \"ip\": \"" + WiFi.localIP().toString() + "\"}"; //TODO Change Device Number

  Serial.println(json);
  http.POST(json);
  http.end();
}
//________________________________________________________________________________

//________________________________________________________________________________ VOID SETUP()
void setup() {
  // put your setup code here, to run once:

  // Disable brownout detector.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.println();

  // Initialize WiFi using WiFiManager
  WiFiManager wifiManager;
  
  // Uncomment to reset saved credentials (for testing)
  // wifiManager.resetSettings();

if (!wifiManager.autoConnect("ESP32-CAM-AP-DEVICE4")) { //TODO Change Device Number
    Serial.println("Failed to connect and hit timeout");
    ESP.restart();
    delay(1000);
  }

  pinMode(FLASH_LED_PIN, OUTPUT);

  // Setting the ESP32 WiFi to station mode.
  // WiFi.mode(WIFI_STA);
  Serial.println();

  //---------------------------------------- The process of connecting ESP32 CAM with WiFi Hotspot / WiFi Router.
  Serial.println();
  Serial.print("Connecting to : ");
  Serial.println(ssid);
  // WiFi.begin(ssid, password);

  // The process timeout of connecting ESP32 CAM with WiFi Hotspot / WiFi Router is 20 seconds.
  // If within 20 seconds the ESP32 CAM has not been successfully connected to WiFi, the ESP32 CAM will restart.
  // I made this condition because on my ESP32-CAM, there are times when it seems like it can't connect to WiFi, so it needs to be restarted to be able to connect to WiFi.
  int connecting_process_timed_out = 20;  //--> 20 = 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    if (connecting_process_timed_out > 0) connecting_process_timed_out--;
    if (connecting_process_timed_out == 0) {
      Serial.println();
      Serial.print("Failed to connect to ");
      Serial.println(ssid);
      Serial.println("Restarting the ESP32 CAM.");
      delay(1000);
      ESP.restart();
    }
  }
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);

  Serial.println();
  Serial.print("Successfully connected to ");
  Serial.println(ssid);
  //Serial.print("ESP32-CAM IP Address: ");
  //Serial.println(WiFi.localIP());
  //----------------------------------------

  //---------------------------------------- Set the camera ESP32 CAM.
  Serial.println();
  Serial.print("Set the camera ESP32 CAM...");

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

  // init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //--> 0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 8;  //--> 0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    Serial.println();
    Serial.println("Restarting the ESP32 CAM.");
    delay(1000);
    ESP.restart();
  }

  sensor_t *s = esp_camera_sensor_get();

  // Selectable camera resolution details :
  // -UXGA   = 1600 x 1200 pixels
  // -SXGA   = 1280 x 1024 pixels
  // -XGA    = 1024 x 768  pixels
  // -SVGA   = 800 x 600   pixels
  // -VGA    = 640 x 480   pixels
  // -CIF    = 352 x 288   pixels
  // -QVGA   = 320 x 240   pixels
  // -HQVGA  = 240 x 160   pixels
  // -QQVGA  = 160 x 120   pixels
  s->set_framesize(s, FRAMESIZE_XGA);  //--> UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
  s->set_brightness(s, 0);             // range is from -2 to 2
  s->set_contrast(s, -2);              // range is from -2 to 2

  Serial.println();
  Serial.println("Set camera ESP32 CAM successfully.");
  //----------------------------------------
  startCameraServer();
  sendIpAddress();
  Serial.println();
  Serial.print("ESP32-CAM captures and sends photos to the server every 60 seconds.");
}
//________________________________________________________________________________

//________________________________________________________________________________ // Starts HTTP server with stream handler
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &stream_uri);
    Serial.println("Wi-Fi connected: " + WiFi.localIP().toString());
  }
}
//________________________________________________________________________________



//________________________________________________________________________________ VOID LOOP()
void loop() {
  // put your main code here, to run repeatedly:
if (!client.connected()) {
    reconnect();
  }
  mqttClient.loop();
  //---------------------------------------- Timer/Millis to capture and send photos to server every 20 seconds (see Interval variable).
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= Interval) {
    previousMillis = currentMillis;

    sendPhotoToServer();
  }
  //----------------------------------------
}
//________________________________________________________________________________

//________________________________________________________________________________ reconnect mqtt()
// === MQTT Reconnect ===
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ESP32CAM_device4")) { //TODO Change Device Number
      Serial.println("✅ Connected!");
      mqttClient.subscribe("camera/device4/stream"); //TODO Change Device Number
    } else {
      Serial.print("❌ Failed, rc=");
      Serial.print(mqttClient.state());
      delay(2000);
    }
  }
}
//________________________________________________________________________________

//________________________________________________________________________________// === MQTT Callback ===
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) message += (char)payload[i];
  Serial.print("MQTT Message Received: ");
  Serial.println(message);
  Serial.print("Topic: ");
  Serial.println(topic);

  if (strcmp(topic, "camera/device4/stream") == 0) { //TODO Change Device Number
    StaticJsonDocument<64> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) return;

    bool isStreaming = doc["is_streaming"];
    if (isStreaming) {
      streaming_enabled = true;
      Serial.println("Streaming ON");
    } else if (!isStreaming) {
      streaming_enabled = false;
      Serial.println("Streaming OFF");
    }
  }
}
//________________________________________________________________________________
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<