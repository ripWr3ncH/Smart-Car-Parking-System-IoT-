/*******************************************************
 * ESP32-CAM — Real Car Detection with Edge Impulse
 * Uses trained model to detect cars and notify main ESP32
 * Combines Edge Impulse inference with HTTP communication
 *******************************************************/

// Edge Impulse includes
#include <car_detect_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// WiFi and HTTP includes
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_http_server.h"

// ================= Wi-Fi Credentials =================
const char* ssid = "sayeed";
const char* password = "12345678";

// ================= Main ESP32 Endpoint =================
const char* mainEspUrl = "http://192.168.137.243/api/car_detected";  // Change IP to your main ESP32

// ================= Web Server for Camera Stream =================
httpd_handle_t camera_httpd = NULL;

// ================= Camera Pin Setup (AI Thinker) =================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_GPIO_NUM     4  // Built-in LED Flash

// ================= Edge Impulse Constants =================
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

// ================= Global Variables =================
static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;
unsigned long lastTrigger = 0;
const unsigned long cooldown = 3000; // 3 seconds cooldown to prevent spamming
const float confidence_threshold = 0.7; // 70% confidence threshold

// ================= Camera Stream Variables =================
bool streamActive = false;
float lastDetectionConfidence = 0.0;
String lastDetectionTime = "";
unsigned long detectionCount = 0;

// Camera configuration
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,

    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// ================= Function Prototypes =================
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
float getCarConfidenceFromModel();
void notifyMainESP(float confidence);
void setupWiFi();
void setupCamera();
void startCameraWebServer();

// ================= HTML Interface =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Arial,sans-serif;background:#000;color:#fff;padding:10px;text-align:center}
img{width:100%;max-width:640px;border:2px solid #fff;margin:20px auto;display:block}
.confidence{font-size:48px;font-weight:bold;margin:20px;padding:20px;background:#222;border-radius:10px;display:inline-block}
.detection-box{max-width:640px;margin:20px auto;padding:20px;border-radius:10px;font-size:24px;font-weight:bold;background:#4CAF50;color:#000;display:none}
</style>
</head>
<body>
<img src="/stream" alt="Camera">
<div class="confidence" id="conf">--</div>
<div class="detection-box" id="detectionBox">Car Detected</div>
<script>
function updateStats(){
fetch('/stats')
.then(function(response){return response.json();})
.then(function(data){
var conf=data.confidence*100;
document.getElementById('conf').textContent=conf.toFixed(1)+'%';
var box=document.getElementById('detectionBox');
if(data.confidence>0.7){
box.style.display='block';
}else{
box.style.display='none';
}
})
.catch(function(error){
console.log('Error:',error);
});
}
setInterval(updateStats,1000);
updateStats();
</script>
</body>
</html>
)rawliteral";

// ================= Setup Function =================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n🚀 Booting ESP32-CAM with Edge Impulse Car Detection...");

    // Setup Flash LED
    pinMode(FLASH_GPIO_NUM, OUTPUT);
    digitalWrite(FLASH_GPIO_NUM, LOW);

    // Initialize WiFi
    setupWiFi();
    
    // Initialize Camera and Edge Impulse
    setupCamera();
    
    // Start camera web server
    startCameraWebServer();

    Serial.println("🎯 Starting car detection with Edge Impulse model...");
    Serial.printf("📊 Confidence threshold: %.1f%%\n", confidence_threshold * 100);
    Serial.printf("🌐 Camera stream available at: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println("====================================");
}

// ================= Main Loop =================
void loop() {
    Serial.println("====================================");
    Serial.printf("🕒 Time: %lu ms\n", millis());

    // Get car confidence from actual Edge Impulse model
    float carConfidence = getCarConfidenceFromModel();
    
    // ALWAYS update lastDetectionConfidence for real-time display
    // This allows the website to show confidence changes in real-time
    lastDetectionConfidence = carConfidence;
    Serial.printf("📊 Updated lastDetectionConfidence to: %.2f%%\n", lastDetectionConfidence * 100);
    
    if (carConfidence > confidence_threshold && millis() - lastTrigger > cooldown) {
        Serial.printf("🚗 ✅ Car detected! Confidence: %.2f%% → notifying main ESP32...\n", carConfidence * 100);
        // digitalWrite(FLASH_GPIO_NUM, HIGH); // Flash LED ON - DISABLED
        notifyMainESP(carConfidence);
        // digitalWrite(FLASH_GPIO_NUM, LOW);  // Flash LED OFF - DISABLED
        lastTrigger = millis();
        detectionCount++;
        lastDetectionTime = String(millis() / 1000) + "s ago";
    } else if (carConfidence <= confidence_threshold) {
        Serial.printf("⚠️  Confidence too low: %.2f%%. Waiting for next frame...\n", carConfidence * 100);
    } else {
        Serial.println("⏱️  In cooldown period. Waiting...");
    }

    delay(500); // Check every 0.5 seconds for faster response
}

// ================= WiFi Setup =================
void setupWiFi() {
    Serial.printf("📶 Connecting to WiFi: %s\n", ssid);
    WiFi.begin(ssid, password);
    
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
        delay(500);
        Serial.print(".");
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi connected!");
        Serial.print("📍 ESP32-CAM IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n❌ Failed to connect to WiFi! Check SSID or password.");
        while (true) delay(1000);
    }
}

// ================= Camera Setup =================
void setupCamera() {
    Serial.println("📸 Initializing Edge Impulse camera...");
    
    if (ei_camera_init() == false) {
        Serial.println("❌ Failed to initialize Camera!");
        while (true) delay(1000);
    } else {
        Serial.println("✅ Camera initialized successfully!");
    }
}

// ================= Edge Impulse Car Detection =================
float getCarConfidenceFromModel() {
    // Allocate snapshot buffer
    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    if (snapshot_buf == nullptr) {
        Serial.println("❌ Failed to allocate snapshot buffer!");
        return 0.0;
    }

    // Setup signal
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    // Capture image
    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        Serial.println("❌ Failed to capture image");
        free(snapshot_buf);
        return 0.0;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    
    if (err != EI_IMPULSE_OK) {
        Serial.printf("❌ Failed to run classifier (%d)\n", err);
        free(snapshot_buf);
        return 0.0;
    }

    float maxConfidence = 0.0;
    String detectedLabel = "";

    // Print inference timing
    Serial.printf("⏱️  Inference timing - DSP: %d ms, Classification: %d ms\n", 
                  result.timing.dsp, result.timing.classification);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    // Object detection mode
    Serial.println("🔍 Object detection results:");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        Serial.printf("  📦 %s (%.2f%%) [ x: %u, y: %u, w: %u, h: %u ]\n",
                bb.label, bb.value * 100, bb.x, bb.y, bb.width, bb.height);
        
        // Check if this is a car detection with higher confidence
        if (strcmp(bb.label, "car") == 0 && bb.value > maxConfidence) {
            maxConfidence = bb.value;
            detectedLabel = bb.label;
        }
    }
#else
    // Classification mode
    Serial.println("🔍 Classification results:");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        float confidence = result.classification[i].value;
        Serial.printf("  🏷️  %s: %.2f%%\n", ei_classifier_inferencing_categories[i], confidence * 100);
        
        // Check if this is a car classification with higher confidence
        if (strcmp(ei_classifier_inferencing_categories[i], "car") == 0 && confidence > maxConfidence) {
            maxConfidence = confidence;
            detectedLabel = ei_classifier_inferencing_categories[i];
        }
    }
#endif

    // Print anomaly result if available
#if EI_CLASSIFIER_HAS_ANOMALY
    Serial.printf("🚨 Anomaly prediction: %.3f\n", result.anomaly);
#endif

    free(snapshot_buf);
    
    if (maxConfidence > 0) {
        Serial.printf("🎯 Best car detection: %s with %.2f%% confidence\n", detectedLabel.c_str(), maxConfidence * 100);
    }
    
    return maxConfidence;
}

// ================= HTTP Notification =================
void notifyMainESP(float confidence) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("❌ WiFi disconnected! Skipping HTTP request.");
        return;
    }

    Serial.printf("🌐 Sending car detection to main ESP32...\n");
    Serial.printf("📡 URL: %s\n", mainEspUrl);
    
    HTTPClient http;
    http.begin(mainEspUrl);
    http.addHeader("Content-Type", "application/json");
    
    // Create JSON payload with confidence data
    String jsonPayload = "{";
    jsonPayload += "\"car_detected\":true,";
    jsonPayload += "\"confidence\":" + String(confidence, 3) + ",";
    jsonPayload += "\"timestamp\":" + String(millis()) + ",";
    jsonPayload += "\"source\":\"esp32_cam\"";
    jsonPayload += "}";
    
    Serial.println("📨 Payload: " + jsonPayload);
    
    int httpCode = http.POST(jsonPayload);

    if (httpCode > 0) {
        Serial.printf("✅ HTTP Response: %d\n", httpCode);
        String response = http.getString();
        if (response.length() > 0) {
            Serial.println("📬 Response: " + response);
        }
    } else {
        Serial.printf("❌ HTTP Error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
}

// ================= Edge Impulse Camera Functions =================
bool ei_camera_init(void) {
    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
#endif

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("❌ Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        Serial.println("❌ Camera deinit failed");
        return;
    }
    is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        Serial.println("❌ Camera is not initialized");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("❌ Camera capture failed");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);

    if (!converted) {
        Serial.println("❌ Image conversion failed");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}

// ================= Camera Web Server Functions =================
static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_sendstr(req, index_html);
}

static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char part_buf[64];

    res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    if(res != ESP_OK){
        return res;
    }

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("❌ Camera capture failed in stream");
            res = ESP_FAIL;
            break;
        }

        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;

        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, 
                "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, "\r\n--frame\r\n", 9);
        }
        
        esp_camera_fb_return(fb);
        
        if(res != ESP_OK){
            Serial.println("❌ Stream send failed");
            break;
        }
        delay(30); // Control frame rate
    }
    return res;
}

static esp_err_t stats_handler(httpd_req_t *req) {
    String json = "{\"confidence\":" + String(lastDetectionConfidence, 3) + "}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_sendstr(req, json.c_str());
}

void startCameraWebServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t stats_uri = {
        .uri       = "/stats",
        .method    = HTTP_GET,
        .handler   = stats_handler,
        .user_ctx  = NULL
    };

    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &stream_uri);
        httpd_register_uri_handler(camera_httpd, &stats_uri);
        Serial.println("✅ Camera web server started successfully!");
    } else {
        Serial.println("❌ Failed to start camera web server!");
    }
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
