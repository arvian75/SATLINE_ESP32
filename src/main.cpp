#include <WiFi.h>
#include "esp_camera.h"
#include <Arduino.h>
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include <SPIFFS.h>
#include <FS.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include "NTPClient/NTPClient.h"
#include <WifiUDP.h>
#include "devices.h"
#include "secrets.h"

// Photo File Name to save in SPIFFS
#define FILE_PHOTO "/photo.jpg"

#define HIGH 1
#define LOW 0

//Define Firebase Data objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig fbConfig;
FirebaseAuth fbAuth;
FirebaseData fbdoStream;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

bool buzzerState = false, taskCompleted = false, readyToSendDoorState = false, doneTakePicture = false;
int doorState = 0, beforeDoorState = 1, lockState = 0;
String formattedDate = "", timeStamp = "", dayStamp = "";

bool checkPhoto(fs::FS &fs);
void deleteFile(fs::FS &fs, const char * path);
void capturePhotoSaveSpiffs(int flashOnOff);
void initWiFi();
void initTime();
void initSPIFFS();
void initCamera();
void onFirebaseStream(FirebaseStream data);
void Firebase_BeginStream(const String& streamPath);
void Firebase_EndStream(const String& streamPath);
void Firebase_Init(const String& streamPath);

void taskTakePhoto(void *pvParameter ) {
  while (true) {
    if (doorState == HIGH || lockState == 1) {
      readyToSendDoorState = false;
      Firebase_EndStream("Receive");
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      capturePhotoSaveSpiffs(HIGH);
      taskCompleted = false;

    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
    while (!taskCompleted && doneTakePicture && Firebase.ready()) {
      String path = "/images/" + dayStamp + "/" + dayStamp + " " + timeStamp + ".jpg";
      Serial.print("Uploading picture... ");

      if (Firebase.Storage.upload(&fbdo, STORAGE_BUCKET_ID, FILE_PHOTO, mem_storage_type_flash, path, "image/jpeg")) {
        Serial.println("Upload Success");
        taskCompleted = true;
        readyToSendDoorState = true;
        buzzerState = true;
        doneTakePicture = false;
        Firebase_BeginStream("Receive");
        deleteFile(SPIFFS, FILE_PHOTO);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
      else {
        Serial.println(fbdo.errorReason());
        taskCompleted = false;
      }
      vTaskDelay(4000 / portTICK_PERIOD_MS);
    }

    if (doneTakePicture == true) {
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void taskBuzzer(void *pvParameter) {
  while (true) {
    if (buzzerState == true) {
      digitalWrite(BUZZER, LOW);
      vTaskDelay(250 / portTICK_PERIOD_MS);
      if(doorState == LOW || lockState == 0) {
        buzzerState = false;
        digitalWrite(BUZZER, HIGH);
      }
    } else {
      digitalWrite(BUZZER, HIGH);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void taskSendDoorState(void *pvParameter) {
  while (true) {
    if (doorState != beforeDoorState && readyToSendDoorState == true) {
      beforeDoorState = doorState;
      if (doorState == HIGH) {
        Firebase.RTDB.setString(&fbdo, "/Send/DoorState", "Open");
        Serial.println("Door Open Send");
      } else if (doorState == LOW) {
        Firebase.RTDB.setString(&fbdo, "/Send/DoorState", "Close");
        Serial.println("Door Close Send");
      }
    } 
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  ledcSetup(LEDC_CHANNEL_0, 5000, 8);
  ledcAttachPin(LED_FLASH, LEDC_CHANNEL_0);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);
  pinMode(REED_SWITCH, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  initWiFi();
  initTime();
  initSPIFFS();
  delay(1000);
  SPIFFS.format();
  
  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  initCamera();
  // Test Capture Camera
  capturePhotoSaveSpiffs(LOW);
  deleteFile(SPIFFS, FILE_PHOTO);
  doneTakePicture = false;

  Firebase_Init("Receive");

  xTaskCreatePinnedToCore(taskTakePhoto, "taskTakePhoto", 10000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskBuzzer, "taskBuzzer", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskSendDoorState, "taskSendDoorState", 10000, NULL, 1, NULL, 1);
  digitalWrite(PIN_LED, HIGH);
  delay(1000);
}

void loop() {
  doorState = digitalRead(REED_SWITCH);
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

// Check if photo capture was successful
bool checkPhoto( fs::FS &fs ) {
  File f_pic = fs.open( FILE_PHOTO );
  unsigned int pic_sz = f_pic.size();
  return ( pic_sz > 100 );
}

void deleteFile(fs::FS &fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("file deleted");
  } else {
    Serial.println("delete failed");
  }
}

// Capture Photo and Save it to SPIFFS
void capturePhotoSaveSpiffs(int flashOnOff) {
  camera_fb_t *fb = NULL; // pointer
  bool ok = 0; // Boolean indicating if the picture has been taken correctly
  if (flashOnOff == HIGH) {
    ledcWrite(0, 50);
  }

  do {
    // Take a photo with the camera
    Serial.println("Taking a photo...");
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      ESP.restart();
    }
    // Photo file name
    Serial.printf("Picture file name: %s\n", FILE_PHOTO);
    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);
    // Insert the data in the photo file
    if (!file) {
      Serial.println("Failed to open file in writing mode");
    }
    else {
      file.write(fb->buf, fb->len); // payload (image), payload length
      Serial.print("The picture has been saved in ");
      Serial.print(FILE_PHOTO);
      Serial.print(" - Size: ");
      Serial.print(file.size());
      Serial.println(" bytes");
    }
    formattedDate = timeClient.getFormattedDate();
    int timeIndex = formattedDate.indexOf("T");
    dayStamp = formattedDate.substring(0, timeIndex);
    timeStamp = formattedDate.substring(timeIndex + 1, formattedDate.length() - 1);

    // Close the file
    file.close();
    esp_camera_fb_return(fb);

    // check if file has been correctly saved in SPIFFS
    ok = checkPhoto(SPIFFS);
  } while ( !ok );
  
  if (flashOnOff == HIGH) {
    ledcWrite(0, 0);
  }
  doneTakePicture = true;
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(500);
    ESP.restart();
  }  
  Serial.print("System connected with IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("RSSI: %d\n", WiFi.RSSI());
}

void initTime() {
  timeClient.begin();
  timeClient.setTimeOffset(25200);
  while(!timeClient.update()) {
    Serial.println("Waiting for time");
    timeClient.forceUpdate();
  }
}

void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }
}

void initCamera() {
 // OV2640 camera module
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
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  } 
  Serial.printf("Camera init success with frame size: %dx%d", config.frame_size, config.jpeg_quality);
}

void onFirebaseStream(FirebaseStream data) {
  Serial.printf("onFirebaseStream: %s %s %s %s\n", data.streamPath().c_str(),
                data.dataPath().c_str(), data.dataType().c_str(),
                data.stringData().c_str());
  if (data.dataType() == "json") {
    if(data.stringData() == "{\"LockState\":1}") {
      lockState = 1;
    } else {
      lockState = 0;
    }
  }
}

void Firebase_BeginStream(const String& streamPath) {
  String path = streamPath;
  if (Firebase.RTDB.beginStream(&fbdoStream, path.c_str()))
  {
    Serial.println("Firebase stream on "+ path);
    Firebase.RTDB.setStreamCallback(&fbdoStream, onFirebaseStream, 0);
  }
  else 
    Serial.println("Firebase stream failed: "+fbdoStream.errorReason());
}

void Firebase_EndStream(const String& streamPath) {
  if(Firebase.RTDB.endStream(&fbdoStream))
    Serial.println("Firebase stream end");
  else {
    Serial.println("Firebase stream end failed: "+fbdoStream.errorReason());
    Serial.println("Trying to end stream again");
    Firebase_EndStream(streamPath);
  }
}

void Firebase_Init(const String& streamPath) {
  fbConfig.host = FIREBASE_HOST;
  fbConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
  fbConfig.token_status_callback = tokenStatusCallback;
  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);

  while (!Firebase.ready())
  {
    Serial.println("Connecting to firebase...");
    delay(1000);
  }
  Serial.println("Connected to firebase");
  Firebase_BeginStream(streamPath);
}