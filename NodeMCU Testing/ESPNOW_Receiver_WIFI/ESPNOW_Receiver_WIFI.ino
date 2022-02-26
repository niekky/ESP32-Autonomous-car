#include <esp_now.h>
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif
#include "FirebaseESP32.h"
#include "esp_task_wdt.h"
#include <esp_now.h>
#define FIREBASE_USE_PSRAM

#define FIREBASE_HOST "https://nodemcu-a4907-default-rtdb.asia-southeast1.firebasedatabase.app" 
#define FIREBASE_AUTH "frB74idkfdayCS44bsuY0a3WLY59PZtJrxvTUMnD"

#define WIFI_SSID "SS A20 FREE"
#define WIFI_PASSWORD "19781902Cfc"

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int b;
    float c;
    bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Khai báo task
TaskHandle_t Task1;
TaskHandle_t Task2;

// Semaphore để share data
SemaphoreHandle_t baton;

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Int: ");
  Serial.println(myData.b);
  Serial.print("Float: ");
  Serial.println(myData.c);
  Serial.print("Bool: ");
  Serial.println(myData.d);
  Serial.println();
}
 
void setup() {
  // Initialize Serial Monitor
  pinMode(12,OUTPUT);
  Serial.begin(115200);
  
  disableCore0WDT();
  disableCore1WDT();
  disableLoopWDT();

  WiFi.mode(WIFI_AP_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  xTaskCreatePinnedToCore(
                  MainTask,   /* Task function. */
                  "Task2",     /* name of task. */
                  10000,       /* Stack size of task */
                  NULL,        /* parameter of the task */
                  1,           /* priority of the task */
                  &Task2,      /* Task handle to keep track of created task */
                  1);          /* pin task to core 1 */
  delay(500); 

}

// core 1 task1 for main function
void MainTask( void * pvParameters ){
  for(;;){
    long start =millis();
    
    // xSemaphoreTake(baton,portMAX_DELAY);
    // xSemaphoreGive(baton);

    digitalWrite(12,1);
    delay(500);
    digitalWrite(12,0);
    delay(500);
    Serial.println("MAINTASK Speed: " + String(millis()-start));
  }
}
// DONT USE THIS
void loop(){
    vTaskDelete(NULL);
    // NOTHING HERE
}
