#include "FirebaseESP32.h"
#include <WiFi.h>
#include "esp_task_wdt.h"
#include <esp_now.h>
#define FIREBASE_USE_PSRAM

#define FIREBASE_HOST "https://nodemcu-a4907-default-rtdb.asia-southeast1.firebasedatabase.app" 
#define FIREBASE_AUTH "frB74idkfdayCS44bsuY0a3WLY59PZtJrxvTUMnD"

#define WIFI_SSID "SS A20 FREE"
#define WIFI_PASSWORD "19781902Cfc"

#define red_pinout 12
#define yellow_pinout 14
#define green_pinout 27

// MAC Address
uint8_t broadcastAddress[] = {0xD8, 0xBF, 0xC0, 0xFA, 0x7A, 0x8E};

// Khai báo task
TaskHandle_t Task1;
TaskHandle_t Task2;

// Semaphore để share data
SemaphoreHandle_t baton;

// Must match the receiver structure
typedef struct struct_message {
  int b;
  float c;
  bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

FirebaseData fbdo;

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

void setup()
{
  Serial.begin(115200);
  pinMode(red_pinout,OUTPUT);
  pinMode(yellow_pinout,OUTPUT);
  pinMode(green_pinout,OUTPUT);

 // Disable Watch dog timer debugger (vì nó sẽ reboot nếu mạng gặp trục trặc)
  disableCore0WDT();
  disableCore1WDT();
  disableLoopWDT();

  WiFi.mode(WIFI_AP_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  // Serial.print("Wi-Fi Channel: ");
  // Serial.println(WiFi.channel());

  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Serial.print("Connecting to WiFi");

  // while (WiFi.status() != WL_CONNECTED){
  //   Serial.print(".");
  //   delay(300);
  // }
  // Serial.println();
  // Serial.print("Connected with IP: ");
  // Serial.println(WiFi.localIP());
  // Serial.println();
  
  // Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  // Firebase.reconnectWiFi(true);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  // peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

 // SEMAPHORE để share data
  baton = xSemaphoreCreateMutex();

 

  //TASK WIFI
  xTaskCreatePinnedToCore(
                  ESPNOWSender,   /* Task function. */
                  "Task1",     /* name of task. */
                  10000,       /* Stack size of task */
                  NULL,        /* parameter of the task */
                  configMAX_PRIORITIES,           /* priority of the task */
                  &Task1,      /* Task handle to keep track of created task */
                  1);          /* pin task to core 0 */                  
  delay(500); 
  
  //TASK 1
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

void count(int max_count){
  for(int i=1;i<=max_count;i++){
    delay(1000);
    Serial.println(i);
    Firebase.RTDB.setInt(&fbdo, "traffic/count", i);
  }
}

void red(){
  // Firebase.RTDB.setString(&fbdo, "traffic/light", "red");
  digitalWrite(red_pinout,HIGH);
  count(10);
  digitalWrite(red_pinout,LOW);
}

void yellow(){
  // Firebase.RTDB.setString(&fbdo, "traffic/light", "yellow");
  digitalWrite(yellow_pinout,HIGH);
  count(3);
  digitalWrite(yellow_pinout,LOW);
}

void green(){
  // Firebase.RTDB.setString(&fbdo, "traffic/light", "green");
  digitalWrite(green_pinout,HIGH);
  count(7);
  digitalWrite(green_pinout,LOW);
}

// core 1 task1 for main function
void MainTask( void * pvParameters ){
  for(;;){
    long start =millis();
    
    xSemaphoreTake(baton,portMAX_DELAY);
    xSemaphoreGive(baton);

    green();
    yellow();
    red();

    Serial.println("MAINTASK Speed: " + String(millis()-start));
  }
}

// ESPNOWSender
void ESPNOWSender( void * pvParameters ){
  for(;;){
    long start =millis();
    
    xSemaphoreTake(baton,portMAX_DELAY);
    xSemaphoreGive(baton);
    
    // Set values to send
    myData.b = random(1,20);
    myData.c = 1.2;
    myData.d = false;
    
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

    Serial.println("ESPNOW Sender Speed: " + String(millis()-start));
    delay(100);
  }
}
// DONT USE THIS
void loop(){
    vTaskDelete(NULL);
    // NOTHING HERE
}


