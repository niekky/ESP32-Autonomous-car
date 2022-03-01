
// Khai báo task
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
TaskHandle_t Task5;
// Semaphore để share data
SemaphoreHandle_t baton;

#include "FirebaseESP32.h"
#include <WiFi.h>
#include "esp_task_wdt.h"
#include <esp_now.h>
#include <Wire.h>

#define FIREBASE_USE_PSRAM

#define FIREBASE_HOST "https://nodemcu-a4907-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "frB74idkfdayCS44bsuY0a3WLY59PZtJrxvTUMnD"

#define WIFI_SSID "SS A20 FREE"
#define WIFI_PASSWORD "19781902Cfc"

#define red_pinout_1 5
#define yellow_pinout_1 18
#define green_pinout_1 12

#define red_pinout_2 14
#define yellow_pinout_2 27
#define green_pinout_2 26

#define c_red_pinout 25
#define c_yellow_pinout 33
#define c_green_pinout 32   

#define trigPin 17
#define echoPin 16

#define trigPin2 15
#define echoPin2 13

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message
{
  byte traffic_state;
} struct_message;

// Create a struct_message called myData
struct_message myData;

uint8_t broadcastAddress[] = {0xA4, 0xE5, 0x7C, 0xD6, 0x74, 0xFC};

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

long duration;
float distanceCm;
long duration2;
float distanceCm2;

int x = 0;
byte traffic_state;

// FirebaseData fbdo;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  pinMode(red_pinout_1, OUTPUT);
  pinMode(yellow_pinout_1, OUTPUT);
  pinMode(green_pinout_1, OUTPUT);
  pinMode(red_pinout_2, OUTPUT);
  pinMode(yellow_pinout_2, OUTPUT);
  pinMode(green_pinout_2, OUTPUT);
  pinMode(c_red_pinout, OUTPUT);
  pinMode(c_yellow_pinout, OUTPUT);
  pinMode(c_green_pinout, OUTPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  pinMode(trigPin2,OUTPUT);
  pinMode(echoPin2,INPUT);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  baton = xSemaphoreCreateMutex();

  // Disable Watch dog timer debugger (vì nó sẽ reboot nếu mạng gặp trục trặc)
  disableCore0WDT();
  disableCore1WDT();
  disableLoopWDT();

  digitalWrite(c_red_pinout,HIGH);
  delay(1000);
  digitalWrite(c_red_pinout,LOW);
  digitalWrite(c_yellow_pinout,HIGH);
  delay(1000);
  digitalWrite(c_yellow_pinout,LOW);
  digitalWrite(c_green_pinout,HIGH);

  // // TASK WIFI
  // xTaskCreatePinnedToCore(
  //     SensorTask1, /* Task function. */
  //     "Task1",   /* name of task. */
  //     10000,     /* Stack size of task */
  //     NULL,      /* parameter of the task */
  //     1,         /* priority of the task */
  //     &Task1,    /* Task handle to keep track of created task */
  //     1);        /* pin task to core 0 */
  // delay(500);

  // // TASK 1
  // xTaskCreatePinnedToCore(
  //     TrafficTask, /* Task function. */
  //     "Task2",     /* name of task. */
  //     1000,        /* Stack size of task */
  //     NULL,        /* parameter of the task */
  //     1,           /* priority of the task */
  //     &Task2,      /* Task handle to keep track of created task */
  //     1);          /* pin task to core 1 */
  // delay(500);

  xTaskCreatePinnedToCore(
      ESPNowTask, /* Task function. */
      "Task3",    /* name of task. */
      10000,      /* Stack size of task */
      NULL,       /* parameter of the task */
      1,          /* priority of the task */
      &Task3,     /* Task handle to keep track of created task */
      0);         /* pin task to core 0 */
  delay(500);

  xTaskCreatePinnedToCore(
      STraffic1Task, /* Task function. */
      "Task4",    /* name of task. */
      1000,      /* Stack size of task */
      NULL,       /* parameter of the task */
      1,          /* priority of the task */
      &Task4,     /* Task handle to keep track of created task */
      1);         /* pin task to core 0 */
  delay(500);

  xTaskCreatePinnedToCore(
      STraffic2Task, /* Task function. */
      "Task5",    /* name of task. */
      1000,      /* Stack size of task */
      NULL,       /* parameter of the task */
      1,          /* priority of the task */
      &Task5,     /* Task handle to keep track of created task */
      1);         /* pin task to core 0 */
  delay(500);
}

void normalLED(){
  // red1 ON, green2 ON
    traffic_state = 0;
    digitalWrite(red_pinout_1, HIGH);
    digitalWrite(yellow_pinout_1, LOW);
    digitalWrite(green_pinout_1, LOW);
    digitalWrite(red_pinout_2, LOW);
    digitalWrite(yellow_pinout_2, LOW);
    digitalWrite(green_pinout_2, HIGH);
    Serial.println("                  TrafficTASK: traffic_state=" + String(traffic_state));
    delay(4000);

    // red1 ON, yellow2 ON
    digitalWrite(red_pinout_1, HIGH);
    digitalWrite(yellow_pinout_1, LOW);
    digitalWrite(green_pinout_1, LOW);
    digitalWrite(red_pinout_2, LOW);
    digitalWrite(yellow_pinout_2, HIGH);
    digitalWrite(green_pinout_2, LOW);
    Serial.println("                  TrafficTASK: traffic_state=" + String(traffic_state));
    delay(1000);

    // green1 ON, red2 ON
    traffic_state = 1;
    digitalWrite(red_pinout_1, LOW);
    digitalWrite(yellow_pinout_1, LOW);
    digitalWrite(green_pinout_1, HIGH);
    digitalWrite(red_pinout_2, HIGH);
    digitalWrite(yellow_pinout_2, LOW);
    digitalWrite(green_pinout_2, LOW);
    Serial.println("                  TrafficTASK: traffic_state=" + String(traffic_state));
    delay(4000);

    // yellow1 ON, red2 ON
    digitalWrite(red_pinout_1, LOW);
    digitalWrite(yellow_pinout_1, HIGH);
    digitalWrite(green_pinout_1, LOW);
    digitalWrite(red_pinout_2, HIGH);
    digitalWrite(yellow_pinout_2, LOW);
    digitalWrite(green_pinout_2, LOW);
    Serial.println("                  TrafficTASK: traffic_state=" + String(traffic_state));
    delay(1000);

}

void specialLED(){
  traffic_state=1;
  digitalWrite(red_pinout_1,1);
  digitalWrite(red_pinout_2,1);
  digitalWrite(yellow_pinout_1,0);
  digitalWrite(yellow_pinout_2,0);
  digitalWrite(green_pinout_1,0);
  digitalWrite(green_pinout_2,0);
  delay(7000);
  digitalWrite(red_pinout_1,1);
  digitalWrite(red_pinout_2,1);
  digitalWrite(yellow_pinout_1,1);
  digitalWrite(yellow_pinout_2,1);
  digitalWrite(green_pinout_1,0);
  digitalWrite(green_pinout_2,0);
  delay(3000);
  traffic_state=0;
  digitalWrite(red_pinout_1,0);
  digitalWrite(red_pinout_2,0);
  digitalWrite(yellow_pinout_1,0);
  digitalWrite(yellow_pinout_2,0);
  digitalWrite(green_pinout_1,1);
  digitalWrite(green_pinout_2,1);
  delay(10000);
  digitalWrite(red_pinout_1,0);
  digitalWrite(red_pinout_2,0);
  digitalWrite(yellow_pinout_1,0);
  digitalWrite(yellow_pinout_2,0);
  digitalWrite(green_pinout_1,1);
  digitalWrite(green_pinout_2,1);
  delay(500);
  digitalWrite(red_pinout_1,0);
  digitalWrite(red_pinout_2,0);
  digitalWrite(yellow_pinout_1,0);
  digitalWrite(yellow_pinout_2,0);
  digitalWrite(green_pinout_1,0);
  digitalWrite(green_pinout_2,0);
  delay(500);
  digitalWrite(red_pinout_1,0);
  digitalWrite(red_pinout_2,0);
  digitalWrite(yellow_pinout_1,0);
  digitalWrite(yellow_pinout_2,0);
  digitalWrite(green_pinout_1,0);
  digitalWrite(green_pinout_2,0);
  delay(500);
  digitalWrite(red_pinout_1,0);
  digitalWrite(red_pinout_2,0);
  digitalWrite(yellow_pinout_1,0);
  digitalWrite(yellow_pinout_2,0);
  digitalWrite(green_pinout_1,1);
  digitalWrite(green_pinout_2,1);
  delay(500);
  digitalWrite(red_pinout_1,0);
  digitalWrite(red_pinout_2,0);
  digitalWrite(yellow_pinout_1,0);
  digitalWrite(yellow_pinout_2,0);
  digitalWrite(green_pinout_1,0);
  digitalWrite(green_pinout_2,0);
  delay(500);
  digitalWrite(red_pinout_1,0);
  digitalWrite(red_pinout_2,0);
  digitalWrite(yellow_pinout_1,0);
  digitalWrite(yellow_pinout_2,0);
  digitalWrite(green_pinout_1,1);
  digitalWrite(green_pinout_2,1);
  delay(500);
}

void UltrasonicRead(){
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  digitalWrite(trigPin2,LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  digitalWrite(trigPin2,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  digitalWrite(trigPin2, LOW);

  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  duration2 = pulseIn(echoPin2,HIGH);
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  distanceCm2 = duration2 * SOUND_SPEED/2;
  // Prints the distance in the Serial Monitor
  Serial.println("                            Distance (cm): "+String(distanceCm));
  Serial.println("                            Distance2 (cm): "+String(distanceCm2));
  delay(200);
}

void count(int max_count)
{
  for (int i = 1; i <= max_count; i++)
  {
    delay(1000);
  }
}

// core 1 task1 for main function
void SensorTask1(void *pvParameters)
{
  for (;;)
  {
    long start = millis();

    xSemaphoreTake(baton, portMAX_DELAY);
    xSemaphoreGive(baton);

    UltrasonicRead();
    Serial.println("LidarTask Speed: " + String(millis() - start));
  }
}

// core 1 task1 for main function
void ESPNowTask(void *pvParameters)
{
  for (;;)
  {
    // long start =millis();

    xSemaphoreTake(baton, portMAX_DELAY);
    xSemaphoreGive(baton);

    // Set values to send
    myData.traffic_state=traffic_state;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    if (result == ESP_OK)
    {
      Serial.println("ESPNOW: Sent with success");
    }
    else
    {
      Serial.println("ESPNOW: Error sending the data");
    }
    delay(100);

    // Serial.println("ESPNOWTask Speed: " + String(millis()-start));
  }
}

// core 1 task1 for main function
void TrafficTask(void *pvParameters)
{
  for (;;)
  {
    // long start =millis();

    xSemaphoreTake(baton, portMAX_DELAY);
    xSemaphoreGive(baton);
    specialLED();
    
    // Serial.println("TrafficTask Speed: " + String(millis()-start));
  }
}

// core 1 task1 for main function
void STraffic1Task(void *pvParameters)
{
  for (;;)
  {
    // long start =millis();

    xSemaphoreTake(baton, portMAX_DELAY);
    xSemaphoreGive(baton);
    traffic_state=1;
    digitalWrite(red_pinout_1,0);
    digitalWrite(yellow_pinout_1,1);
    digitalWrite(green_pinout_1,0);
    delay(2000);
    digitalWrite(red_pinout_1,1);
    digitalWrite(yellow_pinout_1,0);
    digitalWrite(green_pinout_1,0);
    delay(7000);
    digitalWrite(red_pinout_1,1);
    digitalWrite(yellow_pinout_1,1);
    digitalWrite(green_pinout_1,0);
    delay(3000);
    traffic_state=0;
    digitalWrite(red_pinout_1,0);
    digitalWrite(yellow_pinout_1,0);
    digitalWrite(green_pinout_1,1);
    delay(10000);
    // Blink every 500ms in 2s
    digitalWrite(red_pinout_1,0);
    digitalWrite(yellow_pinout_1,0);
    green_blink(green_pinout_1);
    // digitalWrite(red_pinout_1,0);
    // digitalWrite(yellow_pinout_1,0);
    // digitalWrite(green_pinout_1,1);
    // delay(500);
    // digitalWrite(red_pinout_1,0);
    // digitalWrite(yellow_pinout_1,0);
    // digitalWrite(green_pinout_1,0);
    // delay(500);
    // digitalWrite(red_pinout_1,0);
    // digitalWrite(yellow_pinout_1,0);
    // digitalWrite(green_pinout_1,1);
    // delay(500);
    // digitalWrite(red_pinout_1,0);
    // digitalWrite(yellow_pinout_1,0);
    // digitalWrite(green_pinout_1,0);
    // delay(500);
    traffic_state=0;
    digitalWrite(red_pinout_1,0);
    digitalWrite(yellow_pinout_1,1);
    digitalWrite(green_pinout_1,0);
    delay(1000);
    // Serial.println("TrafficTask Speed: " + String(millis()-start));
  }
}

void green_blink(int pinout){
  int g_initialize=0;
  for (int i=0;i<10;i++){
    if (g_initialize==0){
      g_initialize=1;
    } else g_initialize=0;
    digitalWrite(pinout,g_initialize);
    delay(200);
  }
}

// core 1 task1 for main function
void STraffic2Task(void *pvParameters)
{
  for (;;)
  {
    // long start =millis();
    digitalWrite(red_pinout_2,0);
    digitalWrite(yellow_pinout_2,0);
    digitalWrite(green_pinout_2,1);
    delay(10000);
    // Blink 10 times in 2s
    digitalWrite(red_pinout_2,0);
    digitalWrite(yellow_pinout_2,0);
    green_blink(green_pinout_2);
    // digitalWrite(red_pinout_2,0);
    // digitalWrite(yellow_pinout_2,0);
    // digitalWrite(green_pinout_2,0);
    // delay(500);
    // digitalWrite(red_pinout_2,0);
    // digitalWrite(yellow_pinout_2,0);
    // digitalWrite(green_pinout_2,1);
    // delay(500);
    // digitalWrite(red_pinout_2,0);
    // digitalWrite(yellow_pinout_2,0);
    // digitalWrite(green_pinout_2,0);
    // delay(500);
    // digitalWrite(red_pinout_2,0);
    // digitalWrite(yellow_pinout_2,0);
    // digitalWrite(green_pinout_2,1);
    // delay(500);
    digitalWrite(red_pinout_2,0);
    digitalWrite(yellow_pinout_2,1);
    digitalWrite(green_pinout_2,0);
    delay(3000);
    digitalWrite(red_pinout_2,1);
    digitalWrite(yellow_pinout_2,0);
    digitalWrite(green_pinout_2,0);
    delay(7000);
    digitalWrite(red_pinout_2,1);
    digitalWrite(yellow_pinout_2,1);
    digitalWrite(green_pinout_2,0);
    delay(3000);

    // Serial.println("TrafficTask Speed: " + String(millis()-start));
  }
}


void loop()
{
  vTaskDelete(NULL);
}
