
// Khai báo task
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

// Semaphore để share data
SemaphoreHandle_t baton;

#include "FirebaseESP32.h"
#include <WiFi.h>
#include "esp_task_wdt.h"
#include <esp_now.h>

#define FIREBASE_USE_PSRAM

#define FIREBASE_HOST "https://nodemcu-a4907-default-rtdb.asia-southeast1.firebasedatabase.app" 
#define FIREBASE_AUTH "frB74idkfdayCS44bsuY0a3WLY59PZtJrxvTUMnD"

#define WIFI_SSID "SS A20 FREE"
#define WIFI_PASSWORD "19781902Cfc"

#define red_pinout_1 12
#define yellow_pinout_1 14
#define green_pinout_1 27

#define red_pinout_2 26
#define yellow_pinout_2 25
#define green_pinout_2 33

#define c_red_pinout 32
#define c_yellow_pinout 18
#define c_green_pinout 5

#include <Wire.h>

// Structure example to send data
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

unsigned char ok_flag;
unsigned char fail_flag;

unsigned short lenth_val = 0;
unsigned char i2c_rx_buf[16];
unsigned char dirsend_flag=0;
unsigned long previous_time1=0;
unsigned long previous_time2=0;
unsigned long previous_time3=0;
unsigned long previous_time4=0;
unsigned long previous_time5=0;
unsigned long previous_time6=0;

uint8_t broadcastAddress[] = {0xD8, 0xBF, 0xC0, 0xFA, 0x7A, 0x8E};

int x=0;
byte traffic_state;

// FirebaseData fbdo;

void setup()
{
  Wire.begin();
  Serial.begin(9600,SERIAL_8N1);
  pinMode(red_pinout_1,OUTPUT);
  pinMode(yellow_pinout_1,OUTPUT);
  pinMode(green_pinout_1,OUTPUT);
  pinMode(red_pinout_2,OUTPUT);
  pinMode(yellow_pinout_2,OUTPUT);
  pinMode(green_pinout_2,OUTPUT);
  pinMode(c_red_pinout,OUTPUT);
  pinMode(c_yellow_pinout,OUTPUT);
  pinMode(c_green_pinout,OUTPUT);


  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

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
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  baton = xSemaphoreCreateMutex();

  // Disable Watch dog timer debugger (vì nó sẽ reboot nếu mạng gặp trục trặc)
  disableCore0WDT();
  disableCore1WDT();
  disableLoopWDT();

  //TASK WIFI
  xTaskCreatePinnedToCore(
                  LidarTask,   /* Task function. */
                  "Task1",     /* name of task. */
                  10000,       /* Stack size of task */
                  NULL,        /* parameter of the task */
                  1,           /* priority of the task */
                  &Task1,      /* Task handle to keep track of created task */
                  1);          /* pin task to core 0 */                  
  delay(500); 
  
  //TASK 1
  xTaskCreatePinnedToCore(
                  TrafficTask,   /* Task function. */
                  "Task2",     /* name of task. */
                  1000,       /* Stack size of task */
                  NULL,        /* parameter of the task */
                  1,           /* priority of the task */
                  &Task2,      /* Task handle to keep track of created task */
                  1);          /* pin task to core 1 */
  delay(500); 

  xTaskCreatePinnedToCore(
                  ESPNowTask,   /* Task function. */
                  "Task3",     /* name of task. */
                  10000,       /* Stack size of task */
                  NULL,        /* parameter of the task */
                  0,           /* priority of the task */
                  &Task3,      /* Task handle to keep track of created task */
                  0);          /* pin task to core 0 */
  delay(500); 
}

//SDA 21 SCL 22 ADDRESS 0x52 
void SensorRead(unsigned char addr,unsigned char* datbuf,unsigned char cnt) 
{
  unsigned short result=0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(82); // transmit to device #82 (0x52)
  // the address specified in the datasheet is 164 (0xa4)
  // but i2c adressing uses the high 7 bits so it's 82
  Wire.write(byte(addr));      // sets distance data address (addr)
  Wire.endTransmission();      // stop transmitting
  // step 2: wait for readings to happen
  delay(1);                   // datasheet suggests at least 30uS
  // step 3: request reading from sensor
  Wire.requestFrom(82, cnt);    // request cnt bytes from slave device #82 (0x52)
  // step 5: receive reading from sensor
  if (cnt <= Wire.available()) { // if two bytes were received
    *datbuf++ = Wire.read();  // receive high byte (overwrites previous reading)
    *datbuf++ = Wire.read(); // receive low byte as lower 8 bits
  }
}

int ReadDistance(){
    SensorRead(0x00,i2c_rx_buf,2);
    lenth_val=i2c_rx_buf[0];
    lenth_val=lenth_val<<8;
    lenth_val|=i2c_rx_buf[1];
    delay(300); 
    return lenth_val;
}

void count(int max_count){
  for(int i=1;i<=max_count;i++){
    delay(1000);
    Serial.println(i);
    // Firebase.RTDB.setInt(&fbdo, "traffic/count", i);
  }
}

// void red(){
//   // Firebase.RTDB.setString(&fbdo, "traffic/light", "red");
//   traffic_state=0;
//   digitalWrite(red_pinout_1,HIGH);
//   count(10);
//   digitalWrite(red_pinout,LOW);
// }

// void yellow(){
//   // Firebase.RTDB.setString(&fbdo, "traffic/light", "yellow");
//   traffic_state=1;
//   digitalWrite(yellow_pinout,HIGH);
//   count(3);
//   digitalWrite(yellow_pinout,LOW);
// }

// void green(){
//   // Firebase.RTDB.setString(&fbdo, "traffic/light", "green");
//   traffic_state=2;
//   digitalWrite(green_pinout,HIGH);
//   count(7);
//   digitalWrite(green_pinout,LOW);
// }

// core 1 task1 for main function
void LidarTask( void * pvParameters ){
  for(;;){
    long start =millis();
    
    xSemaphoreTake(baton,portMAX_DELAY);
    xSemaphoreGive(baton);

    x=ReadDistance();
    Serial.print(x);
    Serial.println(" mm");

    Serial.println("LidarTask Speed: " + String(millis()-start));
  }
}


// core 1 task1 for main function
void ESPNowTask( void * pvParameters ){
  for(;;){
    // long start =millis();
    
    xSemaphoreTake(baton,portMAX_DELAY);
    xSemaphoreGive(baton);

    // Set values to send
    myData.b = random(1,20);
    myData.c = 1.2;
    myData.d = false;
    
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
      Serial.println("ESPNOW: Sent with success");
    }
    else {
      Serial.println("ESPNOW: Error sending the data");
    }
    delay(100);

    // Serial.println("ESPNOWTask Speed: " + String(millis()-start));
  }
}


// core 1 task1 for main function
void TrafficTask( void * pvParameters ){
  for(;;){
    // long start =millis();
    
    xSemaphoreTake(baton,portMAX_DELAY);
    xSemaphoreGive(baton);

    // red1 ON, green2 ON
    traffic_state=0;
    digitalWrite(red_pinout_1,HIGH);
    digitalWrite(yellow_pinout_1,LOW);
    digitalWrite(green_pinout_1,LOW);
    digitalWrite(red_pinout_2,LOW);
    digitalWrite(yellow_pinout_2,LOW);
    digitalWrite(green_pinout_2,HIGH);
    Serial.println("                  TrafficTASK: traffic_state="+String(traffic_state));
    delay(4000);

    // red1 ON, yellow2 ON
    digitalWrite(red_pinout_1,HIGH);
    digitalWrite(yellow_pinout_1,LOW);
    digitalWrite(green_pinout_1,LOW);
    digitalWrite(red_pinout_2,LOW);
    digitalWrite(yellow_pinout_2,HIGH);
    digitalWrite(green_pinout_2,LOW);
    Serial.println("                  TrafficTASK: traffic_state="+String(traffic_state));
    delay(1000);

    // green1 ON, red2 ON
    traffic_state=1;
    digitalWrite(red_pinout_1,LOW);
    digitalWrite(yellow_pinout_1,LOW);
    digitalWrite(green_pinout_1,HIGH);
    digitalWrite(red_pinout_2,HIGH);
    digitalWrite(yellow_pinout_2,LOW);
    digitalWrite(green_pinout_2,LOW);
    Serial.println("                  TrafficTASK: traffic_state="+String(traffic_state));
    delay(4000);

    // yellow1 ON, red2 ON
    digitalWrite(red_pinout_1,LOW);
    digitalWrite(yellow_pinout_1,HIGH);
    digitalWrite(green_pinout_1,LOW);
    digitalWrite(red_pinout_2,HIGH);
    digitalWrite(yellow_pinout_2,LOW);
    digitalWrite(green_pinout_2,LOW);
    Serial.println("                  TrafficTASK: traffic_state="+String(traffic_state));
    delay(1000);

    // Serial.println("TrafficTask Speed: " + String(millis()-start));
  }
}


void loop()
{  
  vTaskDelete(NULL);
}


