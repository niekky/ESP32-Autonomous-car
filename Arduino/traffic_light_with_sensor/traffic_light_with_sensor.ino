// Khai báo task
TaskHandle_t Task1;
TaskHandle_t Task2;

// Semaphore để share data
SemaphoreHandle_t baton;

#include "FirebaseESP32.h"
#include <WiFi.h>
#include "esp_task_wdt.h"
#define FIREBASE_USE_PSRAM

#define FIREBASE_HOST "https://nodemcu-a4907-default-rtdb.asia-southeast1.firebasedatabase.app" 
#define FIREBASE_AUTH "frB74idkfdayCS44bsuY0a3WLY59PZtJrxvTUMnD"

#define WIFI_SSID "Ku min"
#define WIFI_PASSWORD "01658186379"

#define red_pinout 12
#define yellow_pinout 14
#define green_pinout 27

#include <Wire.h>

unsigned char ok_flag;
unsigned char fail_flag;

unsigned short lenth_val = 0;
unsigned char i2c_rx_buf[16];
unsigned char dirsend_flag=0;
int x=0;
byte traffic_state;

// FirebaseData fbdo;

void setup()
{
  Serial.begin(9600,SERIAL_8N1);
  pinMode(red_pinout,OUTPUT);
  pinMode(yellow_pinout,OUTPUT);
  pinMode(green_pinout,OUTPUT);
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
                  10000,       /* Stack size of task */
                  NULL,        /* parameter of the task */
                  1,           /* priority of the task */
                  &Task2,      /* Task handle to keep track of created task */
                  1);          /* pin task to core 1 */
  delay(500); 
}

//SDA 21 SCL 23 ADDRESS 0x52 
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

void red(){
  // Firebase.RTDB.setString(&fbdo, "traffic/light", "red");
  traffic_state=0;
  digitalWrite(red_pinout,HIGH);
  count(10);
  digitalWrite(red_pinout,LOW);
}

void yellow(){
  // Firebase.RTDB.setString(&fbdo, "traffic/light", "yellow");
  traffic_state=1;
  digitalWrite(yellow_pinout,HIGH);
  count(3);
  digitalWrite(yellow_pinout,LOW);
}

void green(){
  // Firebase.RTDB.setString(&fbdo, "traffic/light", "green");
  traffic_state=2;
  digitalWrite(green_pinout,HIGH);
  count(7);
  digitalWrite(green_pinout,LOW);
}

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
void TrafficTask( void * pvParameters ){
  for(;;){
    long start =millis();
    
    xSemaphoreTake(baton,portMAX_DELAY);
    xSemaphoreGive(baton);

    green();
    yellow();
    red();  

    Serial.println("TrafficTask Speed: " + String(millis()-start));
  }
}


void loop()
{  
  vTaskDelete(NULL);
}


