TaskHandle_t Task1;
TaskHandle_t Task2;


SemaphoreHandle_t baton;

#include "FirebaseESP32.h"
#include <WiFi.h>
#include <QTRSensors.h>

String id_car="car_2";
QTRSensors qtr;
float kp=0;
float ki=0;
float kd=0;
int pid_output=0;
int servo_wip=90;
int motor_speed=0;
int error=0;
int previouserror=0;
uint16_t position=0;

float previous_error = 0, previous_I = 0;
const uint8_t SensorCount = 10;
uint16_t sensorValues[SensorCount];

boolean motor_toggle=false;

// HOST lấy từ Project Settings/Service Accounts/Firebase Admin SDK/databaseURL
#define FIREBASE_HOST "https://nodemcu-a4907-default-rtdb.asia-southeast1.firebasedatabase.app" 
// Auth lấy từ Project Settings/Service Accounts/Database Secrets/Secret
#define FIREBASE_AUTH "frB74idkfdayCS44bsuY0a3WLY59PZtJrxvTUMnD"

// WIFI_SSID: Tên WIFI
#define WIFI_SSID "ABCDEFGH"
// WIFI_PASSWORD: Tên pass của WIFI
#define WIFI_PASSWORD "abcdefgh"

// SERVO CONFIG
#define SERVO_CHANNEL_0     0
#define SERVO_TIMER_13_BIT  13
#define SERVO_BASE_FREQ     50
#define SERVO_PIN           5
// MOTOR CONFIG
#define MOTOR_CHANNEL_0     1
#define MOTOR_TIMER_13_BIT  13
#define MOTOR_BASE_FREQ     8000
#define MOTOR_PIN_ENB       13
#define MOTOR_PIN_1         12
#define MOTOR_PIN_2         14

FirebaseData db;
FirebaseJson json;


void SensorCalibrate(){
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){16, 17, 18, 19, 21, 25, 26, 27,32,33}, 10);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(2, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}


void readFromDB(){
    if (Firebase.getString(db,"IDs/"+id_car+"/P")){
        if (db.dataTypeEnum()== fb_esp_rtdb_data_type_string){
        String dp_kp=db.to<String>();
        kp=dp_kp.toFloat();
        }
    } else {
        Serial.println(db.errorReason());
    }

    if (Firebase.getString(db,"/IDs/"+id_car+"/D")){
        if (db.dataTypeEnum()== fb_esp_rtdb_data_type_string){
        String dp_kd=db.to<String>();
        kd=dp_kd.toFloat();
        }
    } else {
        Serial.println(db.errorReason());
    }

    if (Firebase.getString(db,"/IDs/"+id_car+"/I")){
        if (db.dataTypeEnum()== fb_esp_rtdb_data_type_string){
        String dp_ki=db.to<String>();
        ki=dp_ki.toFloat();
        }
    } else {
        Serial.println(db.errorReason());
    }

    if (Firebase.getString(db,"/IDs/"+id_car+"/Motor")){
        if (db.dataTypeEnum()== fb_esp_rtdb_data_type_string){
        String db_motor_speed=db.to<String>();
        motor_speed=db_motor_speed.toInt();
        }
    } else {
        Serial.println(db.errorReason());
    }

    if (Firebase.getString(db,"/IDs/"+id_car+"/Servo")){
        if (db.dataTypeEnum()== fb_esp_rtdb_data_type_string){
        String db_servo_wip=db.to<String>();
        servo_wip=db_servo_wip.toInt();
        }
    } else {
        Serial.println(db.errorReason());
    }

    if (Firebase.getBool(db,"/IDs/"+id_car+"/Toggle")){
        if (db.dataTypeEnum()== fb_esp_rtdb_data_type_boolean){
        motor_toggle=db.to<bool>();
        }
    } else {
        Serial.println(db.errorReason());
    }
}

// core 0 for calling api
void WifiTask( void * pvParameters ){
  for(;;){
    long start =millis();
    xSemaphoreTake(baton,portMAX_DELAY);
    xSemaphoreGive(baton);
    readFromDB();
    Serial.println("TASKWIFI Speed: " + String(millis()-start));
    vTaskDelay(1000);
  } 
}

// core 1 task1 for main function
void MainTask( void * pvParameters ){
  for(;;){
    // long start =millis();
    
    xSemaphoreTake(baton,portMAX_DELAY);
    xSemaphoreGive(baton);

    if (motor_toggle==true){
        digitalWrite(MOTOR_PIN_1,1);
        digitalWrite(MOTOR_PIN_2,0);
        ledcWrite(MOTOR_CHANNEL_0,motor_speed);
    } else{
        digitalWrite(MOTOR_PIN_1,0);
        digitalWrite(MOTOR_PIN_2,0);
        ledcWrite(MOTOR_CHANNEL_0,0);
    }

    ledcWrite(SERVO_CHANNEL_0,servo_wip);

    position = qtr.readLineBlack(sensorValues);
    // error=position2-position;
    // pid_output = kp*error + kd*(error - previouserror);
    // previouserror = error;

    Serial.println("P: "+String(kp)+" D: "+String(kd)+" I: "+String(ki)+" Motor: "+String(motor_speed)+" Servo: "+String(servo_wip)+" Toggle: "+String(motor_toggle)+" Position: "+String(position));
    // Serial.println("TASK1 Speed: " + String(millis()-start));
  }
}


void setup(){
    pinMode(MOTOR_PIN_1, OUTPUT);
    pinMode(MOTOR_PIN_2, OUTPUT);
    ledcSetup(MOTOR_CHANNEL_0, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);
    ledcAttachPin(MOTOR_PIN_ENB, MOTOR_CHANNEL_0);
    
    ledcSetup(SERVO_CHANNEL_0, SERVO_BASE_FREQ, SERVO_TIMER_13_BIT);
    ledcAttachPin(SERVO_PIN, SERVO_CHANNEL_0);

    Serial.begin(115200);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");

    // Kiểm tra kết nối WIFI
    while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    // Kết nối với Firebase
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    Firebase.reconnectWiFi(true);

    // Calibrate sensor for a while
    SensorCalibrate();

    // SEMAPHORE để share data
    baton = xSemaphoreCreateMutex();

    //TASK WIFI (NOTE: PRIOPRITY MUST BE 0 OTHERWISE WATCHDOG WILL NOT CATCH UP)
    xTaskCreatePinnedToCore(
                    WifiTask,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    20,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
    delay(500); 

    //TASK 1 with PRIOR 1
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

void loop(){
    delay(2000);
    // NOTHING HERE
}