TaskHandle_t Task1;
TaskHandle_t Task2;
// TaskHandle_t Task3;

SemaphoreHandle_t baton;

#include "FirebaseESP32.h"
#include <WiFi.h>

String id_car="car_2";
float kp=0;
float ki=0;
float kd=0;
int pid_output=0;
int servo_wip=90;
int motor_speed=0;
int error=0;
int previouserror=0;
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
#define SERVO_PIN            12
// MOTOR CONFIG
#define MOTOR_CHANNEL_0     1
#define MOTOR_TIMER_13_BIT  13
#define MOTOR_BASE_FREQ     8000
#define MOTOR_PIN           14

int servo_value = 255;
int i=1;
int motor_value = 0;
int j=5;


FirebaseData db;
FirebaseJson json;

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
void Task1code( void * pvParameters ){
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
void Task2code( void * pvParameters ){
  for(;;){
    long start =millis();
    
    xSemaphoreTake(baton,portMAX_DELAY);
    xSemaphoreGive(baton);

    ledcWrite(MOTOR_CHANNEL_0, motor_value);
    motor_value = motor_value + j*15;
    if ( motor_value >= 8191 || motor_value <= 0){
        j= -j;
    }

    ledcWrite(SERVO_CHANNEL_0, servo_value);
    servo_value = servo_value + i*15;
    if ( servo_value >= 800 || servo_value <= 255){
        i= -i;
    }


    Serial.println("P: "+String(kp)+" D: "+String(kd)+" I: "+String(ki)+" Motor: "+String(motor_speed)+" Servo: "+String(servo_wip)+" Toggle: "+String(motor_toggle));
    Serial.println("TASK1 Speed: " + String(millis()-start));
  }
}


// core 1 task2 for secondary function
void Task3code( void * pvParameters ){
  for(;;){
    long start =millis();
    
    xSemaphoreTake(baton,portMAX_DELAY);
    xSemaphoreGive(baton);
    
    ledcWrite(SERVO_CHANNEL_0, servo_value);
    servo_value = servo_value + i;
    if ( servo_value >= 800 || servo_value <= 255){
        i= -i;
    }

    Serial.println("TASK2 Speed: " + String(millis()-start));
  }
}


void setup(){
    pinMode(17, OUTPUT);
    pinMode(16, OUTPUT);
    digitalWrite(17,1);
    digitalWrite(16,0);
    ledcSetup(MOTOR_CHANNEL_0, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);
    ledcAttachPin(MOTOR_PIN, MOTOR_CHANNEL_0);
    
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

    // SEMAPHORE để share data
    baton = xSemaphoreCreateMutex();

    //TASK WIFI (NOTE: PRIOPRITY MUST BE 0 OTHERWISE WATCHDOG WILL NOT CATCH UP)
    xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    20,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
    delay(500); 

    //TASK 1 with PRIOR 1
    xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 

    //TASK 2 with PRIOR 1
    // xTaskCreatePinnedToCore(
    //                 Task3code,   /* Task function. */
    //                 "Task3",     /* name of task. */
    //                 10000,       /* Stack size of task */
    //                 NULL,        /* parameter of the task */
    //                 1,           /* priority of the task */
    //                 &Task3,      /* Task handle to keep track of created task */
    //                 1);          /* pin task to core 1 */
    // delay(500);
}

void loop(){
    delay(2000);
    // NOTHING HERE
}