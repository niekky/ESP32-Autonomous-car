// Khai báo task
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

// Semaphore để share data
SemaphoreHandle_t baton;

// WIFI LIBRARY
#include "FirebaseESP32.h"
#include <WiFi.h>
#include <QTRSensors.h>
#include "esp_task_wdt.h"
#include "QuickPID.h"
#include <esp_now.h>

#define FIREBASE_USE_PSRAM

// ID CAR
String id_car="car_2";

QTRSensors qtr;
float kp=0, ki=0, kd=0;
float Setpoint=4000, Input, Output;
float kp_motor=0, ki_motor=0, kd_motor=0;
int pid_output=0;
int servo_wip=90;
int motor_speed=0;
int error=0;
int previouserror=0;
uint16_t position=0;

typedef struct struct_message {
  int b;
  float c;
  bool d;
} struct_message;

struct_message myData;

float previous_error = 0, previous_I = 0;
float previous_error_motor=0, previous_I_motor=0;
const uint8_t SensorCount = 10;
uint16_t sensorValues[SensorCount];

boolean motor_toggle=false;

QuickPID myPID(&Input,&Output,&Setpoint);
// HOST lấy từ Project Settings/Service Accounts/Firebase Admin SDK/databaseURL
#define FIREBASE_HOST "https://nodemcu-a4907-default-rtdb.asia-southeast1.firebasedatabase.app" 
// Auth lấy từ Project Settings/Service Accounts/Database Secrets/Secret
#define FIREBASE_AUTH "frB74idkfdayCS44bsuY0a3WLY59PZtJrxvTUMnD"

// WIFI_SSID: Tên WIFI
#define WIFI_SSID "SCTV-CAM07"
// WIFI_PASSWORD: Tên pass của WIFI
#define WIFI_PASSWORD "1234567899"

// SERVO CONFIG
#define SERVO_CHANNEL_0     0
#define SERVO_TIMER_16_BIT  16
#define SERVO_BASE_FREQ     50
#define SERVO_PIN           21
// MOTOR CONFIG
#define MOTOR_CHANNEL_0     1
#define MOTOR_TIMER_13_BIT  16
#define MOTOR_BASE_FREQ     8000
#define MOTOR_PIN_ENB       13
#define MOTOR_PIN_1         12
#define MOTOR_PIN_2         14

FirebaseData db;
FirebaseJson json;
FirebaseJsonData result;
FirebaseJsonArray arr;
String jsonValues[6];

void SensorCalibrate(){
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){5, 19, 18, 17, 16, 32, 33, 25,26,27}, 10);
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

void readJSONFromDB(){
    json.get(result, "/IDs/car_2");
    result.get<FirebaseJsonArray>(arr);
    for (size_t i = 0; i < arr.size(); i++)
    {
        arr.get(result, i);
        jsonValues[i]=result.to<String>();
    }
    kp=jsonValues[3].toFloat();
    ki=jsonValues[1].toFloat();
    kd=jsonValues[0].toFloat();
    motor_speed=jsonValues[2].toInt();
    servo_wip=jsonValues[4].toInt();
    arr.get(result, 5);
    motor_toggle=result.to<bool>();
}

void SetServoPos(float pos)
{
    uint32_t duty = (((pos/180.0)
              *2000)/20000.0*65536.0) + 1634;
         // convert 0-180 degrees to 0-65536

    ledcWrite(SERVO_CHANNEL_0,duty);
        // set channel to pos
}

void ServoTesting(){
    for(int x=0;x<100;x++)
   {
      SetServoPos(x);
      delay(10);
   }
   delay(100);
   
}

void readFromDB(){
    if (Firebase.getFloat(db,"IDs/"+id_car+"/P_servo")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_float){
        kp=(double) db.to<float>();
      }
    } else {
      Serial.println(db.errorReason());
    }
    if (Firebase.getFloat(db,"/IDs/"+id_car+"/D_servo")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_float){
        kd=(double) db.to<float>();
      }
    } else {
      Serial.println(db.errorReason());
    }
    if (Firebase.getFloat(db,"/IDs/"+id_car+"/I_servo")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_float){
        ki=(double)db.to<float>();
      }
    } else {
      Serial.println(db.errorReason());
    }
    if (Firebase.getFloat(db,"IDs/"+id_car+"/P_motor")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_float){
        kp_motor=db.to<float>();
      }
    } else {
      Serial.println(db.errorReason());
    }
    if (Firebase.getFloat(db,"/IDs/"+id_car+"/D_motor")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_float){
        kd_motor=db.to<float>();
      }
    } else {
      Serial.println(db.errorReason());
    }
    if (Firebase.getFloat(db,"/IDs/"+id_car+"/I_float")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_float){
        ki_motor=db.to<float>();
      }
    } else {
      Serial.println(db.errorReason());
    }
    if (Firebase.getInt(db,"/IDs/"+id_car+"/Motor")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_integer){
        motor_speed=db.to<int>();
      }
    } else {
      Serial.println(db.errorReason());
    }
    if (Firebase.getInt(db,"/IDs/"+id_car+"/Servo")){
      if (db.dataTypeEnum()== fb_esp_rtdb_data_type_integer){
        servo_wip=db.to<int>();
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
 

// core 0 for calling api
void WifiTask( void * pvParameters ){
  for(;;){
    long start =millis();
    xSemaphoreTake(baton,portMAX_DELAY);
    xSemaphoreGive(baton);
    readFromDB();
    Firebase.setFloat(db,"IDs/"+id_car+"_graph/PID_output",random(300));
    Serial.println("TASKWIFI Speed: " + String(millis()-start));
    vTaskDelay(200);
  } 
}

// core 1 task1 for main function
void MainTask( void * pvParameters ){
  for(;;){
    long start =millis();
    
    xSemaphoreTake(baton,portMAX_DELAY);
    xSemaphoreGive(baton);

    // Drive motor
    if (motor_toggle==true){
        digitalWrite(MOTOR_PIN_1,0);
        digitalWrite(MOTOR_PIN_2,1);
        ledcWrite(MOTOR_CHANNEL_0,motor_speed);
    } else{
        digitalWrite(MOTOR_PIN_1,0);
        digitalWrite(MOTOR_PIN_2,0);
        ledcWrite(MOTOR_CHANNEL_0,0);
    }

    // RAW PID FUNCTION
    // position = qtr.readLineBlack(sensorValues);
    // error=3800-position;
    // pid_output = kp*error + kd*(error - previouserror);
    // previouserror = error;

    // PID LIBRARY
    Input = (float) position;
    myPID.SetTunings(kp,ki,kd);
    myPID.Compute();
    pid_output=(int) Output;

    SetServoPos(max(20,min(130,servo_wip-pid_output)));

    //ServoTesting();

    // Chỉ uncomment nếu muốn đọc số liệu, nếu ko thì nên disable vì nó tốn thời gian excecute
    // Serial.println("P: "+String(kp)+" D: "+String(kd*10)+" I: "+String(ki));
    // Serial.println("P_motor: "+String(kp_motor)+" D_motor: "+String(kd_motor)+" I_motor: "+String(ki_motor));
    // Serial.println("Motor: "+String(motor_speed)+" Servo: "+String(servo_wip)+" Toggle: "+String(motor_toggle)+" Position: "+String(position));
    // Serial.println("TASK1 Speed: " + String(millis()-start));
    Serial.println("Input: "+String(Input)+" Output: "+String(pid_output));
    Serial.println("TASK1 Speed: " + String(millis()-start));
  } 
}

void setup(){
    pinMode(MOTOR_PIN_1, OUTPUT);
    pinMode(MOTOR_PIN_2, OUTPUT);
    ledcSetup(MOTOR_CHANNEL_0, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);
    ledcAttachPin(MOTOR_PIN_ENB, MOTOR_CHANNEL_0);
    
    ledcSetup(SERVO_CHANNEL_0, SERVO_BASE_FREQ, SERVO_TIMER_16_BIT);
    ledcAttachPin(SERVO_PIN, SERVO_CHANNEL_0);

    // PID LIBRARY
    myPID.SetMode(myPID.Control::automatic);
    myPID.SetTunings(kp,ki,kd);
    myPID.SetOutputLimits(-180,180);
    myPID.SetSampleTimeUs(50000);

    // Set servo default
    SetServoPos(90);

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


    // //ESPNOW 
    // // Set device as a Wi-Fi Station
    // WiFi.mode(WIFI_STA);

    // // Init ESP-NOW
    // if (esp_now_init() != 0) {
    //   Serial.println("Error initializing ESP-NOW");
    //   return;
    // }
    
    // // Once ESPNow is successfully Init, we will register for recv CB to
    // // get recv packer info
    // esp_now_register_recv_cb(OnDataRecv);

    // Calibrate sensor for a while
    SensorCalibrate();

    // SEMAPHORE để share data
    baton = xSemaphoreCreateMutex();

    // Disable Watch dog timer debugger (vì nó sẽ reboot nếu mạng gặp trục trặc)
    disableCore0WDT();
    disableCore1WDT();
    disableLoopWDT();

    //TASK WIFI
    xTaskCreatePinnedToCore(
                    WifiTask,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    20,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
    delay(500); 
    
    //TASK 1
    xTaskCreatePinnedToCore(
                    MainTask,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    configMAX_PRIORITIES,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}

// DONT USE THIS
void loop(){
    vTaskDelete(NULL);
    // NOTHING HERE
}
