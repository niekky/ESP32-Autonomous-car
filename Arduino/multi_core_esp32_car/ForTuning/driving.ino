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
#include <esp_now.h>
#include "esp_task_wdt.h"
#define FIREBASE_USE_PSRAM

// ID CAR
String id_car = "car_2";

QTRSensors qtr;
float kp=0, ki=0, kd=0;
float Setpoint=4000, Input, Output;
float kp_motor, ki_motor, kd_motor;
int pid_output=0;
int servo_wip;
int motor_speed;
int sum_err=0;
int error=0;
int count_err=0;
int previouserror=0;
int hall;
uint16_t position=0;

typedef struct struct_message
{
  byte traffic_state;
} struct_message;

struct_message myData;

float previous_error = 0, previous_I = 0;
float previous_error_motor = 0, previous_I_motor = 0, error_motor=0, pid_output_motor=0;
const uint8_t SensorCount = 10;
uint16_t sensorValues[SensorCount];

boolean motor_toggle = false;

long timeStop;

// HOST lấy từ Project Settings/Service Accounts/Firebase Admin SDK/databaseURL
#define FIREBASE_HOST "FIREBASEHOST"
// Auth lấy từ Project Settings/Service Accounts/Database Secrets/Secret
#define FIREBASE_AUTH "FIREBASEAUTH"

// WIFI_SSID: Tên WIFI
#define WIFI_SSID "WIFI_ID"
// WIFI_PASSWORD: Tên pass của WIFI
#define WIFI_PASSWORD "WIFI_PASSWORD"

// SERVO CONFIG
#define SERVO_CHANNEL_0 0
#define SERVO_TIMER_16_BIT 16
#define SERVO_BASE_FREQ 50
#define SERVO_PIN 21
// MOTOR CONFIG
#define MOTOR_CHANNEL_0 1
#define MOTOR_TIMER_13_BIT 16
#define MOTOR_BASE_FREQ 8000
#define MOTOR_PIN_ENB 13
#define MOTOR_PIN_1 12
#define MOTOR_PIN_2 14
#define HALL_PIN 39

FirebaseData db;
FirebaseJson json;
FirebaseJsonData result;
FirebaseJsonArray arr;
String jsonValues[6];

void SensorCalibrate()
{
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){16, 17, 5, 18, 19, 32, 33, 25,  26, 27}, 10);
  qtr.setEmitterPin(4);
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

void SetServoPos(float pos)
{
  uint32_t duty = (((pos / 180.0) * 2000) / 20000.0 * 65536.0) + 1634;
  // convert 0-180 degrees to 0-65536

  ledcWrite(SERVO_CHANNEL_0, duty);
  // set channel to pos
}

void ServoTesting()
{
  for (int x = 0; x < 128; x++)
  {
    SetServoPos(x);
    delay(10);
  }
  delay(500);
}

void windup(){
  sum_err+=error;
  count_err+=1;
  if (count_err>5) count_err=0;
}

void readFromDB(){
  if (Firebase.getFloat(db,"IDs/"+id_car+"/P_servo")){
    if (db.dataTypeEnum()== fb_esp_rtdb_data_type_float){
      kp=db.to<float>();
    }
  } else {
    Serial.println(db.errorReason());
  }
  if (Firebase.getFloat(db, "/IDs/" + id_car + "/D_servo"))
  {
    if (db.dataTypeEnum() == fb_esp_rtdb_data_type_float)
    {
      kd =db.to<float>();
    }
  }
  else
  {
    Serial.println(db.errorReason());
  }
  if (Firebase.getFloat(db, "/IDs/" + id_car + "/I_servo"))
  {
    if (db.dataTypeEnum() == fb_esp_rtdb_data_type_float)
    {
      ki = db.to<float>();
    }
  }
  else
  {
    Serial.println(db.errorReason());
  }
  if (Firebase.getFloat(db, "IDs/" + id_car + "/P_motor"))
  {
    if (db.dataTypeEnum() == fb_esp_rtdb_data_type_float)
    {
      kp_motor = db.to<float>();
    }
  }
  else
  {
    Serial.println(db.errorReason());
  }
  if (Firebase.getFloat(db, "/IDs/" + id_car + "/D_motor"))
  {
    if (db.dataTypeEnum() == fb_esp_rtdb_data_type_float)
    {
      kd_motor = db.to<float>();
    }
  }
  else
  {
    Serial.println(db.errorReason());
  }
  if (Firebase.getFloat(db, "/IDs/" + id_car + "/I_float"))
  {
    if (db.dataTypeEnum() == fb_esp_rtdb_data_type_float)
    {
      ki_motor = db.to<float>();
    }
  }
  else
  {
    Serial.println(db.errorReason());
  }
  if (Firebase.getInt(db, "/IDs/" + id_car + "/Motor"))
  {
    if (db.dataTypeEnum() == fb_esp_rtdb_data_type_integer)
    {
      motor_speed = db.to<int>();
    }
  }
  else
  {
    Serial.println(db.errorReason());
  }
  if (Firebase.getInt(db, "/IDs/" + id_car + "/Servo"))
  {
    if (db.dataTypeEnum() == fb_esp_rtdb_data_type_integer)
    {
      servo_wip = db.to<int>();
    }
  }
  else
  {
    Serial.println(db.errorReason());
  }
  if (Firebase.getBool(db, "/IDs/" + id_car + "/Toggle"))
  {
    if (db.dataTypeEnum() == fb_esp_rtdb_data_type_boolean)
    {
      motor_toggle = db.to<bool>();
    }
  }
  else
  {
    Serial.println(db.errorReason());
  }
}


// core 0 for calling api
void WifiTask(void *pvParameters)
{
  for (;;)
  {
    long start = millis();
    xSemaphoreTake(baton, portMAX_DELAY);
    xSemaphoreGive(baton);
    readFromDB();
    // Firebase.setFloat(db, "IDs/" + id_car + "_graph/PID_output", random(300));
    Serial.println("TASKWIFI Speed: " + String(millis() - start));
    vTaskDelay(200);
  }
}

// core 0 for calling api
void HallTask(void *pvParameters)
{
  for (;;)
  {
    long start = millis();
    xSemaphoreTake(baton, portMAX_DELAY);
    xSemaphoreGive(baton);

    hall=digitalRead(HALL_PIN);

    if (hall==0){
      digitalWrite(MOTOR_PIN_1, 0);
      digitalWrite(MOTOR_PIN_2, 0);
      ledcWrite(MOTOR_CHANNEL_0, 0);
      delay(5000);
    }

    Serial.println("            Hall: "+String(hall));
    Serial.println("TASKWIFI Speed: " + String(millis() - start));
  }
}

// core 1 task1 for main function
void MainTask(void *pvParameters)
{
  for (;;)
  {
    long start = millis();

    xSemaphoreTake(baton, portMAX_DELAY);
    xSemaphoreGive(baton);

    int motor_speed_run = max(5000,motor_speed - (int) abs(pid_output_motor)); 

    // Drive motor
    if (motor_toggle == true)
    {
      digitalWrite(MOTOR_PIN_1, 0);
      digitalWrite(MOTOR_PIN_2, 1);
      ledcWrite(MOTOR_CHANNEL_0, motor_speed_run);
    }
    else
    {
      digitalWrite(MOTOR_PIN_1, 0);
      digitalWrite(MOTOR_PIN_2, 0);
      ledcWrite(MOTOR_CHANNEL_0, 0);
    }

    // Read Hall sensor
    int ki_int = (int) ki_motor;
    // RAW PID FUNCTION
    position = qtr.readLineBlack(sensorValues);
    error=servo_wip-position;
    pid_output = kp*error + kd*(error - previouserror);
    //Serial.println(position);
    // windup();
    previouserror = error;
    error_motor=75-(75-pid_output);
    pid_output_motor=kp_motor*error_motor + kd_motor*(error_motor-previous_error_motor);
    previous_error_motor=error_motor;

    SetServoPos(max(0, min(108, 75 - pid_output)));

    hall=digitalRead(HALL_PIN);

    // ServoTesting();

    // Chỉ uncomment nếu muốn đọc số liệu, nếu ko thì nên disable vì nó tốn thời gian excecute
    // Serial.println("P: "+String(kp)+" D: "+String(kd*10)+" I: "+String(ki));
    // Serial.println("P_motor: "+String(kp_motor)+" D_motor: "+String(kd_motor)+" I_motor: "+String(ki_motor));
    // Serial.println("Motor: "+String(motor_speed)+" Servo: "+String(servo_wip)+" Toggle: "+String(motor_toggle)+" Position: "+String(position));
    // Serial.println("TASK1 Speed: " + String(millis()-start));
    // Serial.println("Input: " + String(Input) + " Output: " + String(pid_output) +" Pos: "+String(position));
    // Serial.println("                    Hall read: "+String(hall));
    // Serial.println("TASK1 Speed: " + String(millis() - start));
  }
}

void setup()
{
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  ledcSetup(MOTOR_CHANNEL_0, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);
  ledcAttachPin(MOTOR_PIN_ENB, MOTOR_CHANNEL_0);

  ledcSetup(SERVO_CHANNEL_0, SERVO_BASE_FREQ, SERVO_TIMER_16_BIT);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL_0);

  // Set servo default
  SetServoPos(90);

  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");

  // Kiểm tra kết nối WIFI
  while (WiFi.status() != WL_CONNECTED)
  {
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

  // Disable Watch dog timer debugger (vì nó sẽ reboot nếu mạng gặp trục trặc)
  disableCore0WDT();
  disableCore1WDT();
  disableLoopWDT();

  timeStop=millis();

  // TASK WIFI
  xTaskCreatePinnedToCore(
      WifiTask, /* Task function. */
      "Task1",  /* name of task. */
      10000,    /* Stack size of task */
      NULL,     /* parameter of the task */
      20,       /* priority of the task */
      &Task1,   /* Task handle to keep track of created task */
      0);       /* pin task to core 0 */
  delay(500);

  // TASK 1
  xTaskCreatePinnedToCore(
      MainTask,             /* Task function. */
      "Task2",              /* name of task. */
      10000,                /* Stack size of task */
      NULL,                 /* parameter of the task */
      configMAX_PRIORITIES, /* priority of the task */
      &Task2,               /* Task handle to keep track of created task */
      1);                   /* pin task to core 1 */
  delay(500);

  // TASK 3
  // xTaskCreatePinnedToCore(
  //     HallTask,             /* Task function. */
  //     "Task3",              /* name of task. */
  //     1000,                /* Stack size of task */
  //     NULL,                 /* parameter of the task */
  //     21, /* priority of the task */
  //     &Task3,               /* Task handle to keep track of created task */
  //     1);                   /* pin task to core 1 */
  // delay(500);
}

// DONT USE THIS
void loop()
{
  vTaskDelete(NULL);
  // NOTHING HERE
}
