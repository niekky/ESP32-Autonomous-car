// Thư viện line sensor
#include <QTRSensors.h>
// Thư viện servo
// #include <Servo.h>
// Thư viện của Firebase và ESP8266WIFI
#include "FirebaseESP32.h"
#include <WiFi.h>

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
boolean motor_toggle=false;

uint16_t position=0;

float previous_error = 0, previous_I = 0;
const uint8_t SensorCount = 10;
uint16_t sensorValues[SensorCount];

string jsonValues[6];


unsigned long previousTime=0;
unsigned long previousTime2=0;

// HOST lấy từ Project Settings/Service Accounts/Firebase Admin SDK/databaseURL
#define FIREBASE_HOST "https://nodemcu-a4907-default-rtdb.asia-southeast1.firebasedatabase.app" 
// Auth lấy từ Project Settings/Service Accounts/Database Secrets/Secret
#define FIREBASE_AUTH "frB74idkfdayCS44bsuY0a3WLY59PZtJrxvTUMnD"

// WIFI_SSID: Tên WIFI
#define WIFI_SSID "SS A20 FREE"
// WIFI_PASSWORD: Tên pass của WIFI
#define WIFI_PASSWORD "19781902Cfc"

// SERVO CONFIG
#define SERVO_CHANNEL_0     0
#define SERVO_TIMER_13_BIT  13
#define SERVO_BASE_FREQ     50
#define SERVO_PIN            5
// MOTOR CONFIG
#define MOTOR_CHANNEL_0     1
#define MOTOR_TIMER_13_BIT  13
#define MOTOR_BASE_FREQ     8000
#define MOTOR_PIN           13

FirebaseData db;
FirebaseJson json;
FirebaseJsonData result;
FirebaseJsonArray arr;

/////////////////////////////////////////////////////////////////////////////////////////

class newpidConfig{
  public:
    void setConfig(float kpp,float kip,float kdp){
      kp=kpp;
      ki=kip;
      kd=kdp;
    }

    float getKP(){
      return kp;
    }

    float getKD(){
      return kd;
    }

    float getKI(){
      return ki;
    }

    float getError(){
      return error;
    }

    float PIDloop(int error){
      float output=kp*error + kd*(error - previous_error);
      previous_error = error;
      return output;
    }

  private:
    float kp;
    float ki;
    float kd;
    int previous_error=0;
};

float pidConfig(float p,float i,float d){
  kp=p;
  ki=i;
  kd=d;
}

// void ServoDefault(){
//   steering.write(servo_wip);
// }

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

void valueChangeDetect(String value,String previous_value){
  if (value!=previous_value){
    Serial.println("LOG: do stuff");
  }
}

void updateMotor(int motor_speed_update){
  if (millis()-previousTime>=100){
    while (motor_speed_update!=motor_speed){
      if (motor_speed < motor_speed_update) motor_speed++;
      else motor_speed--;
    }
    previousTime=millis();
  }
}

void readFromDB(){
  if (millis()-previousTime>=100){
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
    updateMotor(jsonValues[2].toInt());
    servo_wip=jsonValues[4].toInt();
    arr.get(result, 5);
    motor_toggle=result.to<bool>();
    previousTime=millis();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////

// Init PID class
newpidConfig newPIDConfig;

void setup()
{
  // MOTOR ON/OFF PINS
  pinMode(12,OUTPUT);
  pinMode(14,OUTPUT);
  
  ledcSetup(MOTOR_CHANNEL_0, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);
  ledcAttachPin(MOTOR_PIN, MOTOR_CHANNEL_0);
  
  ledcSetup(SERVO_CHANNEL_0, SERVO_BASE_FREQ, SERVO_TIMER_13_BIT);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL_0);
  
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  digitalWrite(16, 1);
  digitalWrite(17, 0);

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
  // SensorCalibrate();
  // pidConfig(0.1,0,0); //0.02 0 0.01
}


int servo_value = 255;
int i=1;
int motor_value = 0;
int j=5;

void loop()
{
  
  ledcWrite(SERVO_CHANNEL_0, servo_value);
  servo_value = servo_value + i;
  if ( servo_value >= 800 || servo_value <= 255){
    i= -i;
  }

  ledcWrite(MOTOR_CHANNEL_0, motor_value);
  motor_value = motor_value + j;
  if ( motor_value >= 8191 || motor_value <= 0){
    j= -j;
  }
  
}
